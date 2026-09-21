"""Jog the arms with joint_state_publisher_gui, watch each end effector, and record the
poses you like into a waypoint CSV that `plan` turns into a multi-arm trajectory."""

import csv
import math
import signal
import threading
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import ttk

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

from vamp_mr_arms.arms import WORLD_FRAME, load_config

POSE_FIELDS = ["x", "y", "z", "roll", "pitch", "yaw"]
POSE_COLUMNS = ["x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"]
TICK_MS = 50


def quaternion_to_rpy(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Teach(Node):
    def __init__(self):
        super().__init__("teach")
        self.declare_parameter("config", "")
        self.declare_parameter("output", "waypoints.csv")
        self.config = load_config(self.get_parameter("config").value)
        self.output = Path(self.get_parameter("output").value).expanduser().resolve()

        self.history = {arm["name"]: deque(maxlen=64) for arm in self.config["arms"]}
        for arm in self.config["arms"]:
            self.create_subscription(
                JointState, f"/{arm['name']}/joint_states",
                lambda msg, a=arm: self.on_joint_state(a, msg), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.waypoints = []

    def on_joint_state(self, arm, msg):
        by_name = dict(zip(msg.name, msg.position))
        if all(joint in by_name for joint in arm["joints"]):
            self.history[arm["name"]].append(
                (Time.from_msg(msg.header.stamp).nanoseconds,
                 [by_name[joint] for joint in arm["joints"]]))

    def sample(self, arm):
        """Joint values and end-effector pose that belong together: the newest transform
        the listener holds, paired with the joint state it was built from. Taking the
        newest of each independently would log a pose one tick behind the joints."""
        history = self.history[arm["name"]]
        if not history:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, f"{arm['name']}/{arm['ee_frame']}", Time())
        except tf2_ros.TransformException:
            return None
        stamp = Time.from_msg(tf.header.stamp).nanoseconds
        joints = next((q for t, q in reversed(history) if t <= stamp), history[-1][1])
        position = tf.transform.translation
        return joints + [position.x, position.y, position.z,
                         *quaternion_to_rpy(tf.transform.rotation)]

    def record(self, name):
        samples = [self.sample(arm) for arm in self.config["arms"]]
        if any(sample is None for sample in samples):
            return False
        self.waypoints.append((name, samples))
        return True

    def save(self):
        header = ["name"]
        for arm in self.config["arms"]:
            header += [f"{arm['name']}_{joint}_rad" for joint in arm["joints"]]
            header += [f"{arm['name']}_{column}" for column in POSE_COLUMNS]
        with self.output.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for name, samples in self.waypoints:
                writer.writerow([name] + [f"{v:.6f}" for s in samples for v in s])
        self.get_logger().info(f"wrote {len(self.waypoints)} waypoints to {self.output}")


class Toolbar(tk.Tk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.title("VAMP-MR waypoints")
        self.readouts = {}

        for column, arm in enumerate(node.config["arms"]):
            frame = ttk.LabelFrame(self, text=arm["name"], padding=8)
            frame.grid(row=0, column=column, padx=8, pady=8, sticky="nsew")
            self.readouts[arm["name"]] = (
                self._row(frame, 0, [f"J{i + 1}" for i in range(len(arm["joints"]))], "deg"),
                self._row(frame, 1, POSE_FIELDS, f"{arm['ee_frame']}   m / deg"))

        toolbar = ttk.Frame(self, padding=(8, 0, 8, 8))
        toolbar.grid(row=1, column=0, columnspan=len(node.config["arms"]), sticky="ew")
        ttk.Label(toolbar, text="Name:").pack(side="left")
        self.name = tk.StringVar(value="waypoint")
        ttk.Entry(toolbar, textvariable=self.name, width=14).pack(side="left", padx=(2, 10))
        ttk.Button(toolbar, text="Record", command=self.record).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Save CSV", command=self.node.save).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)
        self.status = tk.StringVar(value="waiting for joint states and TF")
        ttk.Label(toolbar, textvariable=self.status).pack(side="left", padx=12)

        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.after(TICK_MS, self.tick)

    def _row(self, parent, row, labels, unit):
        frame = ttk.LabelFrame(parent, text=unit, padding=4)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        variables = []
        for column, label in enumerate(labels):
            ttk.Label(frame, text=label, anchor="center").grid(row=0, column=column, padx=3)
            variable = tk.StringVar(value="-")
            ttk.Label(frame, textvariable=variable, width=9, anchor="e",
                      relief="sunken").grid(row=1, column=column, padx=3)
            variables.append(variable)
        return variables

    def record(self):
        if self.node.record(self.name.get().strip() or f"waypoint{len(self.node.waypoints)}"):
            self.status.set(f"{len(self.node.waypoints)} recorded")
        else:
            self.status.set("waiting for joint states and TF")

    def clear(self):
        self.node.waypoints.clear()
        self.status.set("cleared")

    def tick(self):
        for arm in self.node.config["arms"]:
            joint_labels, pose_labels = self.readouts[arm["name"]]
            sample = self.node.sample(arm)
            if sample:
                split = len(arm["joints"])
                for label, value in zip(joint_labels, sample[:split]):
                    label.set(f"{math.degrees(value):.2f}")
                for label, value in zip(pose_labels, sample[split:split + 3]):
                    label.set(f"{value:.4f}")
                for label, value in zip(pose_labels[3:], sample[split + 3:]):
                    label.set(f"{math.degrees(value):.2f}")
        self.after(TICK_MS, self.tick)


def main():
    rclpy.init()
    node = Teach()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    toolbar = Toolbar(node)
    # Tk's mainloop blocks in C, so ros2 launch's SIGINT would otherwise leave the
    # window orphaned after the rest of the stack has gone.
    signal.signal(signal.SIGINT, lambda *_: toolbar.destroy())
    signal.signal(signal.SIGTERM, lambda *_: toolbar.destroy())
    try:
        toolbar.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
