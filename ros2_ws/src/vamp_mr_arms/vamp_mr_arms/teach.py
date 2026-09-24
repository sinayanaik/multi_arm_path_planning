"""Jog the arms with joint_state_publisher_gui, watch each end effector, and record the
poses you like into a waypoint CSV. A pose the planner cannot use cannot be recorded."""

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

from vamp_mr_arms import world
from vamp_mr_arms.arms import (WORLD_FRAME, collision_sphere_markers, load_config,
                                publish_collision_spheres, publish_scene)

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
        self.declare_parameter("meshcat", False)
        self.declare_parameter("meshcat_host", "127.0.0.1")
        self.declare_parameter("meshcat_port", 7600)
        self.config = load_config(self.get_parameter("config").value)
        self.arms = self.config["arms"]
        self.output = Path(self.get_parameter("output").value).expanduser().resolve()
        self.environment = world.build(self.config)
        publish_scene(self, self.config)
        self.collision_spheres_pub = publish_collision_spheres(self)
        self.collision_spheres_supported = True

        self.meshcat_enabled = bool(self.get_parameter("meshcat").value)
        if self.meshcat_enabled:
            host = self.get_parameter("meshcat_host").value
            port = self.get_parameter("meshcat_port").value
            self.environment.enable_meshcat(host, port)
            self.get_logger().info(
                f"meshcat enabled, streaming to {host}:{port} -- this shows VAMP's actual "
                "collision geometry (spheres, including the attached gripper) alongside "
                "RViz's mesh view. Start the bridge first if you haven't: "
                "python3 <mr_planner_core>/scripts/visualization/meshcat_bridge.py "
                f"--port {port} -- its terminal output prints the viewer URL to open in a "
                "browser.")

        self.history = {arm["name"]: deque(maxlen=64) for arm in self.arms}
        for arm in self.arms:
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
                 world.wrap([by_name[joint] for joint in arm["joints"]])))

    def sample(self, arm):
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

    def state(self):
        samples = [self.sample(arm) for arm in self.arms]
        return None if any(sample is None for sample in samples) else samples

    def reason(self, samples):
        return world.collision_reason(
            self.environment, self.arms,
            [sample[:len(arm["joints"])] for arm, sample in zip(self.arms, samples)])

    def push_meshcat(self, samples):
        if self.meshcat_enabled:
            self.environment.set_joint_positions(
                [sample[:len(arm["joints"])] for arm, sample in zip(self.arms, samples)])

    def publish_collision_spheres(self, samples):
        if not self.collision_spheres_supported:
            return
        try:
            spheres = [sphere
                       for index, (arm, sample) in enumerate(zip(self.arms, samples))
                       for sphere in self.environment.robot_spheres(
                           index, sample[:len(arm["joints"])])]
        except AttributeError:
            self.collision_spheres_supported = False
            self.get_logger().warning(
                "environment.robot_spheres() is missing -- mr_planner_core needs a rebuild "
                "and reinstall (cmake --build build -j && sudo cmake --install build) to "
                "pick up the collision-sphere visualization; not publishing /collision_spheres "
                "until then.")
            return
        self.collision_spheres_pub.publish(
            collision_sphere_markers(spheres, world.GRIPPER_PREFIX))

    def record(self, name):
        samples = self.state()
        if samples is None:
            return "waiting for joint states and TF"
        reason = self.reason(samples)
        if reason:
            return f"not recorded: {reason}"
        self.waypoints.append((name, samples))
        return f"{len(self.waypoints)} recorded"

    def save(self):
        header = ["name"]
        for arm in self.arms:
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

        for column, arm in enumerate(node.arms):
            frame = ttk.LabelFrame(self, text=arm["name"], padding=8)
            frame.grid(row=0, column=column, padx=8, pady=8, sticky="nsew")
            self.readouts[arm["name"]] = (
                self._row(frame, 0, [f"J{i + 1}" for i in range(len(arm["joints"]))], "deg"),
                self._row(frame, 1, POSE_FIELDS, f"{arm['ee_frame']}   m / deg"))

        toolbar = ttk.Frame(self, padding=(8, 0, 8, 8))
        toolbar.grid(row=1, column=0, columnspan=len(node.arms), sticky="ew")
        ttk.Label(toolbar, text="Name:").pack(side="left")
        self.name = tk.StringVar(value="waypoint")
        ttk.Entry(toolbar, textvariable=self.name, width=14).pack(side="left", padx=(2, 10))
        ttk.Button(toolbar, text="Record", command=self.record).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Save CSV", command=self.node.save).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)
        self.validity = tk.StringVar(value="waiting for joint states and TF")
        ttk.Label(toolbar, textvariable=self.validity, width=64).pack(side="left", padx=12)
        self.status = tk.StringVar(value="0 recorded")
        ttk.Label(toolbar, textvariable=self.status).pack(side="left")

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
        self.status.set(self.node.record(
            self.name.get().strip() or f"waypoint{len(self.node.waypoints)}"))

    def clear(self):
        self.node.waypoints.clear()
        self.status.set("cleared")

    def tick(self):
        try:
            self._tick()
        except Exception:
            self.node.get_logger().error("tick() failed", exc_info=True)
        self.after(TICK_MS, self.tick)

    def _tick(self):
        samples = self.node.state()
        if samples is None:
            self.validity.set("waiting for joint states and TF")
        else:
            self.node.push_meshcat(samples)
            self.node.publish_collision_spheres(samples)
            reason = self.node.reason(samples)
            self.validity.set(f"cannot plan: {reason}" if reason else "plannable")
            for arm, sample in zip(self.node.arms, samples):
                joint_labels, pose_labels = self.readouts[arm["name"]]
                split = len(arm["joints"])
                for label, value in zip(joint_labels, sample[:split]):
                    label.set(f"{math.degrees(value):.2f}")
                for label, value in zip(pose_labels, sample[split:split + 3]):
                    label.set(f"{value:.4f}")
                for label, value in zip(pose_labels[3:], sample[split + 3:]):
                    label.set(f"{math.degrees(value):.2f}")


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
