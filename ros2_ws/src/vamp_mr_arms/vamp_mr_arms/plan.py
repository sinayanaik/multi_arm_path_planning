"""Plan the recorded waypoints as one coordinated multi-arm motion with VAMP-MR and
replay it into RViz as joint states."""

import csv
from pathlib import Path

import mr_planner_core
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from vamp_mr_arms.arms import WORLD_FRAME, load_config

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def read_waypoints(path, arms):
    """One row per waypoint: name, then each arm's joints followed by its end-effector
    pose. Only the joint columns are planned with; the pose columns are the human log."""
    with Path(path).open() as handle:
        rows = list(csv.reader(handle))[1:]
    waypoints = []
    for row in rows:
        values, column = [], 1
        for arm in arms:
            values.append([float(v) for v in row[column:column + len(arm["joints"])]])
            column += len(arm["joints"]) + 6
        waypoints.append(values)
    if len(waypoints) < 2:
        raise SystemExit(f"{path}: need at least two waypoints to plan a leg")
    return waypoints


class Plan(Node):
    def __init__(self):
        super().__init__("plan")
        self.declare_parameter("config", "")
        self.declare_parameter("waypoints", "waypoints.csv")
        self.config = load_config(self.get_parameter("config").value)
        self.arms = self.config["arms"]
        settings = self.config["planning"]

        self.environment = mr_planner_core.VampEnvironment(
            self.config["env_json"], vmax=settings["vmax"], seed=settings["seed"])
        for obstacle in self.config["obstacles"]:
            self.environment.add_object(self.make_object(obstacle))

        self.publish_obstacles()
        trajectories = self.solve(
            read_waypoints(self.get_parameter("waypoints").value, self.arms), settings)

        self.joint_publishers = [
            self.create_publisher(JointState, f"/{arm['name']}/joint_states", 10)
            for arm in self.arms]
        self.publish_paths(trajectories, settings["dt"])

        self.trajectories = trajectories
        self.step = 0
        self.create_timer(settings["dt"], self.play)

    @staticmethod
    def make_object(obstacle):
        item = mr_planner_core.Object()
        item.name = obstacle["name"]
        item.length, item.width, item.height = obstacle["size"]
        item.x, item.y, item.z = obstacle["xyz"]
        return item

    def solve(self, waypoints, settings):
        trajectories = [[] for _ in self.arms]
        for leg, (start, goal) in enumerate(zip(waypoints, waypoints[1:])):
            result = self.environment.plan(
                planner=settings["planner"], planning_time=settings["planning_time"],
                shortcut_time=settings["shortcut_time"], seed=settings["seed"],
                dt=settings["dt"], start=start, goal=goal,
                write_files=False, write_tpg=False, return_trajectories=True)
            for index, path in enumerate(result["traj"]):
                trajectories[index].extend(path)
            self.get_logger().info(
                f"leg {leg + 1}/{len(waypoints) - 1}: {result['planner_time_sec']:.2f} s, "
                f"{len(result['traj'][0])} steps")

        self.get_logger().info(
            f"{len(trajectories[0])} steps total, "
            f"collision-free: {not self.environment.trajectory_in_collision(trajectories)}")
        return trajectories

    def publish_obstacles(self):
        markers = MarkerArray()
        for index, obstacle in enumerate(self.config["obstacles"]):
            marker = Marker()
            marker.header.frame_id = WORLD_FRAME
            marker.ns, marker.id, marker.type, marker.action = "obstacles", index, Marker.CUBE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = obstacle["xyz"]
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = obstacle["size"]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.55, 0.42, 0.30, 0.85
            markers.markers.append(marker)
        self.create_publisher(MarkerArray, "obstacles", LATCHED).publish(markers)

    def publish_paths(self, trajectories, dt):
        for arm, path in zip(self.arms, trajectories):
            message = JointTrajectory()
            message.joint_names = arm["joints"]
            for step, configuration in enumerate(path):
                point = JointTrajectoryPoint()
                point.positions = list(configuration)
                point.time_from_start = Duration(seconds=step * dt).to_msg()
                message.points.append(point)
            self.create_publisher(JointTrajectory, f"/{arm['name']}/trajectory",
                                  LATCHED).publish(message)

    def play(self):
        stamp = self.get_clock().now().to_msg()
        for arm, publisher, path in zip(self.arms, self.joint_publishers, self.trajectories):
            message = JointState()
            message.header.stamp = stamp
            message.name = arm["joints"]
            message.position = list(path[self.step % len(path)])
            publisher.publish(message)
        self.step += 1


def main():
    rclpy.init()
    node = Plan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
