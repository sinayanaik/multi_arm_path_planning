"""Plan the routine as one coordinated two-arm motion with VAMP-MR, then replay exactly
that trajectory into RViz."""

import csv
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from vamp_mr_arms import world
from vamp_mr_arms.arms import LATCHED, load_config, publish_scene


def read_waypoints(path, arms):
    with Path(path).expanduser().open() as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise world.Invalid(f"{path}: a plan needs at least two waypoints")
    names, waypoints = [], []
    for number, row in enumerate(rows, start=1):
        names.append(row.get("name") or f"row{number}")
        waypoint = []
        for arm in arms:
            columns = [f"{arm['name']}_{joint}_rad" for joint in arm["joints"]]
            if any(row.get(column) is None for column in columns):
                raise world.Invalid(f"{path}: row {number} has no joint columns for {arm['name']}")
            waypoint.append(world.wrap([float(row[column]) for column in columns]))
        waypoints.append(waypoint)
    return names, waypoints


def publish_trajectories(node, arms, paths, times):
    for arm, path in zip(arms, paths):
        message = JointTrajectory()
        message.joint_names = arm["joints"]
        for stamp, configuration in zip(times, path):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in configuration[:len(arm["joints"])]]
            point.time_from_start = Duration(seconds=stamp).to_msg()
            message.points.append(point)
        node.create_publisher(JointTrajectory, f"/{arm['name']}/trajectory", LATCHED).publish(message)


class Replay:
    def __init__(self, node, arms, paths, settings, period):
        self.node, self.arms, self.paths = node, arms, paths
        self.loop, self.step = settings["loop"], 0
        self.publishers = [node.create_publisher(JointState, f"/{arm['name']}/joint_states", 10)
                           for arm in arms]
        node.create_timer(period, self.tick)

    def tick(self):
        last = len(self.paths[0]) - 1
        index = self.step % len(self.paths[0]) if self.loop else min(self.step, last)
        stamp = self.node.get_clock().now().to_msg()
        for arm, publisher, path in zip(self.arms, self.publishers, self.paths):
            message = JointState()
            message.header.stamp = stamp
            message.name = arm["joints"]
            message.position = [float(value) for value in path[index][:len(arm["joints"])]]
            publisher.publish(message)
        self.step += 1


def solve(node, config):
    environment = world.build(config)
    source = node.get_parameter("waypoints").value
    if source:
        names, waypoints = read_waypoints(source, config["arms"])
        world.reject_unplannable(environment, config["arms"], names, waypoints)
    else:
        names, waypoints = world.solve_routine(environment, config)
    node.get_logger().info(f"{len(waypoints)} waypoints: {' -> '.join(names)}")

    paths, times, legs = world.plan_legs(environment, names, waypoints, config["planning"])
    for label, seconds, steps, makespan in legs:
        node.get_logger().info(f"{label}: planned in {seconds * 1000:.0f} ms, {steps} steps, {makespan:.2f} s")
    node.get_logger().info(
        f"{len(paths[0])} steps, {times[-1]:.2f} s, "
        f"collision-free: {not environment.trajectory_in_collision(paths)}")
    return paths, times


def main():
    rclpy.init()
    node = Node("plan")
    node.declare_parameter("config", "")
    node.declare_parameter("waypoints", "")
    config = load_config(node.get_parameter("config").value)
    publish_scene(node, config)

    try:
        paths, times = solve(node, config)
    except world.Invalid as error:
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    publish_trajectories(node, config["arms"], paths, times)
    Replay(node, config["arms"], paths, config["replay"],
           config["planning"]["dt"] / config["replay"]["rate"])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
