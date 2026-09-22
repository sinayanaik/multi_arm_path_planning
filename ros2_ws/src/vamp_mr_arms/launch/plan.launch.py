"""Spawn both arms in the cell and replay the VAMP-MR plan."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vamp_mr_arms.arms import display_nodes, load_config, rviz_node


def generate_launch_description():
    config = load_config()
    waypoints = LaunchConfiguration("waypoints")

    plan = Node(package="vamp_mr_arms", executable="plan", name="plan",
                output="screen", parameters=[{"waypoints": waypoints}])

    return LaunchDescription(
        [DeclareLaunchArgument(
            "waypoints", default_value="",
            description="waypoint CSV from the teach toolbar; empty plans the routine in arms.yaml")]
        + display_nodes(config) + [rviz_node(), plan])
