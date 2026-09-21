"""Spawn both arms, jog them with per-arm sliders, and record waypoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vamp_mr_arms.arms import display_nodes, load_config, rviz_node


def generate_launch_description():
    config = load_config()
    output = LaunchConfiguration("output")

    sliders = [Node(package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    namespace=arm["name"], name="joint_state_publisher_gui")
               for arm in config["arms"]]

    teach = Node(package="vamp_mr_arms", executable="teach", name="teach",
                 output="screen", parameters=[{"output": output}])

    return LaunchDescription(
        [DeclareLaunchArgument("output", default_value="waypoints.csv",
                               description="where Save CSV writes the waypoints")]
        + display_nodes(config) + sliders + [rviz_node(), teach])
