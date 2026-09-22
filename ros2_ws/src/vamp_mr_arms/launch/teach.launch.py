"""Spawn both arms, jog them with per-arm sliders, and record waypoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vamp_mr_arms.arms import display_nodes, load_config, rviz_node


def generate_launch_description():
    config = load_config()
    output = LaunchConfiguration("output")
    meshcat = LaunchConfiguration("meshcat")
    meshcat_port = LaunchConfiguration("meshcat_port")

    sliders = [Node(package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    namespace=arm["name"], name="joint_state_publisher_gui")
               for arm in config["arms"]]

    teach = Node(package="vamp_mr_arms", executable="teach", name="teach",
                 output="screen", parameters=[{
                     "output": output,
                     "meshcat": ParameterValue(meshcat, value_type=bool),
                     "meshcat_port": ParameterValue(meshcat_port, value_type=int),
                 }])

    return LaunchDescription(
        [DeclareLaunchArgument("output", default_value="waypoints.csv",
                               description="where Save CSV writes the waypoints"),
         DeclareLaunchArgument("meshcat", default_value="false",
                               description="stream VAMP's actual collision geometry "
                                            "(spheres, including the attached gripper) to a "
                                            "Meshcat bridge running at meshcat_port -- start it "
                                            "first with mr_planner_core/scripts/visualization/"
                                            "meshcat_bridge.py --port <meshcat_port>"),
         DeclareLaunchArgument("meshcat_port", default_value="7600")]
        + display_nodes(config) + sliders + [rviz_node(), teach])
