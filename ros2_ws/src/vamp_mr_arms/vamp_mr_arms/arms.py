"""The ROS side of the cell: the config file, the scene drawn as markers, and the display
nodes both modes launch."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE = "vamp_mr_arms"
WORLD_FRAME = "world"
SCENE_TOPIC = "/scene"
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# VAMP's UR5 model stands on a pedestal and is yawed; the arms from ur_description are
# not. Adding it here keeps the URDFs untouched and matches what the planner sees.
PEDESTAL_Z = 0.9144
PEDESTAL_YAW = 1.57

SECTIONS = ("env_json", "arms", "scene", "display_only", "routine", "ik", "planning", "replay")

ROLE_COLOR = {"table": (0.55, 0.42, 0.30),
              "conveyor": (0.45, 0.47, 0.50),
              "source": (0.25, 0.45, 0.75),
              "target": (0.90, 0.55, 0.15),
              "riser": (0.22, 0.22, 0.25)}


def share_dir():
    return Path(get_package_share_directory(PACKAGE))


def load_config(path=""):
    source = Path(path or share_dir() / "config" / "arms.yaml")
    config = yaml.safe_load(source.read_text())
    missing = [section for section in SECTIONS if section not in config]
    if missing:
        raise SystemExit(f"{source}: missing {', '.join(missing)}")
    for step in config["routine"]:
        for arm in config["arms"]:
            if arm["name"] not in step:
                raise SystemExit(f"{source}: routine step {step['name']} has no {arm['name']} target")
    config["env_json"] = str(share_dir() / "vamp_env" / config["env_json"])
    return config


BIN_WALL = 0.015
BIN_FLOOR = 0.015


def box_marker(entry, marker_id):
    marker = Marker()
    marker.header.frame_id = WORLD_FRAME
    marker.ns, marker.id = entry["role"], marker_id
    marker.type, marker.action = Marker.CUBE, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = entry["xyz"]
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = entry["size"]
    marker.color.r, marker.color.g, marker.color.b = ROLE_COLOR[entry["role"]]
    marker.color.a = 0.9
    return marker


def bin_markers(entry, first_id):
    # The collision box the planner uses stays a single solid volume (world.py); this is
    # the open-top container drawn in its place so it reads as a bin, not a crate.
    length, width, height = entry["size"]
    cx, cy, cz = entry["xyz"]
    bottom = cz - height / 2.0
    parts = [(length, width, BIN_FLOOR, cx, cy, bottom + BIN_FLOOR / 2.0),
             (BIN_WALL, width, height, cx - length / 2.0 + BIN_WALL / 2.0, cy, cz),
             (BIN_WALL, width, height, cx + length / 2.0 - BIN_WALL / 2.0, cy, cz),
             (length, BIN_WALL, height, cx, cy - width / 2.0 + BIN_WALL / 2.0, cz),
             (length, BIN_WALL, height, cx, cy + width / 2.0 - BIN_WALL / 2.0, cz)]
    markers = []
    for offset, (sx, sy, sz, x, y, z) in enumerate(parts):
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns, marker.id = entry["name"], first_id + offset
        marker.type, marker.action = Marker.CUBE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
        marker.color.r, marker.color.g, marker.color.b = ROLE_COLOR[entry["role"]]
        marker.color.a = 0.9
        markers.append(marker)
    return markers


BIN_ROLES = {"source", "target"}


def scene_markers(config):
    markers = MarkerArray()
    next_id = 0
    for entry in config["scene"] + config["display_only"]:
        new_markers = bin_markers(entry, next_id) if entry["role"] in BIN_ROLES else [box_marker(entry, next_id)]
        markers.markers.extend(new_markers)
        next_id += len(new_markers)
    return markers


def publish_scene(node, config):
    publisher = node.create_publisher(MarkerArray, SCENE_TOPIC, LATCHED)
    publisher.publish(scene_markers(config))
    return publisher


def display_nodes(config):
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    xacro = PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"])
    nodes = []
    for arm in config["arms"]:
        xyz, rpy = arm["base"]["xyz"], arm["base"]["rpy"]
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=arm["name"], name="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(
                Command(["xacro ", xacro, " ur_type:=", arm["ur_type"],
                         " name:=", arm["name"]]), value_type=str),
                "frame_prefix": f"{arm['name']}/"}]))
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"{arm['name']}_base", arguments=[
                "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2] + PEDESTAL_Z),
                "--roll", str(rpy[0]), "--pitch", str(rpy[1]),
                "--yaw", str(rpy[2] + PEDESTAL_YAW),
                "--frame-id", WORLD_FRAME, "--child-frame-id", f"{arm['name']}/world"]))
    return nodes


def rviz_node(config_name="arms.rviz"):
    from launch_ros.actions import Node

    return Node(package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", str(share_dir() / "rviz" / config_name)])
