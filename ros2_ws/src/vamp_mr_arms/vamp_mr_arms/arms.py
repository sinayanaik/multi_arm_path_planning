"""Shared plumbing: where the generated VAMP environment lives, how each arm's URDF
becomes a robot_description RViz can render, and the display nodes both modes need."""

import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = "vamp_mr_arms"
WORLD_FRAME = "world"


def share_dir():
    return Path(get_package_share_directory(PACKAGE))


def env_dir():
    return share_dir() / "vamp_env"


def load_config(path=""):
    config = yaml.safe_load(Path(path or share_dir() / "config" / "arms.yaml").read_text())
    config["env_json"] = str(env_dir() / config["env_json"])
    for arm in config["arms"]:
        arm["urdf"] = str(env_dir() / arm["urdf"])
    return config


def robot_description(urdf_path):
    """VAMP's URDFs use a malformed `package://meshes/...` URI and bare relative paths;
    the generated ones use absolute paths. RViz understands none of those, only file://."""
    urdf_path = Path(urdf_path)

    def to_uri(match):
        name = match.group(1)
        if name.startswith("package://") and not name.startswith("package://meshes/"):
            return match.group(0)
        if name.startswith("/"):
            return f'filename="file://{name}"'
        return f'filename="file://{urdf_path.parent / name.removeprefix("package://")}"'

    return re.sub(r'filename="([^"]*)"', to_uri, urdf_path.read_text())


def display_nodes(config):
    from launch_ros.actions import Node

    nodes = []
    for arm in config["arms"]:
        base = arm["base"]
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=arm["name"], name="robot_state_publisher",
            parameters=[{"robot_description": robot_description(arm["urdf"]),
                         "frame_prefix": f"{arm['name']}/"}]))
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"{arm['name']}_base", arguments=[
                "--x", str(base["xyz"][0]), "--y", str(base["xyz"][1]), "--z", str(base["xyz"][2]),
                "--roll", str(base["rpy"][0]), "--pitch", str(base["rpy"][1]), "--yaw", str(base["rpy"][2]),
                "--frame-id", WORLD_FRAME, "--child-frame-id", f"{arm['name']}/offset_link"]))
    return nodes


def rviz_node(config_name="arms.rviz"):
    from launch_ros.actions import Node

    return Node(package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", str(share_dir() / "rviz" / config_name)])
