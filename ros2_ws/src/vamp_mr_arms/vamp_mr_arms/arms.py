"""The ROS side of the cell: the config file, the scene drawn as markers, and the display
nodes both modes launch."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE = "vamp_mr_arms"
WORLD_FRAME = "world"
SCENE_TOPIC = "/scene"
COLLISION_SPHERES_TOPIC = "/collision_spheres"
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

COLLISION_SPHERE_COLOR = (1.0, 0.25, 0.05)
GRIPPER_SPHERE_COLOR = (0.95, 0.85, 0.10)
COLLISION_SPHERE_ALPHA = 0.35


SECTIONS = ("env_json", "arms", "scene", "display_only", "routine", "ik", "planning", "replay")

# Roles are written by tools/mjcf_to_scene.py from the MuJoCo body names; they exist only to
# colour the markers.
ROLE_COLOR = {"cabinet": (0.15, 0.15, 0.17),
              "table": (0.55, 0.42, 0.30),
              "conveyor": (0.45, 0.47, 0.50),
              "source": (0.25, 0.45, 0.75),
              "target": (0.90, 0.55, 0.15),
              "part": (0.80, 0.50, 0.20),
              "fixture": (0.35, 0.35, 0.38)}
FALLBACK_COLOR = (0.5, 0.5, 0.5)


def share_dir():
    return Path(get_package_share_directory(PACKAGE))


# Written by tools/mjcf_to_scene.py from the MuJoCo cell: where the arms are bolted and
# every obstacle around them. Kept apart from arms.yaml so that re-importing the cell cannot
# overwrite a routine you spent an afternoon teaching.
CELL_SECTIONS = ("arms", "scene", "display_only")


def load_config(path=""):
    source = Path(path or share_dir() / "config" / "arms.yaml")
    config = yaml.safe_load(source.read_text())

    cell_path = source.parent / "cell.yaml"
    if not cell_path.is_file():
        raise SystemExit(f"{cell_path}: missing. Run tools/make_env.sh to import the cell "
                         "from models/bimanual_scene.xml.")
    cell = yaml.safe_load(cell_path.read_text())
    for section in CELL_SECTIONS:
        if section not in cell:
            raise SystemExit(f"{cell_path}: missing {section}")
        config[section] = cell[section]

    missing = [section for section in SECTIONS if section not in config]
    if missing:
        raise SystemExit(f"{source}: missing {', '.join(missing)}")
    for step in config["routine"]:
        for arm in config["arms"]:
            if arm["name"] not in step:
                raise SystemExit(f"{source}: routine step {step['name']} has no {arm['name']} target")
    config["env_json"] = str(share_dir() / "vamp_env" / config["env_json"])
    return config


def quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def pose_of(marker, xyz, rpy):
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
    (marker.pose.orientation.x, marker.pose.orientation.y,
     marker.pose.orientation.z, marker.pose.orientation.w) = quaternion(rpy)


def box_marker(entry, marker_id):
    marker = Marker()
    marker.header.frame_id = WORLD_FRAME
    marker.ns, marker.id = entry["role"], marker_id
    marker.type, marker.action = Marker.CUBE, Marker.ADD
    pose_of(marker, entry["xyz"], entry.get("rpy", (0.0, 0.0, 0.0)))
    marker.scale.x, marker.scale.y, marker.scale.z = entry["size"]
    marker.color.r, marker.color.g, marker.color.b = ROLE_COLOR.get(entry["role"], FALLBACK_COLOR)
    marker.color.a = 1.0
    return marker


def mesh_marker(entry, marker_id):
    """The cell's own mesh, where MuJoCo has one, drawn in place of the box.

    The box is still what the planner checks -- this only replaces the cube RViz would
    otherwise draw for it, so the cabinet and conveyor look like themselves instead of like
    the axis-aligned blocks they are approximated by.
    """
    marker = box_marker(entry, marker_id)
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = entry["mesh"]
    marker.mesh_use_embedded_materials = False
    pose_of(marker, entry["mesh_xyz"], entry.get("mesh_rpy", (0.0, 0.0, 0.0)))
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    return marker


def scene_markers(config):
    markers = MarkerArray()
    for marker_id, entry in enumerate(config["scene"] + config["display_only"]):
        maker = mesh_marker if "mesh" in entry else box_marker
        markers.markers.append(maker(entry, marker_id))
    return markers


def publish_scene(node, config):
    publisher = node.create_publisher(MarkerArray, SCENE_TOPIC, LATCHED)
    publisher.publish(scene_markers(config))
    return publisher


def sphere_marker(sphere, marker_id, gripper_prefix=""):
    marker = Marker()
    marker.header.frame_id = WORLD_FRAME
    marker.ns, marker.id = "collision_spheres", marker_id
    marker.type, marker.action = Marker.SPHERE, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
        sphere["x"], sphere["y"], sphere["z"])
    marker.pose.orientation.w = 1.0
    diameter = 2.0 * sphere["radius"]
    marker.scale.x = marker.scale.y = marker.scale.z = diameter
    gripper = bool(gripper_prefix) and sphere["link"].startswith(gripper_prefix)
    marker.color.r, marker.color.g, marker.color.b = (
        GRIPPER_SPHERE_COLOR if gripper else COLLISION_SPHERE_COLOR)
    marker.color.a = COLLISION_SPHERE_ALPHA
    return marker


def collision_sphere_markers(spheres, gripper_prefix=""):
    markers = MarkerArray()
    markers.markers = [sphere_marker(sphere, marker_id, gripper_prefix)
                        for marker_id, sphere in enumerate(spheres)]
    return markers


def publish_collision_spheres(node):
    return node.create_publisher(MarkerArray, COLLISION_SPHERES_TOPIC, 10)


ROBOT_URDF = "models/ur5e.urdf"


def robot_description():
    """The UR5e + 2F85, generated from models/ur5e.xml by tools/mjcf_to_urdf.py.

    Both arms are the same robot, so they share one file and differ only by namespace and
    frame prefix. Read off disk rather than xacro'd: there are no parameters to substitute,
    and this is the file cricket spherized into the model VAMP checks -- reading it directly
    is what keeps the two from being different robots.
    """
    path = share_dir() / ROBOT_URDF
    if not path.is_file():
        raise SystemExit(f"{path}: missing. Run tools/make_env.sh to generate the model.")
    return path.read_text()


def display_nodes(config):
    from launch_ros.actions import Node

    urdf = robot_description()
    nodes = []
    for arm in config["arms"]:
        xyz, rpy = arm["base"]["xyz"], arm["base"]["rpy"]
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=arm["name"], name="robot_state_publisher",
            parameters=[{"robot_description": urdf,
                         "frame_prefix": f"{arm['name']}/"}]))
        # arms.yaml's base is the whole mount transform -- the MuJoCo cell bolts the arm
        # straight to the cabinet, so there is no pedestal offset to add on top of it.
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"{arm['name']}_base", arguments=[
                "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
                "--frame-id", WORLD_FRAME, "--child-frame-id", f"{arm['name']}/base_link"]))
    return nodes


def rviz_node(config_name="arms.rviz"):
    from launch_ros.actions import Node

    return Node(package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", str(share_dir() / "rviz" / config_name)])
