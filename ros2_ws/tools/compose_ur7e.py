#!/usr/bin/env python3
"""Turn the ur_description UR7e arm into a VAMP-shaped robot: same 0.9144 m pedestal,
same FTS + Robotiq 85 gripper and same +-pi joint limits as vamp's built-in UR5."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from urdf_paths import absolutize_meshes

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
PEDESTAL_XYZ = (0.0, 0.0, 0.9144)
PEDESTAL_RPY = (0.0, 0.0, 1.57)
DROPPED_TAGS = {"ros2_control", "transmission", "gazebo"}


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(rotation):
    pitch = math.asin(-max(-1.0, min(1.0, rotation[2, 0])))
    return (math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
            math.atan2(rotation[1, 0], rotation[0, 0]))


def joint_transform(joint):
    origin = joint.find("origin")
    xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def set_origin(joint, transform):
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    origin.set("xyz", " ".join(f"{v:.9g}" for v in transform[:3, 3]))
    origin.set("rpy", " ".join(f"{v:.9g}" for v in matrix_to_rpy(transform[:3, :3])))


def find_joint(robot, name):
    return next(j for j in robot.iter("joint") if j.get("name") == name)


def root_link(robot):
    children = {j.find("child").get("link") for j in robot.findall("joint")}
    return next(l.get("name") for l in robot.findall("link") if l.get("name") not in children)


def drop_non_kinematic(robot):
    for tag in DROPPED_TAGS:
        for element in robot.findall(tag):
            robot.remove(element)


def drop_world_anchor(robot):
    for joint in [j for j in robot.findall("joint") if j.find("parent").get("link") == "world"]:
        robot.remove(joint)
    for link in [l for l in robot.findall("link") if l.get("name") == "world"]:
        robot.remove(link)


def clamp_arm_limits(robot):
    for joint in robot.findall("joint"):
        if joint.get("name") in ARM_JOINTS:
            limit = joint.find("limit")
            limit.set("lower", f"{-math.pi:.9g}")
            limit.set("upper", f"{math.pi:.9g}")


def add_pedestal(robot):
    joint = ET.Element("joint", {"name": "offset_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "offset_link"})
    ET.SubElement(joint, "child", {"link": root_link(robot)})
    ET.SubElement(joint, "origin", {"xyz": " ".join(map(str, PEDESTAL_XYZ)),
                                    "rpy": " ".join(map(str, PEDESTAL_RPY))})
    robot.insert(0, joint)
    robot.insert(0, ET.Element("link", {"name": "offset_link"}))


def gripper_subtree(donor, attach_from, attach_to):
    """Everything hanging off the donor's ee_link, re-rooted onto our tool0."""
    joints_by_parent = {}
    for joint in donor.findall("joint"):
        joints_by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)
    links_by_name = {link.get("name"): link for link in donor.findall("link")}

    ee_in_tool0 = np.linalg.inv(joint_transform(find_joint(donor, attach_from))) @ \
        joint_transform(find_joint(donor, attach_to))

    elements, frontier = [], ["ee_link"]
    while frontier:
        for joint in joints_by_parent.get(frontier.pop(), []):
            child = joint.find("child").get("link")
            elements += [joint, links_by_name[child]]
            frontier.append(child)

    bridge = ET.Element("joint", {"name": "tool0_to_ee_link", "type": "fixed"})
    ET.SubElement(bridge, "parent", {"link": "tool0"})
    ET.SubElement(bridge, "child", {"link": "ee_link"})
    set_origin(bridge, ee_in_tool0)
    return [links_by_name["ee_link"], bridge] + elements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-urdf", required=True)
    parser.add_argument("--donor-urdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ur-description", required=True)
    args = parser.parse_args()

    robot = ET.parse(args.arm_urdf).getroot()
    donor = ET.parse(args.donor_urdf).getroot()

    drop_non_kinematic(robot)
    drop_world_anchor(robot)
    clamp_arm_limits(robot)
    add_pedestal(robot)

    for element in gripper_subtree(donor, "wrist_3_link-tool0_fixed_joint", "ee_fixed_joint"):
        robot.append(element)
    absolutize_meshes(robot, Path(args.donor_urdf).resolve().parent,
                      {"ur_description": Path(args.ur_description).resolve(),
                       "meshes": Path(args.donor_urdf).resolve().parent / "meshes"})

    robot.set("name", "ur7e")
    ET.indent(robot, "  ")
    Path(args.output).write_text(ET.tostring(robot, encoding="unicode") + "\n")
    print(f"{args.output}: {len(robot.findall('link'))} links, {len(robot.findall('joint'))} joints")


if __name__ == "__main__":
    main()
