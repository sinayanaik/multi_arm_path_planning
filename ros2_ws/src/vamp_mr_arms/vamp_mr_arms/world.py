"""The planner's side of the cell: the VAMP-MR environment, the joint range it accepts,
the routine solved into joint waypoints, and one plan() call per leg."""

import math
import time

import mr_planner_core

FULL_TURN = 2.0 * math.pi

# VAMP's compiled UR5 model permanently includes a Robotiq 85 gripper + FTS300 sensor as
# part of its own link chain (not a runtime attachment) -- these are its link names. Per
# https://github.com/KavrakiLab (VAMP's link_mapping.hh), sphere indices 23-39 of the
# UR5 model's 40 spheres belong to these links; indices 0-22 are the bare arm.
GRIPPER_LINKS = [
    "fts_robotside", "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link", "robotiq_85_left_finger_link",
    "robotiq_85_left_inner_knuckle_link", "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_knuckle_link", "robotiq_85_right_finger_link",
    "robotiq_85_right_inner_knuckle_link", "robotiq_85_right_finger_tip_link",
]


class Invalid(Exception):
    pass


def wrap(joints):
    return [(value + math.pi) % FULL_TURN - math.pi for value in joints]


def base_matrix(base):
    x, y, z = base["xyz"]
    roll, pitch, yaw = base["rpy"]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z],
            [0.0, 0.0, 0.0, 1.0]]


def collision_box(entry):
    box = mr_planner_core.Object()
    box.name = entry["name"]
    box.length, box.width, box.height = entry["size"]
    box.x, box.y, box.z = entry["xyz"]
    return box


def build(config):
    settings = config["planning"]
    environment = mr_planner_core.VampEnvironment(
        config["env_json"], vmax=settings["vmax"], seed=settings["seed"])
    for index, arm in enumerate(config["arms"]):
        environment.set_robot_base_transform(index, base_matrix(arm["base"]))
    for entry in config["scene"]:
        environment.add_object(collision_box(entry))
    for link in GRIPPER_LINKS:
        environment.set_allowed_collision("*", link, True)
    return environment


def strip_object_prefix(name):
    return name[len("object::"):] if name.startswith("object::") else name


def describe_collision(environment, waypoint, index=None, self_only=False):
    pairs = (environment.colliding_links_robot(index, waypoint[index], self_only=self_only)
             if index is not None else environment.colliding_links(waypoint, self_only=self_only))
    if not pairs:
        return None
    pair = pairs[0]
    return f"{strip_object_prefix(pair['link_a'])} hits {strip_object_prefix(pair['link_b'])}"


def collision_reason(environment, arms, waypoint):
    for index, arm in enumerate(arms):
        if environment.in_collision_robot(index, waypoint[index], self_only=True):
            detail = describe_collision(environment, waypoint, index, self_only=True)
            return f"{arm['name']} folds into itself ({detail})" if detail else f"{arm['name']} folds into itself"
        if environment.in_collision_robot(index, waypoint[index]):
            detail = describe_collision(environment, waypoint, index)
            return f"{arm['name']} hits the cell ({detail})" if detail else f"{arm['name']} hits the cell"
    if environment.in_collision(waypoint, self_only=True):
        detail = describe_collision(environment, waypoint, self_only=True)
        return f"the arms hit each other ({detail})" if detail else "the arms hit each other"
    if environment.in_collision(waypoint):
        detail = describe_collision(environment, waypoint)
        return f"the arms hit the cell ({detail})" if detail else "the arms hit the cell"
    return ""


def reject_unplannable(environment, arms, names, waypoints):
    for number, (name, waypoint) in enumerate(zip(names, waypoints), start=1):
        reason = collision_reason(environment, arms, waypoint)
        if reason:
            raise Invalid(f"waypoint {number} ({name}): {reason}")


def target_matrix(tool_down, xyz):
    return [list(tool_down[row][:3]) + [xyz[row]] for row in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def solve_routine(environment, config):
    arms, settings = config["arms"], config["ik"]
    reference = settings["reference"]
    tool_down = [environment.end_effector_transform(index, reference) for index in range(len(arms))]
    padding = [[0.0] * (len(environment.sample_pose(index)) - len(reference))
               for index in range(len(arms))]

    names, waypoints, seeds = [], [], [list(reference) for _ in arms]
    for step in config["routine"]:
        waypoint = []
        for index, arm in enumerate(arms):
            xyz = step[arm["name"]]
            joints = environment.inverse_kinematics(
                index, target_matrix(tool_down[index], xyz),
                seed=seeds[index] + padding[index],
                max_restarts=settings["max_restarts"],
                tol_pos=settings["tol_pos"], tol_ang=settings["tol_ang"])
            if joints is None:
                raise Invalid(f"waypoint {step['name']}: {arm['name']} cannot reach {xyz}")
            waypoint.append(wrap(joints[:len(arm["joints"])]))
        reason = collision_reason(environment, arms, waypoint)
        if reason:
            raise Invalid(f"waypoint {step['name']}: {reason}")
        names.append(step["name"])
        waypoints.append(waypoint)
        seeds = waypoint
    return names, waypoints


def plan_legs(environment, names, waypoints, settings):
    paths = [[] for _ in waypoints[0]]
    times, legs, offset = [], [], 0.0
    for index, (start, goal) in enumerate(zip(waypoints, waypoints[1:])):
        label = f"{names[index]} -> {names[index + 1]}"
        started = time.perf_counter()
        try:
            result = environment.plan(
                planner=settings["planner"], planning_time=settings["planning_time"],
                shortcut_time=settings["shortcut_time"], seed=settings["seed"],
                dt=settings["dt"], vmax=settings["vmax"],
                roadmap_samples=settings["roadmap_samples"],
                roadmap_max_dist=settings["roadmap_max_dist"],
                roadmap_seed=settings["seed"], start=start, goal=goal,
                write_files=False, write_tpg=False, return_trajectories=True)
        except RuntimeError as error:
            raise Invalid(f"leg {label}: {error}") from error
        if not result["times"] or not all(result["traj"]):
            raise Invalid(f"leg {label}: the planner returned an empty trajectory")
        for arm_index, path in enumerate(result["traj"]):
            paths[arm_index].extend(path)
        times.extend(offset + stamp for stamp in result["times"])
        offset = times[-1] + settings["dt"]
        legs.append((label, time.perf_counter() - started, len(result["times"]), result["times"][-1]))
    if any(len(path) != len(times) for path in paths):
        raise Invalid("the planner returned a path that does not match its own timestamps")
    return paths, times, legs
