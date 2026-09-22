"""The planner's side of the cell: the VAMP-MR environment, the joint range it accepts,
the routine solved into joint waypoints, and one plan() call per leg."""

import math
import time

import mr_planner_core

FULL_TURN = 2.0 * math.pi


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
    return environment


def collision_reason(environment, arms, waypoint):
    for index, arm in enumerate(arms):
        if environment.in_collision_robot(index, waypoint[index], self_only=True):
            return f"{arm['name']} folds into itself"
        if environment.in_collision_robot(index, waypoint[index]):
            return f"{arm['name']} hits the cell"
    if environment.in_collision(waypoint, self_only=True):
        return "the arms hit each other"
    if environment.in_collision(waypoint):
        return "the arms hit the cell"
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
