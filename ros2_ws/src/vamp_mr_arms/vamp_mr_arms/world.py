"""The planner's side of the cell: the VAMP-MR environment, the joint range it accepts,
the routine solved into joint waypoints, and one plan() call per leg."""

import json
import math
import time
from pathlib import Path

import mr_planner_core

FULL_TURN = 2.0 * math.pi

# The arm is bolted to the cabinet, so these two links are inside it by construction and
# would otherwise be in collision at every pose. Two reasons it is these two and no more:
# the cabinet's top face is at 0.8635 and the arms mount 0.2 mm above it, and any sphere
# covering the base casting -- a squat cylinder 99 mm tall and 151 mm across -- necessarily
# bulges well past that face. Allowing the contact keeps the cabinet's true height in the
# planner instead of truncating it to a shape the arm happens to clear.
MOUNTED_LINKS = ["base_link", "shoulder_link"]

# tools/mjcf_to_urdf.py renames the 2F85's bodies out of the way of the arm's own `base_link`.
# Unlike the old UR5 model's gripper, this one is on screen and is checked like any other
# link -- the prefix is only used to colour it differently in /collision_spheres.
GRIPPER_PREFIX = "gripper_"


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


def quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def collision_box(entry):
    box = mr_planner_core.Object()
    box.name = entry["name"]
    box.length, box.width, box.height = entry["size"]
    box.x, box.y, box.z = entry["xyz"]
    # The bins, the table and every block in the cell carry a 90 degree yaw in the MuJoCo
    # scene, so an entry's orientation is not optional here the way it was when the cell was
    # hand-written and axis-aligned.
    box.qx, box.qy, box.qz, box.qw = quaternion(entry.get("rpy", (0.0, 0.0, 0.0)))
    return box


# The cabinet entry mjcf_to_scene.py writes for the body the arms are bolted to.
MOUNT_OBJECT = "robot_base"


def check_plugin(env_json):
    """Say plainly that the model has not been generated, before the loader says it badly.

    mr_planner_core loads the plugin through dlopen; a missing .so surfaces as a C++
    exception naming a path, with no hint that it is generated or by what.
    """
    path = Path(env_json)
    if not path.is_file():
        raise Invalid(f"{path} is missing -- run tools/make_env.sh")
    plugin = Path(json.loads(path.read_text()).get("vamp_plugin", ""))
    if not plugin.is_file():
        raise Invalid(
            f"the VAMP plugin {plugin.name} is missing. It is generated from "
            "models/ur5e.xml by tools/make_env.sh, which needs cricket's fkcc_gen "
            "(see the header of that script).")


def build(config):
    settings = config["planning"]
    check_plugin(config["env_json"])
    environment = mr_planner_core.VampEnvironment(
        config["env_json"], vmax=settings["vmax"], seed=settings["seed"])
    for index, arm in enumerate(config["arms"]):
        environment.set_robot_base_transform(index, base_matrix(arm["base"]))
    names = set()
    for entry in config["scene"]:
        environment.add_object(collision_box(entry))
        names.add(entry["name"])
    if MOUNT_OBJECT not in names:
        raise Invalid(f"the scene has no {MOUNT_OBJECT!r} for the arms to stand on; "
                      "re-run tools/mjcf_to_scene.py")
    for link in MOUNTED_LINKS:
        environment.set_allowed_collision(MOUNT_OBJECT, link, True)
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
