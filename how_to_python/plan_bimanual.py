import socket
from pathlib import Path

import numpy as np
import mr_planner_core

HERE = Path(__file__).parent
ENV_JSON = HERE / "env" / "ur5_bimanual.json"
MESHCAT_HOST, MESHCAT_PORT = "127.0.0.1", 7600
TUCKED = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]
TABLE_HEIGHT = 0.83
NUM_ARMS = 2

env = mr_planner_core.VampEnvironment(str(ENV_JSON), vmax=2.5, seed=1)


def meshcat_bridge_is_running():
    with socket.socket() as probe:
        probe.settimeout(0.2)
        return probe.connect_ex((MESHCAT_HOST, MESHCAT_PORT)) == 0


playback = meshcat_bridge_is_running()
if playback:
    env.enable_meshcat(MESHCAT_HOST, MESHCAT_PORT)
else:
    print(f"no meshcat bridge on {MESHCAT_HOST}:{MESHCAT_PORT} - planning only. To watch it, run:")
    print("  python3 ../mr_planner_core/scripts/visualization/meshcat_bridge.py")


def add_box(name, size, position):
    obj = mr_planner_core.Object()
    obj.name = name
    obj.length, obj.width, obj.height = size
    obj.x, obj.y, obj.z = position
    env.add_object(obj)


add_box("table_top", (1.50, 0.80, 0.05), (0.0, 0.0, TABLE_HEIGHT - 0.025))
for sx in (-1, 1):
    for sy in (-1, 1):
        leg_height = TABLE_HEIGHT - 0.05
        add_box(f"table_leg_{sx}_{sy}", (0.07, 0.07, leg_height), (sx * 0.65, sy * 0.32, leg_height / 2))

DOWN = [np.array(env.end_effector_transform(arm, TUCKED))[:3, :3] for arm in range(NUM_ARMS)]
EXTRA_JOINTS = [0.0] * (len(env.sample_pose(0)) - 6)

HOLD = None
READY_POSE = ((0.30, -0.20, 1.30, 0, 0, 0), (-0.30, 0.20, 1.30, 0, 0, 0))
WAYPOINTS = [
    #  name        arm 0 (x, y, z, roll, pitch, yaw)     arm 1 (x, y, z, roll, pitch, yaw)
    ("present",   (0.05, -0.15, 1.15,  0, 40,  0),   HOLD),
    ("inspect",   HOLD,                              ( 0.05,  0.15, 1.15,  0, -30, 30)),
    ("ready",     READY_POSE[0],                     READY_POSE[1]),
]


def rot_x(deg):
    c, s = np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg):
    c, s = np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg):
    c, s = np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def target_pose(arm, x, y, z, roll_deg, pitch_deg, yaw_deg):
    tf = np.eye(4)
    tf[:3, :3] = DOWN[arm] @ rot_z(yaw_deg) @ rot_y(pitch_deg) @ rot_x(roll_deg)
    tf[:3, 3] = (x, y, z)
    return tf


def solve_ik(arm, pose, seed):
    tf = target_pose(arm, *pose).tolist()
    q = env.inverse_kinematics(arm, tf, seed=list(seed) + EXTRA_JOINTS,
                                max_restarts=30, tol_pos=0.01, tol_ang=0.15)
    if q is None:
        raise RuntimeError(f"no IK solution for arm {arm} at {pose}")
    return q[:6]


def solve_waypoint(pose_a, pose_b, previous):
    qa = previous[0] if pose_a is HOLD else solve_ik(0, pose_a, previous[0])
    qb = previous[1] if pose_b is HOLD else solve_ik(1, pose_b, previous[1])
    if env.in_collision([qa, qb]):
        raise RuntimeError("arms collide at this waypoint - adjust the pose")
    return [qa, qb]


HOME = [solve_ik(arm, READY_POSE[arm], TUCKED) for arm in range(NUM_ARMS)]

configs = [HOME]
for name, pose_a, pose_b in WAYPOINTS:
    configs.append(solve_waypoint(pose_a, pose_b, configs[-1]))
    print(f"  {name:9s} arm0={np.round(configs[-1][0], 2)} arm1={np.round(configs[-1][1], 2)}")

trajectories = [[], []]
for start, goal in zip(configs, configs[1:]):
    result = env.plan(
        planner="composite_rrt",
        planning_time=10.0,
        shortcut_time=0.1,
        seed=1,
        start=start,
        goal=goal,
        write_files=False,
        write_tpg=False,
        return_trajectories=True,
    )
    for arm in range(NUM_ARMS):
        trajectories[arm].extend(result["traj"][arm])

print(f"total: {len(trajectories[0])} steps x {len(trajectories[0][0])} joints per arm")
print(f"collision-free: {not env.trajectory_in_collision(trajectories)}")

if playback:
    env.play_trajectory(trajectories, dt=0.1, rate=1.0)
