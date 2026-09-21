"""Two UR5s work a shared bench: one presents a part, the other works around it, then they swap.

The motion is a hand-written table of end-effector poses - position *and* tool
orientation, deliberately unevenly spaced. Long transits sit next to centimetre-scale
wrist work, and either arm can HOLD while the other moves. Each consecutive pair of rows
is its own two-robot query; the results are concatenated into one playback.
"""

import socket
from pathlib import Path

import numpy as np
import mr_planner_core

HERE = Path(__file__).parent
ENV_JSON = HERE / "env" / "ur5_bimanual.json"
MESHCAT = ("127.0.0.1", 7600)

HOME = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]   # both arms park here
TABLE_TOP_Z = 0.83                              # just under the arms' built-in pedestal

env = mr_planner_core.VampEnvironment(str(ENV_JSON), vmax=1.0, seed=1)

with socket.socket() as probe:
    probe.settimeout(0.2)
    playback = probe.connect_ex(MESHCAT) == 0
if playback:
    env.enable_meshcat(*MESHCAT)
else:
    print(f"no meshcat bridge on {MESHCAT[0]}:{MESHCAT[1]} - planning only. To watch it, run:")
    print("  python3 ../mr_planner_core/scripts/visualization/meshcat_bridge.py")


def box(name, size, position):
    """Add a static box obstacle. size and position are (x, y, z)."""
    obj = mr_planner_core.Object()
    obj.name = name
    obj.length, obj.width, obj.height = size
    obj.x, obj.y, obj.z = position
    env.add_object(obj)


# A table the arms stand on. Real collision geometry, so the planner avoids it.
box("table_top", (1.50, 0.80, 0.05), (0.0, 0.0, TABLE_TOP_Z - 0.025))
for sx in (-1, 1):
    for sy in (-1, 1):
        leg = TABLE_TOP_Z - 0.05
        box(f"table_leg_{sx}_{sy}", (0.07, 0.07, leg), (sx * 0.65, sy * 0.32, leg / 2))

# Each arm's own tool-down attitude. The bases face each other, so a single shared
# orientation would spend all of one arm's wrist_3 range (clamped to +-pi) just turning
# around. Per-arm, (roll, pitch, yaw) = (0, 0, 0) means "straight down" for either arm.
DOWN = [np.array(env.end_effector_transform(r, HOME))[:3, :3] for r in (0, 1)]
PAD = len(env.sample_pose(0)) - 6   # env reports DOF 7 until a plan pins it to 6
rng = np.random.default_rng(7)

# The motion. One row per leg: a name, then a pose per arm, or HOLD to stay put.
# A pose is (x, y, z in metres, roll, pitch, yaw in degrees about that arm's DOWN).
# Reachable envelope, measured: x, y within +-0.35, z in 1.00..1.45, and the two tools
# no closer than ~0.22 m - the grippers are fat. Keep yaw inside about -45..+120 for
# close work; a bigger spin hits the wrist_3 clamp and reconfigures the whole arm.
HOLD = None
SCRIPT = [
    #  name         arm 0 (base at x = +0.45)            arm 1 (base at x = -0.45)
    ("ready",    ( 0.32, -0.22, 1.28,  0,   0,   0), (-0.32,  0.22, 1.28,  0,   0,   0)),
    ("lift",     ( 0.16, -0.10, 1.40,  0,  20,   0), (-0.30,  0.16, 1.10,  0, -30,   0)),
    # arm 0 tips the part 70 degrees off vertical and holds it there for four legs
    ("offer",    ( 0.00, -0.12, 1.24,  0,  70,   0), ( 0.00,  0.20, 1.34,  0,  30,  40)),
    ("grip",     HOLD,                               ( 0.00,  0.19, 1.28,  0,  45,  70)),
    # centimetre-scale wrist work: position barely moves, attitude does all the talking
    ("twist 1",  HOLD,                               ( 0.01,  0.19, 1.27,  0,  55,  95)),
    ("twist 2",  HOLD,                               ( 0.00,  0.20, 1.29,  0,  40, 115)),
    ("withdraw", ( 0.22, -0.26, 1.12,  0,   0,   0), ( 0.02,  0.18, 1.40,  0,  10, 115)),
    # both arms swap x and y half-spaces at once - this is the leg that needs a planner
    ("cross",    (-0.06,  0.20, 1.32,  0,  35, -60), ( 0.10, -0.14, 1.20,  0, -40,  30)),
    ("inspect",  HOLD,                               ( 0.06, -0.10, 1.38,  0, -70,   0)),
    ("clear",    ( 0.26, -0.16, 1.34,  0,  10,   0), (-0.20,  0.06, 1.32,  0,  10,   0)),
    ("park",     ( 0.32, -0.22, 1.28,  0,   0,   0), (-0.32,  0.22, 1.28,  0,   0,   0)),
]


def rot(axis, deg):
    c, s = np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def target(robot_id, x, y, z, roll, pitch, yaw):
    """A table row's pose -> the 4x4 end-effector transform IK aims at."""
    tf = np.eye(4)
    tf[:3, :3] = DOWN[robot_id] @ rot("z", yaw) @ rot("y", pitch) @ rot("x", roll)
    tf[:3, 3] = (x, y, z)
    return tf


def candidates(robot_id, spec, seed, n=10):
    """IK branches for one pose, nearest to seed first.

    A pose has several elbow/wrist solutions. Sorting them by distance from where the arm
    already is keeps the motion tidy and keeps the next query easy to plan - picking any
    old branch strands the arm somewhere the planner then has to unwind.
    """
    tf = target(robot_id, *spec).tolist()
    out = []
    for k in range(n):
        start = seed if k == 0 else list(np.array(seed) + rng.uniform(-2.0, 2.0, 6))
        q = env.inverse_kinematics(robot_id, tf, seed=list(start) + [0.0] * PAD,
                                   max_restarts=1 if k == 0 else 30,
                                   tol_pos=0.012, tol_ang=0.15)
        if q is None:
            continue
        q = q[:6]
        if not env.in_collision_robot(robot_id, q + [0.0] * PAD):
            out.append(q)
    return sorted(out, key=lambda q: np.abs(np.array(q) - np.array(seed)).max())


def solve_row(arm0, arm1, previous):
    """A waypoint is a *pair*: the arms have to clear each other where they end up.

    Checking one arm against the other's previous configuration would reject rows where
    both move, since the other arm is about to vacate that space.
    """
    options = [[previous[r]] if spec is HOLD else candidates(r, spec, previous[r])
               for r, spec in ((0, arm0), (1, arm1))]
    best, best_travel = None, None
    for qa in options[0][:6]:
        for qb in options[1][:6]:
            if env.in_collision([qa, qb]):
                continue
            travel = (np.abs(np.array(qa) - np.array(previous[0])).max()
                      + np.abs(np.array(qb) - np.array(previous[1])).max())
            if best_travel is None or travel < best_travel:
                best, best_travel = [qa, qb], travel
    return best, best_travel


def tool_gap(pair):
    ee = [np.array(env.end_effector_transform(r, pair[r]))[:3, 3] for r in (0, 1)]
    return np.linalg.norm(ee[0] - ee[1])


rows, config = [], [HOME[:], HOME[:]]
for name, arm0, arm1 in SCRIPT:
    config, travel = solve_row(arm0, arm1, config)
    if config is None:
        raise RuntimeError(f"no collision-free pair of IK solutions for row '{name}'")
    print(f"  {name:9s} joint travel {travel:4.2f} rad   tools {tool_gap(config):.3f} m apart")
    rows.append(config)

traj = [[], []]
for i, (start, goal) in enumerate(zip(rows, rows[1:])):
    res = env.plan(
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
    for arm in (0, 1):
        traj[arm].extend(res["traj"][arm])
    print(f"  leg {i + 1:2d}/{len(rows) - 1} {SCRIPT[i][0]:9s} -> {SCRIPT[i + 1][0]:9s}"
          f" {len(res['traj'][0]):4d} steps")

print(f"total: {len(traj[0])} steps x {len(traj[0][0])} joints per arm")

# trajectory_in_collision only checks the sampled states; also sweep between them.
swept = any(env.motion_in_collision([traj[0][i], traj[1][i]],
                                    [traj[0][i + 1], traj[1][i + 1]], 0.01, False)
            for i in range(len(traj[0]) - 1))
print(f"collision-free: states={not env.trajectory_in_collision(traj)}  swept={not swept}")

# How much of the arms' dexterity the motion actually used, rather than just claiming it.
tilts = [np.rad2deg(np.arccos(np.clip(
             -np.array(env.end_effector_transform(arm, traj[arm][i]))[2, 2], -1, 1)))
         for i in range(0, len(traj[0]), 3) for arm in (0, 1)]
gaps = [tool_gap([traj[0][i], traj[1][i]]) for i in range(0, len(traj[0]), 3)]
print(f"tool tilt from vertical: {min(tilts):.0f} to {max(tilts):.0f} deg"
      f"   closest approach: {min(gaps):.3f} m")

if playback:
    env.play_trajectory(traj, dt=0.1, rate=1.0)
