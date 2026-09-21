# Bimanual UR5 planning with the `mr_planner_core` Python API

Two UR5 arms that start already facing each other over a shared table, planned as one
coordinated multi-robot problem and played back in meshcat. The demo motion is a short
three-waypoint routine with asymmetric roles: arm 0 moves to a tilted "present" pose
while arm 1 holds still, then arm 1 moves in to "inspect" while arm 0 holds, then both
return to ready. The table is real collision geometry, so the planner routes around it.

The point of the routine is to show each `mr_planner_core` API call doing one clear
thing — building the environment, solving IK for a Cartesian pose, checking a pair of
arms for collision, planning between two joint configurations, and playing the result
back — without choreography logic obscuring the calls.

## Run it

```bash
bash build_env.sh                                              # once, ~1 min
```

Start the viewer bridge and leave it running in its own terminal:

```bash
python3 ../mr_planner_core/scripts/visualization/meshcat_bridge.py
```

It takes a second to bind. Once it prints `Listening on ('127.0.0.1', 7600)`,
open <http://127.0.0.1:7000> and plan in a third terminal:

```bash
python3 plan_bimanual.py
```

**The bridge must be up before you run the script.** `enable_meshcat()` is a TCP
*client* — the bridge is what listens on 7600. If it isn't there, the script
says so once and plans without playback, rather than logging a refused
connection for every frame.

Planning the whole routine takes a few seconds and yields ~88 trajectory steps, about
9 s of playback at `dt=0.1` — brisk, since `vmax=2.5` on `VampEnvironment`, and short,
since the arms start at `ready` instead of planning a leg to get there. Pass a lower
`vmax` if you want a slower, more deliberate motion instead.

## Files

| File | What it is |
|---|---|
| `bases.json` | Where the two arms stand: `translation` + `rpy`, one entry per arm |
| `build_env.sh` | Compiles a 2×UR5 VAMP plugin into `env/` |
| `plan_bimanual.py` | The API demo: waypoint table → IK → plan → play |
| `env/`, `out/` | Generated (gitignored) |

## The robot model

`build_env.sh` uses the VAMP UR5 header already installed at
`/usr/local/include/vamp/robots/ur5.hh`, so no URDF processing is needed. That
header is a **UR5 with a Robotiq 85 gripper**: 6 joints, 40 collision spheres,
and every joint clamped to ±π (a real UR5 allows ±2π).

The URDFs in `universal_robots/` are **not used at runtime**. VAMP collision
checking is sphere-based, and meshcat draws those spheres, not the meshes.

## Giving the motion

The motion is a literal table of end-effector poses, not a parametric curve and not
hand-written joint angles. One row per leg:

```python
HOLD = None
READY_POSE = ((0.30, -0.20, 1.30, 0, 0, 0), (-0.30, 0.20, 1.30, 0, 0, 0))
WAYPOINTS = [
    #  name        arm 0 (x, y, z, roll, pitch, yaw)     arm 1 (x, y, z, roll, pitch, yaw)
    ("present",   (0.05, -0.15, 1.15,  0, 40,  0),   HOLD),
    ("inspect",   HOLD,                              ( 0.05,  0.15, 1.15,  0, -30, 30)),
    ("ready",     READY_POSE[0],                     READY_POSE[1]),
]
```

A cell is `(x, y, z, roll_deg, pitch_deg, yaw_deg)` — metres in the world frame, degrees
for the tool attitude — or `HOLD` to leave that arm where it is while the other one
moves. Rows are *synchronised*: each consecutive pair of rows becomes one two-robot
`plan()` query, so both arms start and finish a leg together. There's no leading
`"ready"` row — the arms already start there (see "Starting posture" below).

To write your own routine, edit `WAYPOINTS`. Adding a row subdivides a leg, which is the
easiest way to force the tool through a particular intermediate pose. To skip IK
entirely, drop `WAYPOINTS` and write joint rows directly. Joint rows are **6 wide**,
ordered:

```
shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
```

Arm order follows `robot_groups` in `env/ur5_bimanual.json` (`left_arm`, `right_arm`).

### Orientation is per arm

`(roll, pitch, yaw) = (0, 0, 0)` means "this arm's tool pointing straight down":

```python
DOWN = [np.array(env.end_effector_transform(arm, TUCKED))[:3, :3] for arm in range(NUM_ARMS)]
```

There is one reference per arm on purpose. The bases face each other, 180° apart, so a
single shared world orientation would spend all of one arm's `wrist_3` range just
turning around, and tilted targets on that arm start failing. Per-arm, the same numbers
describe the same gesture for either arm. `TUCKED` is only used to sample *this*
orientation — any joint config works, since only the rotation part of its end-effector
transform is read. It's a bad *position* to actually start from (see below).

### Starting posture

`TUCKED` — a generic UR5 pose with the elbow up and the tool held out to the robot's
side — is a poor place for the routine to actually begin: its end-effector sits at
`x ≈ 0.94`, off the table entirely, so the first leg would be a large, arbitrary swing
in from the side before the routine even starts. Instead, the script solves IK once for
`READY_POSE` — the pose both arms should already be in — and uses *that* as `HOME`:

```python
HOME = [solve_ik(arm, READY_POSE[arm], TUCKED) for arm in range(NUM_ARMS)]
```

`TUCKED` is still useful here purely as an IK seed. The result is a start where both
tools are already over the table pointing down at each other, so the routine's first
leg is `present`, not an unrelated transit to get into position.

### How a row is turned into joint angles

Two small helpers in `plan_bimanual.py` do this:

- `solve_ik()` calls `env.inverse_kinematics()` once per pose, seeded with the arm's
  previous joint configuration so the solver lands on a nearby branch instead of an
  arbitrary one.
- `solve_waypoint()` solves both arms for a row, then checks `env.in_collision([qa, qb])`
  on the pair they actually end up in. Checking one arm against the other's *previous*
  configuration would reject perfectly good rows where both arms move, because the other
  arm is about to vacate that space.

The script prints the joint configuration it picked for each row, so a bad waypoint
shows up before planning starts.

## What the arms can actually do

Measured on this exact setup — worth knowing before authoring waypoints, because most of
these failures look like "IK is broken" rather than "that pose is not available":

| | |
|---|---|
| Shared reachable box | `x, y` within ±0.35, `z` from 1.00 to 1.45. The two arms' workspaces overlap almost completely. |
| Closest the tools can work | **~0.22 m** side by side, ~0.24 m stacked vertically. The Robotiq gripper spheres are fat; below that, every IK branch pair collides. |
| Tool tilt | 0–90° about any axis in the central region. Tilt runs out near the edge of reach — a pose that is fine straight down can be unreachable at 35°. |
| Tool yaw during close work | roughly −45° to +120°. `wrist_3` is clamped to ±π, so a bigger spin forces a whole-arm reconfiguration that then collides with the other arm. |

Worth checking against these numbers when authoring a new waypoint — most IK failures on
this rig are "that pose is outside the box above," not a bug in the script.

### The DOF-7 quirk

Always pass `start`/`goal` explicitly, 6 values per arm. If you omit them,
`plan()` samples a random **7-DOF** problem, because `VampEnvironment.__init__`
sets every robot to DOF 7 and a 6-DOF robot is allowed to keep `dimension + 1`.
Same reason `env.sample_pose(0)` returns 7 numbers, and why `solve_ik()` pads its
seed with `EXTRA_JOINTS` zeros and slices the result with `[:6]`.

## The table

`plan_bimanual.py` builds one with the `add_box()` helper — a top plus four legs,
added through `env.add_object()`:

```python
TABLE_HEIGHT = 0.83
add_box("table_top", (1.50, 0.80, 0.05), (0.0, 0.0, TABLE_HEIGHT - 0.025))
```

These are `mr_planner_core.Object` boxes in the `Static` state, checked by the
planner like any other obstacle — not decoration. Add your own the same way;
`size` and `position` are both `(x, y, z)`.

Why 0.83: the VAMP `ur5.hh` model has a pedestal baked into it, so with a zero
base transform its `base_link` sphere sits at z=0.914 with an 0.08 radius. A top
surface at 0.83 puts the arms on the table without the base spheres intersecting
it. Raise it and you will get a collision at HOME; lower it and the arms float.

## Planners

Pass one of these as `planner=`:

| Value | Use it when |
|---|---|
| `composite_rrt` | Default, and the one used by this demo. Plans both arms together in the joint configuration space. |
| `cbs_prm` | Conflict-Based Search over per-arm PRM roadmaps. Roadmaps are cached on the `env` object, so reuse one `env` across queries. |
| `priority_sipp_rrt` | Plans arms one at a time, each avoiding the others' committed trajectories. The one to build on for online replanning. |

Swapping the planner is a one-line edit (`planner="cbs_prm"` in the `env.plan(...)`
call). `priority_sipp_rrt` commits the first arm's path before planning the second, so
it can struggle on legs where both arms need to give way to each other at once — worth
checking the result against `trajectory_in_collision` regardless of planner, which is
what the script does after planning:

```python
collision-free: True
```

## Moving the arms around

Edit `bases.json` and re-run `build_env.sh`. To experiment without recompiling,
override at runtime instead:

```python
env.set_robot_base_transforms([tf0, tf1])   # two 4x4 row-major matrices
```

Note `env/ur5_bimanual.json` stores an **absolute** path to the compiled `.so`,
so re-run `build_env.sh` if you move the repo. Changing where the arms stand changes
the reachable box and the `DOWN` references, so expect to retune `READY_POSE` and
`WAYPOINTS`.

## Next steps

- **More obstacles** — same `add_box()` helper, or see
  `../mr_planner_core/scripts/regression/python_smoke.py` for moving and
  removing objects at runtime (`move_object`, `remove_object`).
- **Carrying something** — `env.attach_object(name, robot_id, joint_positions)` makes a
  box move with the gripper and be collision-checked there, which is what would turn the
  `present`/`inspect` rows into an actual handover rather than a mime of one.
- **Tracing a Cartesian path** — the legs here are planner-chosen free motion between
  authored poses. To make the tool follow a straight line, interpolate position and
  orientation yourself and feed each sub-pose through `solve_waypoint` as its own row.
- **Named poses** — write an SRDF and use
  `mr_planner_core.pose_matrix_from_named_pose(...)` instead of literal arrays.
  The SRDF must list `<joint>` elements explicitly; `<chain base_link=... tip_link=...>`
  parses to zero joints.
- **A second arm model (UR5e, UR10e, …)** — needs a VAMP header for that robot,
  built with `foam` (URDF spherization) + `cricket` (`fkcc_gen`), neither of
  which is installed here. The generator can only emit N copies of *one* robot,
  but `VampInstance<...>` is variadic, so a mixed pair is one hand-edited line
  in `env/plugin_src/plugin.cpp`.
