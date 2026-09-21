# Bimanual UR5 planning with the `mr_planner_core` Python API

Two UR5 arms facing each other across a table, planned as one coordinated multi-robot
problem and played back in meshcat. The demo motion is a short two-handed routine: one
arm presents a part at a steep angle and holds it while the other works around it with
centimetre-scale wrist moves, then both arms swap sides in a single coordinated leg. The
table is real collision geometry, so the planner routes around it.

The point of the routine is that it exercises things a symmetric orbit cannot: the tool
attitude changes by up to 90°, the two arms take asymmetric roles, and they genuinely
cross into each other's space.

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

Planning the whole routine takes a few seconds and yields ~658 trajectory steps, about
66 s of playback at `dt=0.1`. Pass `rate=2.0` to `play_trajectory` if that drags.

## Files

| File | What it is |
|---|---|
| `bases.json` | Where the two arms stand: `translation` + `rpy`, one entry per arm |
| `build_env.sh` | Compiles a 2×UR5 VAMP plugin into `env/` |
| `plan_bimanual.py` | The API demo: waypoint table → IK → plan → play |
| `code_explanation.md` | Line-by-line walkthrough of that script, and the mathematics behind it |
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
SCRIPT = [
    #  name         arm 0 (base at x = +0.45)            arm 1 (base at x = -0.45)
    ("ready",    ( 0.32, -0.22, 1.28,  0,   0,   0), (-0.32,  0.22, 1.28,  0,   0,   0)),
    ("offer",    ( 0.00, -0.12, 1.24,  0,  70,   0), ( 0.00,  0.20, 1.34,  0,  30,  40)),
    ("grip",     HOLD,                               ( 0.00,  0.19, 1.28,  0,  45,  70)),
    ...
]
```

A cell is `(x, y, z, roll, pitch, yaw)` — metres in the world frame, degrees for the
tool attitude — or `HOLD` to leave that arm where it is while the other one moves. Rows
are *synchronised*: each consecutive pair of rows becomes one two-robot `plan()` query,
so both arms start and finish a leg together.

Rows are deliberately **not** evenly spaced. `ready → lift` is a 0.3 m transit that
plans to 154 steps; `grip → twist 1` is one centimetre plus 25° of yaw and plans to 9.
That unevenness is the motion design — long approaches, then slow deliberate wrist work.

To write your own routine, edit `SCRIPT`. Adding a row subdivides a leg, which is the
easiest way to force the tool through a particular intermediate pose. To skip IK
entirely, drop `SCRIPT` and write joint rows directly. Joint rows are **6 wide**,
ordered:

```
shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
```

Arm order follows `robot_groups` in `env/ur5_bimanual.json` (`left_arm`, `right_arm`).

### Orientation is per arm

`(roll, pitch, yaw) = (0, 0, 0)` means "this arm's tool pointing straight down":

```python
DOWN = [np.array(env.end_effector_transform(r, HOME))[:3, :3] for r in (0, 1)]
```

There is one reference per arm on purpose. The bases face each other, 180° apart, so a
single shared world orientation would spend all of one arm's `wrist_3` range just
turning around, and tilted targets on that arm start failing. Per-arm, the same numbers
describe the same gesture for either arm.

### How a row is turned into joint angles

Two small helpers in `plan_bimanual.py` do this, and both details matter:

- `candidates()` returns the IK solutions for one pose **sorted by distance from where
  the arm already is**. A pose has several elbow/wrist branches; taking the nearest one
  keeps the motion tidy and keeps the next query easy. Taking an arbitrary branch strands
  the arm somewhere the planner then has to unwind, and some legs become unplannable.
- `solve_row()` treats a waypoint as a **pair** and checks `env.in_collision([qa, qb])`
  on the combination the arms actually end up in. Checking one arm against the other's
  *previous* configuration rejects perfectly good rows where both arms move, because the
  other arm is about to vacate that space.

The script prints the joint travel and tool separation it picked for each row, so a bad
waypoint shows up before planning starts.

## What the arms can actually do

Measured on this exact setup — worth knowing before authoring waypoints, because most of
these failures look like "IK is broken" rather than "that pose is not available":

| | |
|---|---|
| Shared reachable box | `x, y` within ±0.35, `z` from 1.00 to 1.45. The two arms' workspaces overlap almost completely. |
| Closest the tools can work | **~0.22 m** side by side, ~0.24 m stacked vertically. The Robotiq gripper spheres are fat; below that, every IK branch pair collides. |
| Tool tilt | 0–90° about any axis in the central region. Tilt runs out near the edge of reach — a pose that is fine straight down can be unreachable at 35°. |
| Tool yaw during close work | roughly −45° to +120°. `wrist_3` is clamped to ±π, so a bigger spin forces a whole-arm reconfiguration that then collides with the other arm. |

The script reports the tilt range and closest approach it achieved, so you can see
whether an edit actually used the extra range or quietly gave it up.

### The DOF-7 quirk

Always pass `start`/`goal` explicitly, 6 values per arm. If you omit them,
`plan()` samples a random **7-DOF** problem, because `VampEnvironment.__init__`
sets every robot to DOF 7 and a 6-DOF robot is allowed to keep `dimension + 1`.
Same reason `env.sample_pose(0)` returns 7 numbers, and why the IK helpers pad their
seed with `PAD` zeros and slice the result with `[:6]`.

## The table

`plan_bimanual.py` builds one with the `box()` helper — a top plus four legs,
added through `env.add_object()`:

```python
TABLE_TOP_Z = 0.83
box("table_top", (1.50, 0.80, 0.05), (0.0, 0.0, TABLE_TOP_Z - 0.025))
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
| `composite_rrt` | Default, and the only one that solves this routine cleanly. Plans both arms in the joint configuration space. |
| `cbs_prm` | Conflict-Based Search over per-arm PRM roadmaps. Roadmaps are cached on the `env` object, so reuse one `env` across queries. |
| `priority_sipp_rrt` | Plans arms one at a time, each avoiding the others' committed trajectories. The one to build on for online replanning. |

On this routine, only `composite_rrt` comes through. Swapping the planner is a one-line
edit if you want to see it:

- `cbs_prm` raises `CBS PRM planning failed` — it does not find a solution for the
  interlocked legs.
- `priority_sipp_rrt` returns a trajectory for every leg, but the one for
  `withdraw → cross` does not survive `trajectory_in_collision` (the tools pass within
  0.142 m, under the ~0.22 m floor). Prioritised planning commits the first arm's path
  before the second arm is considered, and that leg is exactly the case where both arms
  need to give way at once.

That is a useful thing to be able to see, which is why the script always runs both the
state check and the swept check and prints the result rather than assuming success:

```python
collision-free: states=True  swept=True
```

`trajectory_in_collision` only tests the sampled states, so the script also sweeps
between consecutive states with `motion_in_collision`.

## Moving the arms around

Edit `bases.json` and re-run `build_env.sh`. To experiment without recompiling,
override at runtime instead:

```python
env.set_robot_base_transforms([tf0, tf1])   # two 4x4 row-major matrices
```

Note `env/ur5_bimanual.json` stores an **absolute** path to the compiled `.so`,
so re-run `build_env.sh` if you move the repo. Changing where the arms stand changes
the reachable box and the `DOWN` references, so expect to retune `SCRIPT`.

## Next steps

- **More obstacles** — same `box()` helper, or see
  `../mr_planner_core/scripts/regression/python_smoke.py` for moving and
  removing objects at runtime (`move_object`, `remove_object`).
- **Carrying something** — `env.attach_object(name, robot_id, joint_positions)` makes a
  box move with the gripper and be collision-checked there, which is what turns the
  `offer`/`grip` rows into an actual handover rather than a mime of one.
- **Tracing a Cartesian path** — the legs here are planner-chosen free motion between
  authored poses. To make the tool follow a straight line, interpolate position and
  orientation yourself and feed each sub-pose through `solve_row` as its own row.
- **Named poses** — write an SRDF and use
  `mr_planner_core.pose_matrix_from_named_pose(...)` instead of literal arrays.
  The SRDF must list `<joint>` elements explicitly; `<chain base_link=... tip_link=...>`
  parses to zero joints.
- **A second arm model (UR5e, UR10e, …)** — needs a VAMP header for that robot,
  built with `foam` (URDF spherization) + `cricket` (`fkcc_gen`), neither of
  which is installed here. The generator can only emit N copies of *one* robot,
  but `VampInstance<...>` is variadic, so a mixed pair is one hand-edited line
  in `env/plugin_src/plugin.cpp`.
