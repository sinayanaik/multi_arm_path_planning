# ROS 2 workspace for VAMP-MR

A two-arm pick-and-place cell: a UR5 and a UR7e mounted side by side on a work table, a
conveyor in front of them carrying two source bins, and a target box on their own table.
Both arms sweep from the bins to the box as **one coordinated motion planned by VAMP-MR**,
and RViz replays exactly the trajectory the planner returned.

Nothing in the parent repository is modified, and no URDF is edited anywhere: `vamp` and
`mr_planner_core` are used from their existing `/usr/local` install, and both arms are
displayed straight from the vendored `ur_description`.

```bash
tools/make_env.sh                       # once: build the VAMP plugin + environment JSON
colcon build --symlink-install
source install/setup.bash

ros2 launch vamp_mr_arms plan.launch.py                         # plan and replay the routine
ros2 launch vamp_mr_arms teach.launch.py output:=routine.csv    # jog and record your own
ros2 launch vamp_mr_arms plan.launch.py waypoints:=routine.csv  # replay what you recorded
```

## The cell

```
        y
        ^                     conveyor  belt top 0.78, 1.80 x 0.50
        |        +-------------------------------------------+
        |        |   [ bin_left ]             [ bin_right ]  |   0.20 x 0.20 x 0.10, lid 0.88
        |        +-------------------------------------------+
        |                        y = +0.35
        |        +-------------------------------------------+
        |        |               [ target_box ]              |   0.26 x 0.26 x 0.11, rim 0.94
        |        |                                           |
        |        |      (ur5)                    (ur7e)      |   bases x = -0.35 / +0.35
        |        +-------------------------------------------+
        |               work table  top 0.83, 1.50 x 0.80,  y = -0.25
        +--------------------------------------------------------> x
```

Both arms carry `rpy: [0, 0, 0]`, so they are mounted in the **same orientation** and both
face the conveyor: `shoulder_pan = 0` points along +y.

| Feature | Footprint (m) | Top surface (m) | Centre (m) |
|---|---|---|---|
| work table | 1.50 x 0.80 | 0.83 | `(0.00, -0.25)` |
| conveyor belt | 1.80 x 0.50 | 0.78 | `(0.00, +0.35)` |
| `bin_left` / `bin_right` | 0.20 x 0.20 | 0.88 | `(-+0.35, +0.35)` |
| `target_box` | 0.26 x 0.26 | 0.94 | `(0.00, -0.02)` |
| arm mounting flange | — | 0.9144 | `(-+0.35, -0.25)` |

The routine is `home → over_bins → at_bins → lift → over_target → place → retreat`, written
in `config/arms.yaml` as one end-effector position per arm per waypoint. Each consecutive
pair becomes **one two-arm `plan()` query**, so both arms start and finish every leg
together and VAMP-MR is the only thing deciding how they avoid each other.

Measured on this machine: 7 waypoints, 6 legs, **~110 ms per leg**, 51 samples,
**4.92 s** of motion, `trajectory_in_collision` → `False`.

## What the planner actually sees

VAMP ships exactly one arm model — a UR5 with a Robotiq 85 gripper on a 0.9144 m pedestal —
so **both arms are planned as that UR5**. Four consequences are worth knowing, because every
number in `arms.yaml` follows from them.

**1. The configuration space is ±π per joint, not ±2π.**
`/usr/local/include/vamp/robots/ur5.hh` scales the unit cube with `s_m = 2π`, `s_a = -π`. A
waypoint outside that box cannot be planned even when it is perfectly collision-free, and
the failure surfaces only as `RuntimeError: Planning failed`. Every joint value entering the
planner is therefore wrapped into ±π (`world.wrap`), which is a no-op geometrically since a
revolute joint at `q` and at `q ± 2π` is the same pose. The
`joint_state_publisher_gui` sliders follow `ur_description`, which allows ±2π — that is why
wrapping happens on the way in rather than being left to you.

**2. The arm stands on a pedestal the URDFs do not have.**
`base.xyz` in `arms.yaml` is the **foot** of that pedestal; `base_link` ends up 0.9144 m
above it, yawed 1.57 rad. `arms.py` adds exactly that offset to each arm's static transform,
so the display lines up with the planner while the URDFs stay untouched. Verified: for the
same joint values, the world-frame distance between the displayed `tool0` and the planner's
end-effector is a constant **0.0350 m**, which is precisely the `tool0` →
`robotiq_85_base_link` offset in VAMP's own `ur5.urdf`.

**3. The table top cannot go above 0.834 m.**
The model's base sphere has r = 0.08 centred on `base_link` at z = 0.9144, so its underside
sits at 0.8344. A top at 0.86 already collides at the tucked pose. The table top is at
**0.83**, and the 84 mm from there up to the flange is drawn as a `riser` marker — that
volume is already occupied by the robot's own base sphere, so it must never be handed to the
planner as an obstacle. `display_only` entries in `arms.yaml` exist for exactly this.

**4. The gripper reserves about 0.22 m above any surface.**
The planner checks a gripper neither arm on screen is wearing, so its clearances err toward
caution. The lowest reachable end-effector heights measured in this cell are **1.05 m** over
a bin lid (0.88) and **1.10 m** over the target box rim (0.94); the routine uses 1.06 and
1.11. That is why `at_bins` and `place` hover rather than dip — this cell represents the
pick-and-place motion, it does not grasp or attach anything.

Also worth knowing: the plugin reports **7 DOF** per arm (six joints plus the gripper
finger). A trailing seventh value is accepted and ignored by the collision model, so the
nodes work in six joints throughout and only pad the IK seed, which is length-checked.

The UR7e's links differ from the UR5's — shoulder 73 mm taller, wrists 5-24 mm longer (upper
arm identical, forearm within 0.05 mm) — so arm 2's clearance is approximate. A genuine UR7e
collision model would need `foam` + `cricket` codegen; see `IMPLEMENTATION.md`.

## Layout

| Path | What it is |
|---|---|
| `src/vamp_mr_arms/config/arms.yaml` | the cell, the routine and every planner parameter |
| `src/vamp_mr_arms/vamp_mr_arms/world.py` | the planner side: environment, wrapping, validity, IK, `plan()` per leg |
| `src/vamp_mr_arms/vamp_mr_arms/arms.py` | the ROS side: config loading, scene markers, display nodes |
| `src/vamp_mr_arms/vamp_mr_arms/plan.py` | plan the waypoints, publish the trajectories, replay them |
| `src/vamp_mr_arms/vamp_mr_arms/teach.py` | jog, watch, and record waypoints the planner can use |
| `src/vamp_mr_arms/vamp_env/` | generated: the VAMP plugin and the environment JSON |
| `src/ur_description/` | vendored Universal Robots descriptions, unmodified |
| `tools/` | `make_env.sh` and the one script it drives |

## Teaching your own waypoints

```bash
ros2 launch vamp_mr_arms teach.launch.py output:=routine.csv
```

Both arms appear in the cell with a slider window each. The toolbar shows every joint, the
live end-effector pose, and — the part that matters — whether the current pose is
**plannable**. `Record` refuses a pose the planner cannot use and says why (`ur7e folds into
itself`, `the arms hit each other`, `ur5 hits the cell`), so a saved CSV is plannable by
construction. `Save CSV` writes it out.

One row per waypoint: `name`, then each arm's six joints (rad, already wrapped into ±π)
followed by its end-effector pose (m and rad). Only the joint columns are planned with; the
pose columns are there for you to read. Each pose is looked up at its own joint state's
timestamp, so the pose in a row always belongs to the joints beside it.

`plan.launch.py waypoints:=routine.csv` reads those joint columns **by header name**, wraps
them, checks every waypoint, and refuses the whole run with one line if any waypoint is
unusable — it never plans a different motion than the one you recorded.

## Parameters

Everything lives in `src/vamp_mr_arms/config/arms.yaml`.

| Key | Unit | Effect |
|---|---|---|
| `env_json` | — | environment file under `vamp_env/`, names the plugin and the robot groups |
| `arms[].name` | — | ROS namespace, TF prefix and CSV column prefix |
| `arms[].ur_type` | — | which `ur_description` model is displayed (`ur5`, `ur7e`, …) |
| `arms[].base.xyz` | m | foot of the VAMP pedestal; `base_link` lands 0.9144 m higher |
| `arms[].base.rpy` | rad | base orientation; the display adds 1.57 rad of yaw |
| `arms[].ee_frame` | — | frame the teach toolbar reads the pose of |
| `arms[].joints` | — | joint names, in planner order |
| `scene[]` | m | collision boxes: `size` is full extent, `xyz` is the centre |
| `scene[].role` | — | marker colour: `table`, `conveyor`, `source`, `target` |
| `display_only[]` | m | drawn in RViz, never given to the planner |
| `routine[]` | m | one end-effector position per arm per waypoint, world frame |
| `ik.reference` | rad | first IK seed, and the source of the tool-down orientation |
| `ik.max_restarts` | — | IK restarts before a waypoint is declared unreachable |
| `ik.tol_pos` / `ik.tol_ang` | m / rad | IK convergence tolerances |
| `planning.planner` | — | `composite_rrt`, `cbs_prm` or `priority_sipp_rrt` |
| `planning.planning_time` | s | per-leg planning budget |
| `planning.shortcut_time` | s | per-leg shortcutting budget; dominates the ~110 ms per leg |
| `planning.vmax` | rad/s | joint speed the trajectory is timed for |
| `planning.dt` | s | spacing of trajectory samples, and the replay tick |
| `planning.seed` | — | fixes the sampler and the roadmap; the same run twice is identical |
| `planning.roadmap_samples` / `roadmap_max_dist` | — / rad | `cbs_prm` roadmap only |
| `replay.rate` | x | replay speed; the timer runs at `dt / rate` |
| `replay.loop` | bool | loop the motion, or hold the final pose |

Measured effect of the two you are most likely to touch: `vmax: 2.5` → 4.92 s of motion,
`vmax: 0.5` → 22.50 s. All three planners solve this cell (`composite_rrt` 4.92 s,
`cbs_prm` 4.60 s, `priority_sipp_rrt` 4.78 s).

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/scene` | `MarkerArray` | latched, one cube per `scene` + `display_only` entry |
| `/<arm>/joint_states` | `JointState` | the replay, at `dt / rate` (10 Hz by default) |
| `/<arm>/trajectory` | `JointTrajectory` | latched, published once, `time_from_start` straight from the planner |
| `/<arm>/robot_description` | `String` | from the unmodified `ur.urdf.xacro` |

## Changing things

* **Move an arm** — edit `arms[].base` and relaunch. `world.build` re-applies the base
  transforms at runtime, so you do *not* need to re-run `tools/make_env.sh`; the YAML is the
  only place the layout is written down.
* **Change the cell** — add or edit `scene` entries. Remember the 0.834 m ceiling under the
  arms and the 0.22 m the gripper reserves above anything you want to reach.
* **Change the motion** — edit `routine`. A target the arm cannot reach, or a pair that
  collides, is reported by name before anything is planned.
* **Change the robots** — `arms[].ur_type` picks any model in `ur_description`. The planner
  still checks a UR5.
* **Re-run `tools/make_env.sh`** only when you change the number of arms or the plugin: it
  compiles the two-UR5 plugin with the repo's own `generate_vamp_robot_plugin.py`, then
  writes `two_arms.json` from `arms.yaml`.

Python nodes are **copied**, not symlinked, by `colcon build --symlink-install`, so re-run
`colcon build` after editing a node.

## If something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `waypoint 3 (pick): ur7e folds into itself` | that recorded pose is in self-collision | re-teach it; the toolbar now says `plannable` before you record |
| `waypoint over_bins: ur5 cannot reach [...]` | routine target outside the arm's reach, or inside the gripper's 0.22 m clearance | raise the target or move it closer to the base |
| `leg home -> over_bins: Planning failed` | no collision-free path within `planning_time` | raise `planning_time`, or move the waypoints apart from the obstacles |
| arms float above the table in RViz | `display_only` risers removed, or `PEDESTAL_Z` changed | leave the risers in; they are the 84 mm the base sphere occupies |
| everything collides at every pose | table top raised above 0.834 m | lower `work_table_top` |
| walls of roadmap histograms in the log | `mr_planner_core` prints its roadmap statistics on stdout with `cbs_prm` | harmless; use `composite_rrt` for a quiet log |
