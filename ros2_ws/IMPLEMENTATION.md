# How this workspace was built, and everything that went wrong on the way

`README.md` describes the cell as it stands. This file is the working log: the order the
pieces were built in, and every error that cost real time — what it looked like, what was
actually wrong, the evidence that proved it, and where the fix lives.

---

## Part 1 — Implementation steps

The parent repository is deliberately ROS-free. Its only ROS code is dormant **ROS 1**
behind `MR_PLANNER_WITH_ROS=0`, there is no `package.xml` anywhere, and its visualizer is
meshcat. `how_to_python/` already proved the whole planning pipeline in plain Python
(`build_env.sh` for a 2×UR5 VAMP plugin, `plan_bimanual.py` for the planning demo). This
workspace is that pipeline turned into a ROS 2 Jazzy package, without touching anything
upstream.

1. **Vendor the robot descriptions.** `src/ur_description/` is a checkout of the Universal
   Robots descriptions. It carries `ur5` and `ur7e` configs and their meshes, and
   `urdf/ur.urdf.xacro` renders either with `ur_type:=…`. Nothing in it is edited, ever.
2. **Build the VAMP plugin.** `tools/make_env.sh` calls the repository's own
   `mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py` with
   `--robot-struct UR5 --num-robots 2`, producing
   `libmr_planner_vamp_plugin_ur5_2r.so`. Both arms are planned as VAMP's UR5 (see the
   UR7e section below).
3. **Write the environment JSON.** `tools/make_env_json.py` turns `arms.yaml` into
   `two_arms.json`: robot group names, the plugin path, and the base transforms. Deriving it
   from the same YAML the nodes read is what keeps ROS and the planner from drifting apart.
4. **Display the arms.** `arms.display_nodes()` spawns one `robot_state_publisher` per arm
   in its own namespace with a `frame_prefix`, plus a `static_transform_publisher` from
   `world` to `<arm>/world` that adds VAMP's pedestal (`PEDESTAL_Z`, `PEDESTAL_YAW`) to the
   arm's configured base. This is the only place the two worlds are reconciled.
5. **Model the cell.** `scene` entries in `arms.yaml` become both
   `mr_planner_core.Object` boxes for the planner (`world.collision_box`) and RViz cubes
   (`arms.scene_markers`), so what you see is what is checked. `display_only` entries are
   markers only.
6. **Define the motion.** `routine` entries are end-effector positions per arm.
   `world.solve_routine` solves each with `env.inverse_kinematics`, seeded by the previous
   waypoint and oriented by the tool-down rotation of `ik.reference` — the same trick
   `how_to_python/plan_bimanual.py` uses.
7. **Plan leg by leg.** `world.plan_legs` runs one `env.plan()` per consecutive waypoint
   pair with every parameter taken from `planning`, and concatenates the returned per-arm
   trajectories along with the planner's own timestamps.
8. **Replay and publish.** `plan.py` publishes each arm's full path once on
   `/<arm>/trajectory` and steps `/<arm>/joint_states` at `dt / rate`.
9. **Teach.** `teach.py` builds the same environment so it can tell you, live, whether the
   pose under the sliders is plannable, and refuses to record one that is not.

---

## Part 2 — What went wrong

### 1. `RuntimeError: Planning failed`, cause one: the planner's joint range is ±π

**Symptom**

```
[plan-6]   File ".../vamp_mr_arms/plan.py", line 77, in solve
[plan-6]     result = self.environment.plan(
[plan-6] RuntimeError: Planning failed
[ERROR] [plan-6]: process has died [pid 35847, exit code 1, ...]
```

Nothing in that message says which waypoint, which arm, or why.

**Cause.** VAMP's UR5 maps the unit hypercube onto **±π per joint**:

```c++
// /usr/local/include/vamp/robots/ur5.hh:49-63
static constexpr std::array<float, dimension> s_m{6.2831854820251465, ...};   // 2π
static constexpr std::array<float, dimension> s_a{-3.1415927410125732, ...};  // -π
```

`joint_state_publisher_gui` takes its slider range from `ur_description`, which allows
**±2π**. The recorded `routine1.csv` therefore held values like `-4.948637`, `3.641734`,
`-5.354531` — physically identical poses, but outside the box the planner samples in.

**Evidence.** Two waypoints from that CSV that are provably collision-free:

| start/goal | result |
|---|---|
| raw, as recorded (`|q|` up to 5.35) | `Planning failed` after 0.26 s |
| the same poses wrapped into ±π | solved in < 10 ms, 52 samples |

**Fix.** `world.wrap` maps every joint value entering the planner to
`(q + π) mod 2π − π`. It is applied when a CSV is read (`plan.read_waypoints`), when a pose
is recorded (`teach.Teach.on_joint_state`) and to every IK solution
(`world.solve_routine`). Geometrically it changes nothing; a revolute joint at `q` and at
`q ± 2π` is the same pose.

### 2. `RuntimeError: Planning failed`, cause two: the taught waypoints were in collision

**Cause.** The old teach toolbar recorded whatever the sliders were showing, with no
validity check of any kind. Three of the five waypoints in `routine1.csv` are unusable, so
every leg touching them is unsolvable no matter how long the planner is given.

**Evidence.** Checking each row against the same environment the planner uses:

| row | `in_collision` | what is wrong |
|---|---|---|
| 1 | True | `ur7e` self-collision |
| 2 | True | `ur7e` self-collision |
| 3 | False | fine |
| 4 | True | the two arms occupy each other |
| 5 | False | fine |

**Fix, two halves.**
* *Prevention:* `teach.py` now builds the VAMP environment, shows `plannable` or the exact
  reason live, and `Record` refuses an invalid pose. A saved CSV is plannable by
  construction.
* *Diagnosis:* `world.reject_unplannable` checks every waypoint before any planning starts
  and raises one sentence naming the row and the arm:
  `waypoint 1 (waypoint): ur7e folds into itself`.

### 3. A C++ exception where a diagnosis belonged

**Cause.** All the work happened in `Plan.__init__`, so a `RuntimeError` from the pybind
layer escaped as a Python traceback through `ros2 launch`.

**Fix.** `world.Invalid` is the one exception type the pipeline raises, with a message that
names the leg or waypoint. `plan.main` catches it, logs it at `ERROR`, and returns exit
code 1 — one line, no traceback:

```
[ERROR] [plan]: waypoint 1 (waypoint): ur7e folds into itself
[ros2run]: Process exited with failure 1
```

### 4. `vmax` was silently thrown away

**Cause.** `vmax` was passed to the `VampEnvironment` constructor only. The very first line
of the `plan()` binding is `instance_->setVmax(vmax)`, using **that call's** `vmax`, whose
default is `1.0` (`mr_planner_core/python/mr_planner_core_pybind.cpp:1665` and the argument
default at `:3386`). Every trajectory was timed for 1.0 rad/s regardless of the config.

**Fix.** `world.plan_legs` passes the full `planning` block into `plan()`, `vmax` included.
Regression check: `vmax: 2.5` → **4.92 s** of motion, `vmax: 0.5` → **22.50 s**. Before the
fix those two were identical.

### 5. The robot is 7-DOF, and IK is strict about it

**Cause.** `env.sample_pose(0)` returns **seven** values: the six arm joints plus the
Robotiq 85 finger. `inverse_kinematics` rejects a seed whose length is not the robot's DOF,
while `plan()` accepts six and quietly clamps (`vamp_instance.h:1198-1214`), and the
collision model ignores a trailing seventh entry outright
(*"Allow a single trailing configuration entry (e.g., fixed flange joint) and ignore it"*,
`vamp_instance.h:1528-1545`).

**Fix.** The nodes work in six joints everywhere and pad only the IK seed, by exactly
`len(env.sample_pose(index)) - len(reference)` zeros (`world.solve_routine`). No magic
number, and it survives a plugin with a different gripper.

### 6. The table could not be where a table should be

**Cause.** The UR5 model's base collision sphere is **r = 0.08 centred on `base_link`**
(`vamp/resources/ur5/ur5_spherized.urdf`), which sits at z = 0.9144. Its underside is
therefore at 0.8344, and any table top above that collides at every pose.

**Evidence,** sweeping the top surface with the tucked pose and 60 random poses:

| top surface | tucked pose | random poses hitting the cell |
|---|---|---|
| 0.83 | free | 9 / 60 |
| 0.86 | **collides** | 17 / 60 |
| 0.9144 | **collides** | 17 / 60 |

**Fix.** The work table top is at **0.83**, and the 84 mm between it and the flange is a
`display_only` riser marker per arm. That volume is already occupied by the robot's own base
sphere, so nothing can enter it anyway — handing it to the planner as an obstacle would make
the robot collide with itself permanently.

### 7. The arms would not reach into anything

**Cause.** The planner checks a Robotiq 85 gripper that neither displayed arm wears, so
roughly 0.22 m above any surface is reserved. Early routine waypoints at 1.02 m over a bin
and 1.08 m over the target box simply had no IK solution — reported as an IK failure, which
reads like an unreachable target rather than a clearance problem.

**Evidence.** Sweeping the end-effector height for arm 0:

| over `bin_left` (lid 0.88) | over `target_box` (rim 0.94) |
|---|---|
| 1.16 / 1.12 / 1.08 / **1.05** solve | 1.25 / 1.20 / 1.15 / 1.12 / **1.10** solve |
| 1.03 / 1.00 / 0.98 fail | 1.08 / 1.05 fail |

**Fix.** `at_bins` uses 1.06 and `place` uses 1.11, and the cell represents the pick-place
motion without grasping. The 0.22 m figure is written into `arms.yaml` next to the routine so
the next edit starts from it.

### 8. VAMP has no UR7e — and the synthesised one was the wrong answer

**What was tried first.** A `tools/compose_ur7e.py` that rebuilt the `ur_description` UR7e
into a VAMP-shaped robot (same pedestal, same FTS + Robotiq 85, same ±π limits) so that
`foam` + `cricket` (`fkcc_gen`) could generate a real UR7e collision header.

**Why it was dropped.** It is a large, opaque piece of machinery producing a robot model
nobody can easily verify, and it was explicitly rejected: *"i am very suspicious of how this
ur7e robot arm is coming, if you have artificially made it then no need to do that"*, and
*"remove these unnecessary setup and complexity make everything simplified"*.

**What we do instead.** Both arms are **planned as VAMP's UR5**, and the arms on screen come
from the untouched `ur_description`. `tools/compose_ur7e.py` and `tools/urdf_paths.py` are
deleted. The cost is stated rather than hidden: the UR7e's shoulder is 73 mm taller and its
wrists 5-24 mm longer (upper arm identical, forearm within 0.05 mm), so arm 2's clearance is
approximate, and the planner reserves gripper volume you cannot see — erring toward caution.

### 9. Did the display ever match the planner?

This was an inherited assumption worth proving, because a wrong pedestal offset or yaw would
silently plan a different robot than the one on screen.

**Test.** Spawn `robot_state_publisher` + the static transform exactly as `arms.py` does,
publish a joint state, look up `world → ur5/tool0`, and compare against
`env.end_effector_transform` for the same joints.

**Result.** The world-frame distance is a constant **0.0350 m** across configurations, and
`tool0 → robotiq_85_base_link` in VAMP's own `ur5.urdf` is `[0.035, 0, 0.0005]` — the same
0.0350 m. The display and the planner agree; the only difference is the gripper base offset
that VAMP's model has and `ur_description` does not.

### 10. The replay did not have to be the planned motion

**Cause.** The old replay indexed each arm's path with its own `step % len(path)` and
re-timed the published trajectory as `step * dt`. If any leg ever returned paths of unequal
length the arms would drift apart on screen while the log still claimed the plan was
collision-free, and the published `time_from_start` was a reconstruction rather than the
planner's own timing.

**Fix.** One index across both arms, and the planner's `result["times"]` carried through
(offset per leg) into `/<arm>/trajectory`. `plan.py` publishes nothing and replays nothing
unless `plan_legs` returned a trajectory, and the concatenation is checked once with
`trajectory_in_collision` before playback.

### 11. Smaller things that still cost time

* **The layout lived in two files.** `arms.yaml` held the bases for the display and
  `two_arms.json` held them for the planner, so moving an arm meant re-running
  `tools/make_env.sh` or silently planning the old layout. `world.build` now re-applies the
  base transforms from the YAML with `set_robot_base_transform`, so the YAML is the only
  source of truth and the generated JSON cannot drift.
* **`--symlink-install` does not symlink Python nodes.** For `ament_python`, the modules are
  copied into `install/`. Editing a node and relaunching runs the old code; `colcon build`
  is required. This cost one confusing test run where a fixed error message did not change.
* **Tk blocks SIGINT.** `tkinter`'s `mainloop` blocks in C, so `ros2 launch`'s Ctrl-C left
  the teach window orphaned. `teach.main` installs SIGINT/SIGTERM handlers that destroy the
  window.
* **`mr_planner_core` is chatty.** `cbs_prm` prints a full roadmap joint-value histogram to
  stdout, and the shortcutter prints its makespan and bandit statistics. Harmless, but it
  buries the node's own log lines; `composite_rrt` is quiet.
* **`ros2 launch` hides exit codes.** A node that dies reports
  `process has died [pid …, exit code 1]` with the traceback interleaved across lines.
  Keeping the failure path to a single `ERROR` line is what makes it readable at all.
