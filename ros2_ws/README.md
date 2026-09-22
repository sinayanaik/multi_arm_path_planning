# ROS 2 workspace for VAMP-MR

A two-arm pick-and-place cell: a UR5 and a UR7e mounted side by side on a work table, a
conveyor in front of them carrying two source bins, and a target box on their own table.
Both arms sweep from the bins to the box as **one coordinated motion planned by VAMP-MR**,
and RViz replays exactly the trajectory the planner returned.

No URDF is edited anywhere: both arms are displayed straight from the vendored
`ur_description`. `vamp` is used unmodified from its existing `/usr/local` install.
`mr_planner_core` gained two small collision-diagnostic bindings
(`colliding_links` / `colliding_links_robot`, wrapping its existing but previously-unbound
`PlanInstance::debugCollidingLinks`) in its own repo at
`/home/tcs-research/Documents/multi_arm_path_planning/mr_planner_core`; after pulling changes
there, rebuild and reinstall it (`cmake --build build -j && sudo cmake --install build`)
before this workspace's `collision_reason()` messages will include the colliding link/object
names.

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
        ^                     conveyor  belt top 0.78, 1.80 x 0.50,  y = +0.51
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
| conveyor belt | 1.80 x 0.50 | 0.78 | `(0.00, +0.51)` |
| `bin_left` / `bin_right` | 0.20 x 0.20 | 0.88 | `(-+0.35, +0.35)` |
| `target_box` | 0.26 x 0.26 | 0.94 | `(0.00, -0.02)` |
| arm mounting flange | — | 0.9144 | `(-+0.35, -0.25)` |

The conveyor sits off-center from the bins it carries (y = +0.51 vs. their +0.35) so its
footprint clears the work table's (y = -0.65 to +0.15) rather than overlapping it by ~5 cm.
VAMP never checks scene objects against each other, so that overlap never affected planning —
it only ever showed up as the two slabs visibly interpenetrating in RViz.

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

**4. The gripper is excluded from scene-object collision checks.**
VAMP's compiled UR5 model permanently includes a Robotiq 85 gripper + FTS300 sensor as ten
extra links (`fts_robotside` and the nine `robotiq_85_*` links, `GRIPPER_LINKS` in
`world.py`), on top of the bare arm neither arm on screen is wearing — so a rejected pose
could name a gripper link colliding with a bin or the conveyor that was nowhere near what
RViz showed. `world.build()` now calls `environment.set_allowed_collision("*", link, True)`
for each of those links, so they no longer collide with scene objects. This does **not**
cover gripper self-collision or gripper-vs-other-arm: those checks are unrolled into VAMP's
own installed `/usr/local/include/vamp/robots/ur5.hh`, outside this repo, with no filter hook
— removing them would mean patching that third-party header directly (fragile, and reverts
on any `vamp` reinstall), so it hasn't been done. In practice this has never been what
actually triggered a rejection in this cell. The routine's hover heights (`at_bins` 1.06,
`place` 1.11, vs. bin lid 0.88 / target rim 0.94) were originally sized to clear the now-
excluded gripper and are more conservative than the bare arm needs — safe to tighten if you
want the arms to dip further.

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

## Nodes

Both launch files start the same display stack, then add their own mode-specific
nodes. "Both" = launched by both `teach.launch.py` and `plan.launch.py`.

**Launched by both** (via `arms.py: display_nodes()` + `rviz_node()`), one pair
per arm plus one shared viewer:

| Node | Package / executable | Namespace | Input | Output |
|---|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | `/<arm>` | `robot_description` parameter (a URDF string built by `Command(["xacro ", ur.urdf.xacro, "ur_type:=", arm["ur_type"], "name:=", arm["name"]])` from the vendored `ur_description`); subscribes `/<arm>/joint_states` | publishes `/tf` (one dynamic transform per non-fixed joint) and the latched `/<arm>/robot_description` |
| `<arm>_base` (a `tf2_ros static_transform_publisher`) | `tf2_ros` | — | command-line xyz/rpy args, computed in `arms.py: display_nodes()` from `arms.yaml`'s `arms[].base` plus the fixed `PEDESTAL_Z`/`PEDESTAL_YAW` constants | publishes one `/tf_static` transform, `world` → `<arm>/world` |
| `rviz2` | `rviz2` | — | `-d src/vamp_mr_arms/rviz/arms.rviz`; that config's Displays subscribe to `/<arm>/robot_description` (×2, RobotModel), TF, `/scene`, `/collision_spheres` | the 3D view; no topics out |

**`teach.launch.py` also starts** (`vamp_mr_arms/vamp_mr_arms/teach.py`):

| Node | Input | Output |
|---|---|---|
| `<arm>/joint_state_publisher_gui` (×2, package `joint_state_publisher_gui`) | reads `/<arm>/robot_description` once to learn joint names and slider ranges | publishes `/<arm>/joint_states` every time you drag a slider — this is **live user input**, not planner output |
| `teach` | parameters `config` (arms.yaml override), `output` (waypoint CSV path), `meshcat`/`meshcat_host`/`meshcat_port`; subscribes `/<arm>/joint_states` ×2; looks up `world → <arm>/<ee_frame>` on the TF tree for the pose readout | publishes `/scene` once (latched) and `/collision_spheres` every GUI tick (~20 Hz, needs `mr_planner_core`'s `robot_spheres` — see below); opens the Tkinter toolbar; writes the waypoint CSV when you click **Save CSV**; if `meshcat:=true`, also streams raw JSON over a plain TCP socket to a separately-run `meshcat_bridge.py` (not a ROS topic) |

`teach` is one process doing two jobs at once: an `rclpy.Node` spun on a
background thread, and a Tkinter `Toolbar` window run on the main thread's
`mainloop()` (`teach.py: main()`) — the 50 ms `Toolbar.tick()` is what drives
every one of `teach`'s outputs above, not a ROS timer or callback.

**`plan.launch.py` also starts** (`vamp_mr_arms/vamp_mr_arms/plan.py`):

| Node | Input | Output |
|---|---|---|
| `plan` | parameters `config`, `waypoints` (empty = plan `arms.yaml`'s `routine`; a path = read that CSV's `<arm>_<joint>_rad` columns instead, `read_waypoints()`) | publishes `/scene` once (latched); plans with `world.plan_legs()`, then publishes `/<arm>/trajectory` once (latched, the full path) via `publish_trajectories()`; then a `Replay` object (a ROS timer at `planning.dt / replay.rate` seconds) streams that same path onto `/<arm>/joint_states`, looping if `replay.loop`; logs waypoint list, per-leg planning stats, and the final `trajectory_in_collision` check to stdout |

**The one thing to hold onto:** `/<arm>/joint_states` is published by a
*different* node depending on which launch file is running, and the data flows
in opposite directions — `teach.launch.py`'s `joint_state_publisher_gui` turns
your slider drags *into* that topic (you are the source); `plan.launch.py`'s
`plan` node turns a *planned* trajectory *into* that same topic on a timer (the
planner is the source). Either way, `robot_state_publisher` just consumes
whatever shows up there and turns it into TF for RViz to draw — it has no idea
whether the joints came from a human or a plan.

## Teaching your own waypoints

```bash
ros2 launch vamp_mr_arms teach.launch.py output:=routine.csv
```

Both arms appear in the cell with a slider window each. The toolbar shows every joint, the
live end-effector pose, and — the part that matters — whether the current pose is
**plannable**. `Record` refuses a pose the planner cannot use and says why (`ur7e folds into
itself`, `the arms hit each other`, `ur5 hits the cell`), so a saved CSV is plannable by
construction. `Save CSV` writes it out.

The reason names the exact colliding pair, e.g. `ur5 hits the cell (forearm_link hits
work_table_top)`. Gripper links (`fts_robotside`, `robotiq_85_*`) never appear here against
scene objects — VAMP's compiled UR5 model permanently includes that gripper, but
`world.build()` allow-lists it against every scene object (see "What the planner actually
sees" above), since neither arm on screen wears one. A rejection can still look surprising
from a 2D screenshot even so: a link can be sitting right above an obstacle, invisibly close
in Z, while camera perspective makes it look nowhere near. Read the `tool0` pose numbers in
the toolbar, not just the picture, if a rejection looks wrong — VAMP's own forward kinematics
decided it, not what the camera angle makes it look like.

### Seeing what the planner actually checks (Meshcat)

`teach.launch.py` can stream VAMP's real collision geometry — every sphere it approximates
each arm with, gripper included, plus the scene boxes — to a browser, live, as you jog the
sliders. Meshcat draws VAMP's raw spheres unfiltered, so the gripper spheres will still show
overlapping scene objects there even though `world.build()`'s allow-list means that overlap
is no longer reported as a collision — Meshcat shows geometry, not the collision verdict. It
needs the `meshcat` bridge running first:

```bash
# terminal 1 — start once, leave running
python3 /home/tcs-research/Documents/multi_arm_path_planning/mr_planner_core/scripts/visualization/meshcat_bridge.py --port 7600
```

It prints two lines: `[bridge] Meshcat server started at http://127.0.0.1:<some-port>/static/`
and `[bridge] Listening on ('127.0.0.1', 7600)`. The first is the browser URL — open it (its
port is picked independently by the `meshcat` package, not `--port`, so read it from the
output rather than assuming a number). The second confirms it's listening on `--port`, which
is the one `teach.launch.py`'s `meshcat_port` must match. Then in a second terminal:

```bash
# terminal 2
ros2 launch vamp_mr_arms teach.launch.py meshcat:=true meshcat_port:=7600
```

The `teach` node logs `meshcat enabled, streaming to ...` once connected. If the bridge isn't
up yet, `push_meshcat` retries the connection on every GUI tick (20 Hz), so it self-heals once
you start the bridge — but expect a `[meshcat] connection failed: connection refused` warning
on every tick until then, which is harmless but noisy; start the bridge first to avoid it.
`meshcat_port` defaults to `7600` and only needs setting if you changed `--port` above.

### Seeing it without leaving RViz (`/collision_spheres`)

`teach` also publishes a `MarkerArray` on `/collision_spheres` every tick — one translucent
sphere per collision sphere of each arm, in `arms.rviz` already (enabled by default) — no
bridge, no browser. It needs `mr_planner_core` rebuilt and reinstalled for
`environment.robot_spheres()` to exist (`cmake --build build -j && sudo cmake --install
build` in `mr_planner_core`); until then `teach` logs one warning and skips the topic rather
than failing. Gripper spheres are drawn too, in yellow (arm spheres are orange-red) — they're
allow-listed against scene objects (§4 above) but still checked for self-collision and
arm-vs-arm, so they stay visible rather than looking removed when they aren't fully gone.

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

| Topic | Type | Publisher | Subscriber(s) | Notes |
|---|---|---|---|---|
| `/scene` | `MarkerArray` | `teach` or `plan` (`arms.py: publish_scene`) | `rviz2` | latched, one cube per `scene` + `display_only` entry, published once at startup |
| `/collision_spheres` | `MarkerArray` | `teach` only | `rviz2` | not latched, republished every GUI tick (~20 Hz); one sphere per collision sphere per arm (gripper spheres in yellow); needs `mr_planner_core`'s `robot_spheres` installed, else skipped with one logged warning |
| `/<arm>/joint_states` | `JointState` | **teach mode:** `<arm>/joint_state_publisher_gui`, on every slider move. **plan mode:** `plan`'s `Replay`, on a timer at `planning.dt / replay.rate` (10 Hz by default), looping if `replay.loop` | `<arm>/robot_state_publisher` (always); `teach` itself, teach mode only | same topic, opposite direction depending on which launch file is running — see "Nodes" above |
| `/<arm>/trajectory` | `JointTrajectory` | `plan` only (`publish_trajectories`) | none in this workspace (informational, for external consumers) | latched, published once after planning, `time_from_start` straight from the planner |
| `/<arm>/robot_description` | `String` | `<arm>/robot_state_publisher` | `rviz2`; `<arm>/joint_state_publisher_gui` (teach mode only, to learn joint names/limits) | latched; built from the unmodified `ur_description` `ur.urdf.xacro`, not from any file this repo edits |
| `/tf` | `tf2_msgs/TFMessage` | `<arm>/robot_state_publisher` ×2 | `rviz2`; `teach`'s TF listener (for the `tool0` pose readout) | standard dynamic transforms, one per non-fixed joint |
| `/tf_static` | `tf2_msgs/TFMessage` | `<arm>_base` static_transform_publisher ×2 | `rviz2`; `teach`'s TF listener | the fixed `world` → `<arm>/world` mount transform each arm's TF tree hangs off of |

## Changing things

* **Move an arm** — edit `arms[].base` and relaunch. `world.build` re-applies the base
  transforms at runtime, so you do *not* need to re-run `tools/make_env.sh`; the YAML is the
  only place the layout is written down.
* **Change the cell** — add or edit `scene` entries. Remember the 0.834 m ceiling under the
  arms (§3 above); the gripper no longer reserves clearance above reachable surfaces since
  it's excluded from scene-object collision checks (§4 above).
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
| `waypoint 3 (pick): ur7e folds into itself` | that recorded pose is in self-collision | re-teach it; the toolbar now says `plannable` before you record. If the named links are `robotiq_85_*`/`fts_robotside`, that's the gripper self-colliding with the arm — not filterable without patching VAMP's own installed header, see "What the planner actually sees" |
| `... hits the cell (link hits ...)` with nothing visibly touching in RViz | a link (gripper excluded) can be much closer in Z to an obstacle than 2D perspective suggests | read the `tool0` numbers, or launch with `meshcat:=true` / check `/collision_spheres` to see it |
| `waypoint over_bins: ur5 cannot reach [...]` | routine target outside the arm's reach | raise the target or move it closer to the base |
| `leg home -> over_bins: Planning failed` | no collision-free path within `planning_time` | raise `planning_time`, or move the waypoints apart from the obstacles |
| arms float above the table in RViz | `display_only` risers removed, or `PEDESTAL_Z` changed | leave the risers in; they are the 84 mm the base sphere occupies |
| everything collides at every pose | table top raised above 0.834 m | lower `work_table_top` |
| walls of roadmap histograms in the log | `mr_planner_core` prints its roadmap statistics on stdout with `cbs_prm` | harmless; use `composite_rrt` for a quiet log |
