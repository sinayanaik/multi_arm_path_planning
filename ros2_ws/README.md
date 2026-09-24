# ROS 2 workspace for VAMP-MR

Two UR5e arms with Robotiq 2F85 grippers, bolted to a workbench, reaching into three bins on
a table and a conveyor. Both arms move as **one coordinated motion planned by VAMP-MR**, and
RViz replays exactly the trajectory the planner returned.

Everything about the robot and the cell comes from two MuJoCo files and nothing else:

| File | What it decides |
|---|---|
| `src/vamp_mr_arms/models/ur5e.xml` | the robot: link geometry, joint axes, collision shapes, the gripper |
| `src/vamp_mr_arms/models/bimanual_scene.xml` | the cell: where the arms are bolted, and every obstacle around them |

`tools/make_env.sh` turns those into the URDF RViz draws, the sphere model VAMP checks, the
obstacle list the planner is given, and the two-arm plugin — so **the robot on screen and the
robot being collision-checked are generated from the same source in the same run.** They
cannot drift apart. That was not true before: this workspace used to display
`ur_description`'s UR5/UR7e while planning against VAMP's precompiled UR5, and the two
differed by up to 74 mm.

```bash
tools/make_env.sh                       # once: generate the model, the cell and the plugin
colcon build --symlink-install
source install/setup.bash

ros2 launch vamp_mr_arms plan.launch.py                         # plan and replay the routine
ros2 launch vamp_mr_arms teach.launch.py output:=routine.csv    # jog and record your own
ros2 launch vamp_mr_arms plan.launch.py waypoints:=routine.csv  # replay what you recorded
```

## One-time setup: cricket

VAMP ships precompiled collision kernels for a handful of robots and a UR5e is not among
them, so ours is generated. The generator is `fkcc_gen` from
[CoMMALab/cricket](https://github.com/CoMMALab/cricket), which is not packaged anywhere:

```bash
sudo apt install libcgal-dev cppad libeigen3-dev nlohmann-json3-dev libfmt-dev \
                 ros-jazzy-pinocchio
pip install mujoco                      # tooling only; not a runtime dependency

git clone --recursive https://github.com/CoMMALab/cricket ../cricket
cd ../cricket
cmake -GNinja -Bbuild -DCRICKET_BUILD_JIT=OFF .    # JIT off, so no LLVM is needed
cmake --build build
```

`make_env.sh` looks for `../cricket/build/fkcc_gen`; set `FKCC_GEN` if yours is elsewhere.
Without it the script still generates the URDFs and the cell — RViz will draw both arms
correctly — and stops before the planner's kernel with a message saying so.

## The cell

All of it is read out of `bimanual_scene.xml`, with the free bodies dropped onto their
surfaces first, and written to `config/cell.yaml`.

```
        y
        ^                     work table      top 0.75,  1.35 x 0.60,  y = 1.10
        |    +---------------------------------------------------+
        |    |   [ right_bin ]                   [ left_bin ]     |   rims at 0.83
        |    +---------------------------------------------------+
        |
        |            conveyor  belt top 0.705, 1.80 x 0.36,  y = 0.58
        |    +---------------------------------------------------+
        |    |                [ conveyor_bin ]                    |
        |    +---------------------------------------------------+
        |
        |    +---------------------------------------------------+
        |    |      (right)                        (left)         |   bases x = -0.26 / +0.22
        |    |            workbench  top 0.8635, 1.02 x 0.80      |
        |    +---------------------------------------------------+
        +-------------------------------------------------------------> x
```

| Feature | Footprint (m) | Top surface (m) |
|---|---|---|
| workbench (`robot_base`) | 1.02 x 0.80 | 0.8635 |
| work table | 1.35 x 0.60 | 0.750 |
| conveyor frame | 2.00 x 0.60 | 0.700 |
| conveyor belt | 1.80 x 0.36 | 0.705 |
| `left_bin` at (+0.20, 1.00) | 0.40 x 0.30 | floor 0.770, rim 0.830 |
| `right_bin` at (-0.30, 1.00) | 0.40 x 0.30 | floor 0.770, rim 0.830 |
| `conveyor_bin` at (0.00, 0.58) | 0.40 x 0.30 | floor 0.725, rim 0.785 |
| blocks, 15 per source bin | 0.09 x 0.05 x 0.03 | 0.800 |
| arm mounting face | — | 0.8637 |

Both arms are yawed +90° about Z, so `shoulder_pan = 0` points along **+y**, towards the
bins. 49 collision boxes go to the planner: the workbench, the table, the conveyor frame and
belt, all three bins as their real five walls each — so an arm can reach *into* a bin rather
than bouncing off a solid block — and all 30 blocks.

Two of the cell's bodies collide as raw meshes in MuJoCo, and VAMP's obstacle backend takes
only boxes, spheres and cylinders (`Object::Shape::Mesh` is in the enum, but the backend
returns `nullopt` for it). Those two become the axis-aligned box of their own vertices:
exactly right for the workbench, and for the conveyor it also fills the open space under the
belt, which is conservative and which nothing reaches into. RViz still draws the original
meshes over the boxes, so the cell looks like itself.

## What the planner actually sees

**The UR5e it checks is the UR5e on screen.** `tools/check_model.py` runs on every
`make_env.sh` and asserts it:

```
kinematics : 200 configurations, worst link disagreement 0.0000 mm
containment: 35059 points sampled over the MuJoCo collision geometry, 0 outside every sphere
```

The first line drives random configurations through both MuJoCo's own `mj_forward` and the
generated URDF and compares every link pose. The second samples the surface of every MJCF
collision capsule, box and mesh and checks each point falls inside some sphere of the sphere
model — because a sphere cover that *misses* would report clearance the robot does not have.
Separately, the live TF tree (`robot_state_publisher` plus the mount transform) agrees with
the same kinematics to 0.00000 mm in the world frame.

Four things are still worth knowing.

**1. The configuration space is ±π per joint, not ±2π.**
The MJCF allows ±2π on five of the six joints, but the generated URDF caps every joint at
±π, so VAMP samples a box the same size it always did. Nothing is unreachable: a revolute
joint at `q` and at `q ± 2π` is the same pose, and `world.wrap` maps every value entering
the planner into that box. Widen `JOINT_LIMIT` in `tools/mjcf_to_urdf.py` if you ever want
the full range, and accept the larger search volume.

**2. The gripper is real now, and is checked.**
The 2F85 is on screen and collision-checked like any other link — 21 of the model's 69
spheres are gripper. This replaces the old arrangement, where VAMP's compiled UR5 carried a
Robotiq 85 that neither displayed arm wore and which therefore had to be allow-listed away
from every scene object. Its finger joints are fixed at fully open, so the model stays 6-DOF
and the four-bar linkage (which URDF cannot express) never has to be broken; VAMP's own
`ur5_spherized.urdf` does exactly the same with its gripper.

**3. `base_link` and `shoulder_link` are allowed to touch the workbench.**
The arms mount 0.2 mm above the workbench top, so they are inside it by construction. Rather
than truncate the workbench to a height the arm happens to clear — which the old cell did,
with a `display_only` riser standing in for the missing 84 mm — `world.build()` calls
`set_allowed_collision("robot_base", link, True)` for those two links and hands the planner
the workbench's true height. `display_only` still exists in the schema and is now empty.

**4. Sphere approximation makes the base fatter than it is.**
The UR5e base casting is a squat cylinder, 99 mm tall and 151 mm across. *Any* sphere that
covers it bulges about 75 mm past its flat top, so `base_link` and `upper_arm_link` register
as permanently in contact and the SRDF disables that pair. VAMP's own UR5 has the identical
property for the identical reason (one r = 0.08 sphere on `base_link`). The generated SRDF
disables 165 of 231 link pairs — 21 adjacent, 2 from the MJCF's own `<contact><exclude>`, 24
always in contact, 118 never — leaving 66 genuinely checked. Those numbers are stable: 2,000
samples and 200,000 samples give the same answer.

## Layout

| Path | What it is |
|---|---|
| `src/vamp_mr_arms/models/` | the MuJoCo sources, their meshes, and the URDF/SRDF generated from them |
| `src/vamp_mr_arms/config/cell.yaml` | **generated** — the arms' mounts and all 49 obstacles |
| `src/vamp_mr_arms/config/arms.yaml` | hand-written — the routine, the IK seed, every planner parameter |
| `src/vamp_mr_arms/vamp_mr_arms/world.py` | the planner side: environment, wrapping, validity, IK, `plan()` per leg |
| `src/vamp_mr_arms/vamp_mr_arms/arms.py` | the ROS side: config loading, scene markers, display nodes |
| `src/vamp_mr_arms/vamp_mr_arms/plan.py` | plan the waypoints, publish the trajectories, replay them |
| `src/vamp_mr_arms/vamp_mr_arms/teach.py` | jog, watch, and record waypoints the planner can use |
| `src/vamp_mr_arms/vamp_env/` | generated: the VAMP plugin and the environment JSON |
| `tools/` | everything that turns MuJoCo into the above |

The config is split deliberately: re-importing the cell cannot overwrite a routine you spent
an afternoon teaching, and `arms.yaml` never goes stale against the MuJoCo scene because it
no longer describes it.

### tools/

| Script | Does |
|---|---|
| `make_env.sh` | runs all of the below in order |
| `mjcf.py` | reads MJCF: `<default>` class inheritance, the body tree, geom poses and shapes |
| `mjcf_to_urdf.py` | writes `ur5e.urdf` (meshes, for RViz) and `ur5e_spherized.urdf` (spheres, for cricket) |
| `urdf_kin.py` | URDF forward kinematics and sphere placement, with no ROS in the loop |
| `make_srdf.py` | the allowed-collision matrix, sampled against the spheres |
| `mjcf_to_scene.py` | settles the cell in MuJoCo and writes `config/cell.yaml` |
| `check_model.py` | proves the generated model is the MuJoCo model |
| `check_cell.py` | sanity-checks that the imported cell leaves the arms room to move |
| `make_env_json.py` | the environment JSON `mr_planner_core` loads |

## How the spheres are derived

They are not fitted or guessed. Every collision shape on the arm is already a capsule in the
MJCF (a cylinder at the wrist), and a capsule is *by definition* the set of points within `r`
of a line segment — so a row of spheres along that segment reproduces it. Spaced `d` apart
with radius `sqrt(r² + (d/2)²)`, the union contains the capsule: a point at radial distance
≤ `r` and at most `d/2` along the axis from the nearest centre is within that distance of it,
and the rounded ends are covered by the end spheres at radius `r` alone. `--spacing-factor`
trades count against inflation (1.0 → 12% fatter, 0.6 → 4%).

Only the gripper needed judgement, because its collision geometry is meshes. Those get
bounding spheres per slab along the longest axis, with a final pass that grows whichever
sphere is nearest any point still outside — and then `check_model.py` verifies the result
against the actual triangles. 69 spheres over 19 links, against VAMP's UR5's 40 over 17.

## Nodes

Both launch files start the same display stack, then add their own mode-specific nodes.

| Node | Package | Namespace | Input | Output |
|---|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | `/<arm>` | `models/ur5e.urdf`, read off disk; subscribes `/<arm>/joint_states` | `/tf`, and the latched `/<arm>/robot_description` |
| `<arm>_base` | `tf2_ros static_transform_publisher` | — | `cell.yaml`'s `arms[].base` | one `/tf_static`, `world` → `<arm>/base_link` |
| `rviz2` | `rviz2` | — | `rviz/arms.rviz` | the 3D view |

`teach.launch.py` adds a `joint_state_publisher_gui` per arm and the `teach` node; the
sliders are live user input. `plan.launch.py` adds the `plan` node, whose `Replay` timer
drives the same `/<arm>/joint_states` topic from a planned trajectory. Same topic, opposite
direction depending on which launch file is running — `robot_state_publisher` cannot tell
whether the joints came from a human or a plan.

## Changing things

* **Move an arm, or change the cell** — edit `models/bimanual_scene.xml` and re-run
  `tools/make_env.sh`. The plugin is only rebuilt if the *robot* changed; the cell is
  re-applied at runtime from `cell.yaml`.
* **Change the robot** — edit `models/ur5e.xml` and re-run `tools/make_env.sh`. This does
  regenerate the collision kernel, so it needs cricket.
* **Change the motion** — edit `routine` in `config/arms.yaml`, or record one with
  `teach.launch.py`. Targets position the `tcp` frame, which is the gripper's pinch point:
  a waypoint is where the grasp happens, not where the mounting flange goes.

Python nodes are **copied**, not symlinked, by `colcon build --symlink-install`, so re-run
`colcon build` after editing a node.

## If something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `the VAMP plugin ... is missing` | `make_env.sh` has not run, or stopped at cricket | build cricket, then re-run `tools/make_env.sh` |
| `models/ur5e.urdf: missing` | same | as above |
| `cell.yaml: missing` | same | as above |
| `waypoint 3 (pick): left folds into itself` | that pose is in self-collision | re-teach it; the toolbar says `plannable` before you record |
| `waypoint over_bins: left cannot reach [...]` | routine target outside the arm's reach | move it closer to the base, or lower |
| `leg home -> over_bins: Planning failed` | no collision-free path within `planning_time` | raise `planning_time`, or move waypoints away from obstacles |
| arms float above the workbench in RViz | `cell.yaml` edited by hand and now disagrees with the MuJoCo scene | re-run `tools/make_env.sh` |
| walls of roadmap histograms in the log | `mr_planner_core` prints roadmap statistics with `cbs_prm` | harmless; `composite_rrt` is quiet |
