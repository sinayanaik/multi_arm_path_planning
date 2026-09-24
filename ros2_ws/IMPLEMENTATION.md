# How this workspace was built, and everything that went wrong on the way

`README.md` describes the cell as it stands. This file is the working log: the order the
pieces were built in, and every error that cost real time — what it looked like, what was
actually wrong, the evidence that proved it, and where the fix lives.

It is in two halves. **Part 2** is the current architecture: one MuJoCo model, generated into
everything else. **Part 3** is the history that led here, kept because most of its lessons
still bite and because two of them explain why the current design is shaped the way it is.

---

## Part 1 — Implementation steps

The parent repository is deliberately ROS-free. Its only ROS code is dormant **ROS 1** behind
`MR_PLANNER_WITH_ROS=0`, there is no `package.xml` anywhere, and its visualizer is meshcat.
`how_to_python/` already proved the whole planning pipeline in plain Python. This workspace is
that pipeline turned into a ROS 2 Jazzy package, without touching anything upstream.

1. **Read the MuJoCo model.** `tools/mjcf.py` parses MJCF far enough to be useful: the
   `<default>` class tree (flattened so one lookup gives a class's full inherited
   attributes), `childclass` inheritance down the body tree, and each geom's resolved pose,
   type and size. Nothing else in `tools/` guesses at MJCF semantics.
2. **Generate the URDFs.** `tools/mjcf_to_urdf.py` writes `ur5e.urdf` (meshes, materials, for
   RViz) and `ur5e_spherized.urdf` (spheres only, no visuals, for cricket) from that one
   parse.
3. **Generate the SRDF.** `tools/make_srdf.py` samples the sphere model and writes the
   allowed-collision matrix.
4. **Generate the collision kernel.** cricket's `fkcc_gen` compiles the spherized URDF into
   a `vamp::robots::UR5e` header, and `generate_vamp_robot_plugin.py` builds the two-robot
   plugin around it.
5. **Import the cell.** `tools/mjcf_to_scene.py` settles `bimanual_scene.xml` in MuJoCo and
   writes `config/cell.yaml`: the arms' mount transforms and 49 collision boxes.
6. **Prove it.** `tools/check_model.py` compares the generated model against MuJoCo's own
   kinematics and collision geometry. It runs as part of `make_env.sh`, not as an optional
   extra.
7. **Display, teach, plan.** `arms.py` spawns one `robot_state_publisher` per arm off the
   generated URDF plus the mount transform from `cell.yaml`; `teach.py` tells you live
   whether a pose is plannable; `plan.py` runs one `plan()` per leg and replays the result.

---

## Part 2 — Why the model is generated

### The problem this replaced

The workspace used to display `ur_description`'s UR5 and UR7e while planning both arms
against VAMP's precompiled UR5. Three things were wrong with that at once:

* **`ur_description` was not installed.** Not in `src/`, not in `/opt/ros/jazzy/share`, and
  not declared in `package.xml`. `arms.py` called `FindPackageShare("ur_description")`, so
  both launch files died before RViz opened. This is the error that started the rewrite.
* **Arm 2 was a different robot from the one being checked.** UR5 vs UR5e/UR7e differ by
  +73.8 mm at the shoulder, +24.9 mm at `d4`, +5.4 mm at `d5`. The README used to state that
  cost; it could not remove it.
* **The planner's gripper was invisible.** VAMP's UR5 permanently carries a Robotiq 85 that
  neither displayed arm wore, so a rejection could name a gripper link hitting a bin that was
  nowhere near anything on screen. `world.build()` allow-listed all ten gripper links against
  every scene object to make the messages readable — which meant roughly 0.22 m of real
  collision volume was silently switched off.

### Why cricket, and why foam is not needed

VAMP's robots are 10,000-line generated headers with the FK and collision checks unrolled for
SIMD (`vamp/src/impl/vamp/robots/ur5.hh` is 10,027 lines, 40 spheres). They are not
hand-writable. cricket's `fkcc_gen` is the generator; its input is a URDF whose collision
geometry is **nothing but spheres**, plus an SRDF.

The usual way to get that spherized URDF is [foam](https://github.com/CoMMALab/foam), which
approximates meshes with sphere trees. We do not need it, because the MuJoCo model already
carries explicit collision primitives for the arm — capsules, and one cylinder at the wrist.
A capsule is by definition the set of points within `r` of a line segment, so a row of
spheres along that segment reproduces it exactly up to the spacing. Spaced `d` apart with
radius `sqrt(r² + (d/2)²)` the union contains the capsule, because a point at radial distance
≤ `r` and at most `d/2` along the axis from the nearest centre is within that distance of it,
and the rounded ends are covered by the end spheres at radius `r` alone.

Only the 2F85 needed judgement, since its collision geometry is meshes. Those get bounding
spheres per slab, and `check_model.py` then verifies the result against the actual triangles
rather than trusting it.

**Note the repository names.** VAMP's README links `github.com/CoMMALab/cricket` and
`CoMMALab/foam`. The KavrakiLab URLs that appear in some places 404.

### What the checks actually assert

```
kinematics : 200 configurations, worst link disagreement 0.0000 mm
containment: 35059 points sampled over the MuJoCo collision geometry, 0 outside every sphere
```

Plus, separately, the live TF tree against the same kinematics: **0.00000 mm** in the world
frame over 12 random poses, with `robot_state_publisher` and the real mount transform in the
loop. The old workspace's equivalent measurement was a constant 0.0350 m, which was the
`tool0` → `robotiq_85_base_link` offset of a gripper that was not on screen. That gap is now
zero because there is only one robot.

### Three decisions worth writing down

**Joint limits are ±π, not the MJCF's ±2π.** The MJCF allows ±2π on five of six joints (the
elbow is already ±π via `ur5e:size3_limited`). The generated URDF caps all six at ±π so VAMP
samples the same volume it always did and `world.wrap` stays meaningful. Nothing is
unreachable — `q` and `q ± 2π` are the same pose. `JOINT_LIMIT` in `mjcf_to_urdf.py` is the
one place to change it.

**The gripper's finger joints are fixed at fully open.** The 2F85 is a closed four-bar
linkage held together by `<equality><connect>` constraints, which URDF cannot express. Fixing
the joints keeps the model 6-DOF and sidesteps the loop entirely. This is not a shortcut
around a hard problem: VAMP's own `ur5_spherized.urdf` makes every one of its Robotiq 85
joints `fixed` for the same reason.

**`base_link` and `shoulder_link` may touch the workbench.** The arms mount 0.2 mm above it,
so they are inside it by construction. `set_allowed_collision("robot_base", link, True)` says
so directly, instead of the old approach of shortening the obstacle until the arm cleared it
and drawing the missing volume as a `display_only` riser.

---

## Part 3 — What went wrong

### 1. `RuntimeError: Planning failed`, cause one: the planner's joint range is ±π

**Cause.** VAMP's robots map the unit hypercube onto ±π per joint:

```c++
static constexpr std::array<float, dimension> s_m{6.2831854820251465, ...};   // 2π
static constexpr std::array<float, dimension> s_a{-3.1415927410125732, ...};  // -π
```

`joint_state_publisher_gui` took its slider range from `ur_description`, which allowed ±2π.
Recorded waypoints therefore held values like `-4.948637` — physically identical poses,
outside the box the planner samples in.

| start/goal | result |
|---|---|
| raw, as recorded (`\|q\|` up to 5.35) | `Planning failed` after 0.26 s |
| the same poses wrapped into ±π | solved in < 10 ms, 52 samples |

**Fix.** `world.wrap` maps every joint value entering the planner to `(q + π) mod 2π − π`.
Still applies: the generated UR5e URDF is written with ±π limits precisely so the slider
range and the planner's box agree at the source.

### 2. `RuntimeError: Planning failed`, cause two: the taught waypoints were in collision

The old teach toolbar recorded whatever the sliders showed, with no validity check. Three of
five waypoints in the saved CSV were unusable, so every leg touching them was unsolvable no
matter how long the planner was given.

**Fix, two halves.** *Prevention:* `teach.py` builds the VAMP environment, shows `plannable`
or the exact reason live, and `Record` refuses an invalid pose — a saved CSV is plannable by
construction. *Diagnosis:* `world.reject_unplannable` checks every waypoint before any
planning starts and raises one sentence naming the row and the arm.

### 3. A C++ exception where a diagnosis belonged

All the work happened in `Plan.__init__`, so a `RuntimeError` from the pybind layer escaped
as a Python traceback through `ros2 launch`. `world.Invalid` is now the one exception type
the pipeline raises, `plan.main` catches it, logs one `ERROR` line and returns 1.

`world.check_plugin` is the same idea applied to the generated model: a missing `.so` would
otherwise surface as a dlopen failure naming a path, with no hint that the path is generated
or by what.

### 4. `vmax` was silently thrown away

`vmax` was passed to the `VampEnvironment` constructor only. The first line of the `plan()`
binding is `instance_->setVmax(vmax)` using **that call's** `vmax`, whose default is `1.0`.
Every trajectory was timed for 1.0 rad/s regardless of config. `world.plan_legs` now passes
the full `planning` block into `plan()`. Regression check: `vmax: 2.5` → 4.92 s of motion,
`vmax: 0.5` → 22.50 s; before the fix those two were identical.

### 5. The robot's DOF is whatever the plugin says it is

`env.sample_pose(0)` returned **seven** values for the old UR5 plugin: six arm joints plus
the Robotiq 85 finger. `inverse_kinematics` rejects a seed whose length is not the robot's
DOF, while `plan()` accepts six and quietly clamps. The nodes work in six joints everywhere
and pad only the IK seed, by exactly `len(env.sample_pose(index)) - len(reference)` zeros.
The generated UR5e is 6-DOF, so that padding is now empty — but it is computed, not assumed,
which is why changing the gripper does not break it.

### 6. Sphere approximation is not free, and it shows up at the base

The old UR5 model's base sphere was r = 0.08 centred on `base_link`, so its underside sat
0.0804 below the mounting face and any table top above that collided at every pose. Sweeping
the surface: 0.83 free, 0.86 collides, 0.9144 collides.

This is inherent, not a bug, and the generated UR5e has it too. The UR5e base casting is a
squat cylinder — 99 mm tall, 151 mm across — and *any* sphere containing it bulges about
75 mm past its flat top. The consequence now shows up as `base_link` ↔ `upper_arm_link`
appearing permanently in contact, which the SRDF disables.

What changed is the response. The old cell shortened the obstacle and drew the missing 84 mm
as a `display_only` riser. The new one allows the contact for the two links that are bolted
in and keeps the workbench's true 0.8635 m height in the planner.

### 7. VAMP has no UR5e — and the first attempt at one was the wrong answer

**What was tried first.** A `tools/compose_ur7e.py` that rebuilt `ur_description`'s UR7e into
a VAMP-shaped robot so that foam + cricket could generate a real collision header.

**Why it was dropped.** It was a large, opaque piece of machinery producing a robot nobody
could easily verify, and it was rejected: *"i am very suspicious of how this ur7e robot arm is
coming, if you have artificially made it then no need to do that"*.

**Why the current generator is a different thing.** That objection was about provenance, and
it was right. The difference now is not that the model is generated — it is — but that
nothing about it is synthesised:

* the kinematics are the MJCF's own body poses, copied;
* the collision spheres are derived from the MJCF's own capsules by a stated geometric
  identity, not fitted;
* `check_model.py` runs on every generation and compares against MuJoCo itself, so the claim
  "this is the MuJoCo robot" is checked rather than asserted.

The spherization has exactly one free parameter (`--spacing-factor`), and it trades sphere
count against inflation along a formula printed in the source.

### 8. Two bugs in the verification, caught because the verification was wrong loudly

**A containment test that required every sphere.** The first run of `check_model.py` reported
29,139 of 35,059 points outside the sphere model, with a worst case of −46 mm. The model was
fine; the test computed `min(radius − distance)` over the spheres, which asks whether a point
is inside *every* sphere. A point is covered if it is inside *any* sphere, so it is a `max`.
With that fixed: 20 points out, all at 0.00 mm — round-off where a sphere had been grown to
exactly touch its furthest point. One micron of margin in the generator settled it.

**MuJoCo recentres mesh assets.** At compile time MuJoCo moves every mesh's vertices into its
centre-of-mass frame and folds the shift back into the geom, so `geom_xpos`/`geom_xmat` place
the *recentred* vertices. `mjcf_to_scene.py` initially handed those straight to RViz as the
pose of the original `.obj`, and the workbench came out rotated by `[3.064, -1.516, -0.042]`
— its inertia frame. `mesh_frame()` divides `mesh_pos`/`mesh_quat` back out. The three scene
meshes now come out at exactly the body poses their MJCF declares.

The collision-geometry side was never affected: `mjcf_to_urdf.py` reads the raw STL from disk
and composes it with the authored geom frame, which was verified to place vertices in the same
world positions as MuJoCo's own path, to 1e-3.

### 9. The replay did not have to be the planned motion

The old replay indexed each arm's path with its own `step % len(path)` and re-timed the
published trajectory as `step * dt`. If any leg returned paths of unequal length the arms
would drift apart on screen while the log still claimed the plan was collision-free. Now: one
index across both arms, the planner's own `result["times"]` carried through into
`/<arm>/trajectory`, and the concatenation checked once with `trajectory_in_collision` before
playback.

### 10. Smaller things that still cost time

* **The layout lived in two files.** `arms.yaml` held the bases for the display and
  `two_arms.json` held them for the planner, so moving an arm meant re-running `make_env.sh`
  or silently planning the old layout. Both now come from `cell.yaml`, which is itself
  generated, and `world.build` re-applies the transforms with `set_robot_base_transform`.
* **Generated and hand-written config are separate files.** `cell.yaml` is overwritten every
  time the cell is imported. Keeping the routine in `arms.yaml` means re-importing cannot
  destroy a routine that took an afternoon to teach.
* **`--symlink-install` does not symlink Python nodes.** For `ament_python` the modules are
  copied into `install/`. Editing a node and relaunching runs the old code; `colcon build` is
  required. This cost one confusing test run where a fixed error message did not change.
* **Tk blocks SIGINT.** `tkinter`'s `mainloop` blocks in C, so `ros2 launch`'s Ctrl-C left the
  teach window orphaned. `teach.main` installs SIGINT/SIGTERM handlers that destroy the
  window.
* **`mr_planner_core` is chatty.** `cbs_prm` prints a full roadmap joint-value histogram to
  stdout. Harmless, but it buries the node's own log lines; `composite_rrt` is quiet.
* **The `.obj` files name `.mtl` files that do not exist.** MuJoCo ignores `mtllib` and uses
  its own `<material>` assets, so nothing looked wrong there — but RViz would have rendered
  the whole arm default white. The MJCF's material colours are carried into the URDF's
  `<material>` elements instead.
