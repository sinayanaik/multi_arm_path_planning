# ROS 2 workspace for VAMP-MR

Spawn a UR5 and a UR7e in RViz, hand-teach waypoints into a CSV, then plan the whole
routine as one coordinated two-arm motion with VAMP-MR and watch it play back.

Nothing in the parent repository is modified. `vamp` and `mr_planner_core` are used from
their existing `/usr/local` install.

## The one thing to know

VAMP ships a UR5 collision model and no UR7e, so **both arms are planned as UR5s**. The
UR7e's real kinematics are used for display. The two arms are close enough for this to be
reasonable — identical upper arm (0.425 m), forearm within 0.05 mm — but the UR7e's
shoulder sits 73 mm higher and its wrist offsets are 5-24 mm longer, so treat the planned
clearance for that arm as approximate.

To make RViz exactly truthful instead, point the second arm at `ur5.urdf` in
`config/arms.yaml`. A real UR7e model would need `foam` + `cricket` codegen, which is a
much larger piece of work.

## Layout

| Path | What it is |
|---|---|
| `src/vamp_mr_arms/` | the only package you edit — nodes, launch files, RViz config, `config/arms.yaml` |
| `src/vamp_mr_arms/vamp_env/` | generated: both display URDFs, the VAMP plugin, the environment JSON |
| `src/ur_description/` | vendored Universal Robots descriptions (the source of the UR7e) |
| `tools/` | `make_env.sh` and the three small scripts it drives |

## Build

```bash
tools/make_env.sh                 # once
colcon build --symlink-install
source install/setup.bash
```

`make_env.sh` is four steps: copy VAMP's UR5 URDF with its mesh paths fixed, build a UR7e
URDF from `ur_description` shaped like VAMP's UR5 (same 0.9144 m pedestal, same Robotiq 85
gripper, ±π limits), compile a two-UR5 plugin with the repo's own
`generate_vamp_robot_plugin.py`, and write the environment JSON from `arms.yaml`.

## 1 & 2 — spawn and teach

```bash
ros2 launch vamp_mr_arms teach.launch.py
```

Both arms appear in RViz with a slider window each. The toolbar shows every joint and the
live end-effector pose; **Record** appends the current pose of both arms, **Save CSV**
writes them out.

```bash
ros2 launch vamp_mr_arms teach.launch.py output:=routine.csv
```

One row per waypoint: `name`, then each arm's six joints (rad) followed by its
end-effector pose (m and rad). Only the joint columns are planned with — the pose columns
are there for you to read. Each pose is looked up at its own joint state's timestamp, so
the pose in a row always belongs to the joints beside it.

## 3 — plan and replay

```bash
ros2 launch vamp_mr_arms plan.launch.py waypoints:=routine.csv
```

Each consecutive pair of waypoints becomes one two-arm `plan()` query, so both arms start
and finish a leg together. The node logs per-leg planning time and whether the
concatenated result is collision-free, then loops the motion into RViz. It also publishes
each arm's full path once on `/<arm>/trajectory` as a `JointTrajectory`.

## Changing things

Everything tunable lives in `src/vamp_mr_arms/config/arms.yaml`: where the arms stand,
their joint and end-effector frames, the table geometry, and the planner settings
(`composite_rrt`, `cbs_prm` or `priority_sipp_rrt`).

Moving an arm changes what the planner sees, so re-run `tools/make_env.sh` afterwards —
it rewrites the base transforms into the environment JSON from the same file.

The VAMP arm model carries a 0.9144 m pedestal, which is why the table top sits at 0.83 m.
Raise it and the base spheres collide at every pose; lower it and the arms float.
