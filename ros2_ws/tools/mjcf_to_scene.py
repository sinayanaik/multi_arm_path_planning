#!/usr/bin/env python3
"""Turn the MuJoCo cell into the `arms` / `scene` / `display_only` blocks of arms.yaml.

Run this once and commit the result. arms.yaml stays the one readable file the nodes load,
and MuJoCo stays a tool-time dependency rather than a runtime one -- but when the cell
changes in MuJoCo, re-running this is the whole of the update.

Two things make it worth going through MuJoCo rather than parsing the XML directly:

  `<attach>`  bimanual_scene.xml pulls the arm in twice through a model asset with a name
              prefix. Reimplementing that faithfully is a pile of work MuJoCo has already
              done, and getting it subtly wrong would put the arms in the wrong place.

  freejoint   the bins, and the thirty blocks in them, are authored 5-20 cm above the
              surfaces they belong on and drop onto them when the simulation starts. Their
              authored poses are not where they end up, so the cell is stepped until it
              stops moving and the settled poses are what get written out.

VAMP's obstacle backend takes boxes, spheres and cylinders only -- Object::Shape::Mesh is in
the enum but the backend returns nullopt for it -- so the two bodies that collide as raw
meshes in MuJoCo (the cabinet the arms stand on, and the conveyor frame) are emitted as the
axis-aligned box of their own vertices. For a cabinet that is exactly right. For the
conveyor it also fills in the open space under the belt, which is conservative and which no
waypoint would want to reach into anyway.
"""

import argparse
import math
import sys
from pathlib import Path

try:
    import mujoco
except ImportError:
    raise SystemExit("this tool needs MuJoCo to settle the cell:  pip install mujoco")

import numpy as np

# Marker colour, picked off the geom or body name, first match wins. `conveyor_bin` has to
# come before `bin`, and `robot_base` before `base`, or the wrong rule catches them.
ROLES = (("robot_base", "cabinet"), ("cabinet", "cabinet"), ("table", "table"),
         ("conveyor_bin", "target"), ("conveyer", "conveyor"), ("greenmat", "conveyor"),
         ("green", "conveyor"), ("bin", "source"), ("block", "part"))

MESH_PACKAGE = "package://vamp_mr_arms/models"


def role_of(name):
    for needle, role in ROLES:
        if needle in name:
            return role
    return "fixture"


def matrix_to_rpy(m):
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) < 1e-9:
        return 0.0, pitch, math.atan2(-m[0][1], m[1][1])
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def settle(model, data, seconds, tolerance=1e-4):
    """Step until nothing is moving, or until `seconds` of simulated time runs out."""
    steps = int(seconds / model.opt.timestep)
    for step in range(steps):
        mujoco.mj_step(model, data)
        if step % 100 == 0 and step and np.abs(data.qvel).max() < tolerance:
            return data.time, True
    return data.time, np.abs(data.qvel).max() < tolerance


def mesh_vertices(model, geom):
    """The mesh's vertices as MuJoCo stores them -- recentred, see mesh_frame()."""
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    return model.mesh_vert[start:start + model.mesh_vertnum[mesh]].astype(float)


def quat_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mesh_frame(model, data, geom):
    """Where the .obj file's own origin ends up in the world.

    At compile time MuJoCo moves every mesh's vertices into its centre-of-mass frame and
    folds the shift back into the geom, so geom_xpos/geom_xmat place the *recentred*
    vertices. RViz will be handed the original file, so that shift has to come back out --
    otherwise the cabinet is drawn rotated by whatever its inertia happened to be.
    """
    mesh = model.geom_dataid[geom]
    rotation = data.geom_xmat[geom].reshape(3, 3) @ quat_matrix(model.mesh_quat[mesh]).T
    return data.geom_xpos[geom] - rotation @ model.mesh_pos[mesh], rotation


def trim(value, places=5):
    return round(float(value), places)


def emit(entry):
    body = ", ".join(f"{k}: {v}" for k, v in entry.items() if k != "name")
    return f"  - {{name: {entry['name']}, {body}}}"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", required=True, help="models/bimanual_scene.xml")
    parser.add_argument("--output", help="write here; default is stdout")
    parser.add_argument("--settle", type=float, default=4.0,
                        help="seconds of simulated time to let the free bodies drop")
    parser.add_argument("--arm-bodies", default="left_ur5e_base,right_ur5e_base",
                        help="the attached arm root bodies, in arms.yaml order")
    parser.add_argument("--arm-names", default="left,right")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(Path(args.scene).resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    elapsed, quiet = settle(model, data, args.settle)
    print(f"settled after {elapsed:.2f} s of simulated time"
          f"{'' if quiet else ' (STILL MOVING -- raise --settle)'}", file=sys.stderr)
    if not quiet:
        print("warning: bodies were still in motion; the written poses are a snapshot, "
              "not a rest state", file=sys.stderr)

    # Everything under the arms' own bodies belongs to the robot, not the cell.
    robot_bodies = set()
    for name in args.arm_bodies.split(","):
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name.strip())
        if root < 0:
            raise SystemExit(f"{args.scene}: no body named {name.strip()!r}")
        for body in range(model.nbody):
            ancestor = body
            while ancestor > 0:
                if ancestor == root:
                    robot_bodies.add(body)
                    break
                ancestor = model.body_parentid[ancestor]

    scene, skipped, orphan_meshes = [], [], []
    for body in range(model.nbody):
        if body in robot_bodies:
            continue
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        start = model.body_geomadr[body]
        entries, visual = [], None

        for geom in range(start, start + model.body_geomnum[body]):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or body_name
            kind = model.geom_type[geom]
            collides = bool(model.geom_contype[geom] or model.geom_conaffinity[geom])
            mesh_file = None
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = model.geom_dataid[geom]
                mesh_file = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
                position, rotation = mesh_frame(model, data, geom)
                visual = {"mesh": f"{MESH_PACKAGE}/scene_meshes/{mesh_file}.obj",
                          "mesh_xyz": [trim(v) for v in position],
                          "mesh_rpy": [trim(v) for v in matrix_to_rpy(rotation)]}

            if not collides:
                continue
            if kind == mujoco.mjtGeom.mjGEOM_PLANE:
                skipped.append(f"{name} (floor plane -- the arms stand 0.86 m above it)")
                continue

            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                # VAMP takes no meshes, so the obstacle is the box around the vertices.
                # Right for a cabinet; for the conveyor it also fills the space under the
                # belt, which is conservative and which nothing reaches into.
                world = (mesh_vertices(model, geom) @ data.geom_xmat[geom].reshape(3, 3).T
                         + data.geom_xpos[geom])
                lo, hi = world.min(axis=0), world.max(axis=0)
                entries.append({"name": name, "role": role_of(name),
                                "size": [trim(v) for v in (hi - lo)],
                                "xyz": [trim(v) for v in (hi + lo) / 2.0],
                                "rpy": [0.0, 0.0, 0.0]})
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                entries.append({"name": name, "role": role_of(name),
                                "size": [trim(2.0 * v) for v in model.geom_size[geom][:3]],
                                "xyz": [trim(v) for v in data.geom_xpos[geom]],
                                "rpy": [trim(v) for v in
                                        matrix_to_rpy(data.geom_xmat[geom].reshape(3, 3))]})
            else:
                skipped.append(f"{name} ({mujoco.mjtGeom(kind).name})")

        # A body's visual mesh rides along on its first collision box, so RViz draws the
        # cell rather than the blocks the cell is approximated by.
        if visual and entries:
            entries[0].update(visual)
        elif visual:
            orphan_meshes.append(body_name)
        scene.extend(entries)

    arms = []
    for name, body_name in zip(args.arm_names.split(","), args.arm_bodies.split(",")):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name.strip())
        rotation = data.xmat[body].reshape(3, 3)
        arms.append({"name": name.strip(),
                     "xyz": [trim(v) for v in data.xpos[body]],
                     "rpy": [trim(v) for v in matrix_to_rpy(rotation)]})

    lines = [
        "# GENERATED by tools/mjcf_to_scene.py from models/bimanual_scene.xml -- do not",
        "# hand-edit the three blocks below; edit the MuJoCo scene and re-run the tool.",
        "#",
        f"# Free bodies were dropped onto their surfaces first ({elapsed:.2f} s simulated),",
        "# so these are resting poses, not the authored ones.",
        "",
        "arms:",
    ]
    for arm in arms:
        lines.append(f"  - name: {arm['name']}")
        lines.append(f"    base: {{xyz: {arm['xyz']}, rpy: {arm['rpy']}}}")
        lines.append("    ee_frame: tcp")
        lines.append("    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,")
        lines.append("             wrist_1_joint, wrist_2_joint, wrist_3_joint]")
    lines += ["", f"# {len(scene)} collision boxes, straight from the MuJoCo cell.", "scene:"]
    lines += [emit(entry) for entry in scene]
    lines += ["", "display_only: []"]

    text = "\n".join(lines) + "\n"
    if args.output:
        Path(args.output).write_text(text)
        print(f"{args.output}: {len(scene)} collision boxes, {len(arms)} arms", file=sys.stderr)
    else:
        print(text)

    for name in orphan_meshes:
        print(f"note: {name} has a mesh but no collision geom; not drawn and not planned",
              file=sys.stderr)
    for note in skipped:
        print(f"note: skipped {note}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
