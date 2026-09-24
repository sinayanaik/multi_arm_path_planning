#!/usr/bin/env python3
"""Check the generated UR5e against the MuJoCo model it came from.

Two independent questions, because a conversion can fail at either one:

  kinematics  does the URDF put every link where MuJoCo puts it? Random configurations are
              driven through both and the link poses compared. MuJoCo's own mj_forward is
              the reference; nothing in tools/ is involved on that side.

  containment do the spheres actually cover the shapes they replaced? Points are sampled
              over each MJCF collision capsule, box and mesh, and every one must fall inside
              some sphere on that link. A sphere cover that misses is worse than useless:
              the planner would report clearance the robot does not have.

Poses are compared in the base_link frame rather than the world frame, so this tests the
conversion and not the mount transform -- where the arm is bolted comes from cell.yaml and
is MuJoCo's own number either way.
"""

import argparse
import math
import random
import sys
from pathlib import Path

try:
    import mujoco
except ImportError:
    raise SystemExit("this tool needs MuJoCo to check against:  pip install mujoco")

import numpy as np

import urdf_kin

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GRIPPER_PREFIX = "gripper_"


def urdf_link(body):
    return "base_link" if body == "ur5e_base" else body


def link_for(model, body_id, urdf_links):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    for candidate in (urdf_link(name), GRIPPER_PREFIX + name):
        if candidate in urdf_links:
            return candidate
    return None


def sample_geom_points(model, geom, rings=6, steps=9):
    """Points covering one MJCF collision geom, in that geom's own frame."""
    kind, size = model.geom_type[geom], model.geom_size[geom]
    points = []
    if kind in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        radius, half = float(size[0]), float(size[1])
        for i in range(steps):
            z = -half + 2 * half * i / (steps - 1)
            for j in range(rings):
                angle = 2 * math.pi * j / rings
                points.append([radius * math.cos(angle), radius * math.sin(angle), z])
        if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:      # the rounded ends
            for sign in (-1.0, 1.0):
                for j in range(rings):
                    angle = 2 * math.pi * j / rings
                    for tilt in (0.25, 0.5, 0.75):
                        pitch = tilt * math.pi / 2
                        points.append([radius * math.cos(pitch) * math.cos(angle),
                                       radius * math.cos(pitch) * math.sin(angle),
                                       sign * (half + radius * math.sin(pitch))])
        else:                                          # flat caps
            for sign in (-1.0, 1.0):
                for j in range(rings):
                    angle = 2 * math.pi * j / rings
                    for frac in (0.34, 0.67, 1.0):
                        points.append([radius * frac * math.cos(angle),
                                       radius * frac * math.sin(angle), sign * half])
    elif kind == mujoco.mjtGeom.mjGEOM_BOX:
        a, b, c = (float(v) for v in size[:3])
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    points.append([sx * a, sy * b, sz * c])
    elif kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        points = model.mesh_vert[start:start + model.mesh_vertnum[mesh]].astype(float).tolist()
    else:
        return None
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mjcf", required=True, help="models/ur5e.xml")
    parser.add_argument("--urdf", required=True, help="models/ur5e.urdf")
    parser.add_argument("--spherized", required=True, help="models/ur5e_spherized.urdf")
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--tolerance", type=float, default=1e-4,
                        help="metres of disagreement allowed between the two kinematics")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(Path(args.mjcf).resolve()))
    data = mujoco.MjData(model)
    robot = urdf_kin.Robot(args.urdf)
    spheres = urdf_kin.Robot(args.spherized)

    qpos = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINTS]
    if any(j < 0 for j in qpos):
        raise SystemExit(f"{args.mjcf}: missing one of {ARM_JOINTS}")
    addresses = [model.jnt_qposadr[j] for j in qpos]

    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ur5e_base")
    tracked = [(body, link_for(model, body, robot.spheres)) for body in range(model.nbody)]
    tracked = [(body, link) for body, link in tracked if link]

    # ---- kinematics -------------------------------------------------------------------
    rng = random.Random(args.seed)
    worst, worst_where = 0.0, None
    for _ in range(args.trials):
        configuration = [rng.uniform(-math.pi, math.pi) for _ in ARM_JOINTS]
        for address, value in zip(addresses, configuration):
            data.qpos[address] = value
        mujoco.mj_forward(model, data)

        frames = robot.frames(dict(zip(ARM_JOINTS, configuration)))
        base_pos = data.xpos[base].copy()
        base_rot = data.xmat[base].reshape(3, 3).copy()
        for body, link in tracked:
            # MuJoCo pose expressed in base_link, which is what the URDF FK returns.
            relative = base_rot.T @ (data.xpos[body] - base_pos)
            error = float(np.linalg.norm(np.asarray(frames[link][0]) - relative))
            if error > worst:
                worst, worst_where = error, (link, [round(v, 3) for v in configuration])

    print(f"kinematics : {args.trials} configurations, worst link disagreement "
          f"{worst * 1000:.4f} mm")
    if worst_where:
        print(f"             (at {worst_where[0]}, q = {worst_where[1]})")

    # ---- containment ------------------------------------------------------------------
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    base_pos = data.xpos[base].copy()
    base_rot = data.xmat[base].reshape(3, 3).copy()
    rest = spheres.frames([0.0] * len(spheres.joint_names))

    checked, escaped, tightest = 0, [], None
    for geom in range(model.ngeom):
        if model.geom_group[geom] != 3:
            continue
        link = link_for(model, model.geom_bodyid[geom], spheres.spheres)
        if link is None:
            continue
        local = sample_geom_points(model, geom)
        if local is None:
            continue
        scale = model.geom_size[geom] if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH else None
        geom_pos = data.geom_xpos[geom]
        geom_rot = data.geom_xmat[geom].reshape(3, 3)
        placed = np.asarray(local) @ geom_rot.T + geom_pos
        placed = (placed - base_pos) @ base_rot                      # into base_link

        centres = []
        position, rotation = rest[link]
        for centre, radius in spheres.spheres[link]:
            moved = np.asarray(rotation) @ np.asarray(centre) + np.asarray(position)
            centres.append((moved, radius))
        if not centres:
            escaped.append((link, "link has no spheres at all"))
            continue

        for point in placed:
            checked += 1
            # Covered means inside SOME sphere, so the best sphere is the one that counts.
            slack = max(radius - float(np.linalg.norm(point - centre))
                        for centre, radius in centres)
            if slack < 0:
                escaped.append((link, f"a point sits {-slack * 1000:.2f} mm outside every sphere"))
            if tightest is None or slack < tightest[1]:
                tightest = (link, slack)

    unique = sorted({f"{link}: {why}" for link, why in escaped})
    print(f"containment: {checked} points sampled over the MuJoCo collision geometry, "
          f"{len(escaped)} outside every sphere")
    if tightest:
        print(f"             tightest margin {tightest[1] * 1000:+.2f} mm on {tightest[0]}")
    for line in unique[:10]:
        print(f"             ! {line}")

    ok = worst <= args.tolerance and not escaped
    print("\n" + ("PASS -- the generated model is the MuJoCo model"
                  if ok else "FAIL -- see above"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
