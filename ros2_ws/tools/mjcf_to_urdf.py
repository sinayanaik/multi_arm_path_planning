#!/usr/bin/env python3
"""Turn the MuJoCo UR5e + Robotiq 2F85 model into the two URDFs this workspace needs.

  models/ur5e.urdf            the arm as RViz draws it, pointing at the original MuJoCo
                              meshes -- robot_state_publisher and joint_state_publisher_gui
                              read this one.
  models/ur5e_spherized.urdf  the same kinematics with every collision shape replaced by
                              spheres, which is the only collision geometry cricket's
                              fkcc_gen accepts. This is what becomes VAMP's UR5e model.

Both come from the same parse, so the robot RViz draws and the robot VAMP checks cannot
drift apart -- which is the whole point of the exercise.

The spheres are derived, not guessed. Every collision shape on the arm is already a capsule
(or, at the wrist, a cylinder), and a capsule is by definition the set of points within `r`
of a line segment -- so a row of spheres along that segment reproduces it exactly, up to the
spacing. Placing them `d` apart and using radius sqrt(r^2 + (d/2)^2) is enough to contain the
capsule: a point at radial distance <= r and at most d/2 along the axis from the nearest
centre is within that distance of it, and the rounded ends are covered by the end spheres at
radius r alone. The only guesswork is the gripper, whose collision geometry is meshes; those
get bounding spheres per slab, checked afterwards against the actual triangles.
"""

import argparse
import math
import struct
import sys
from pathlib import Path
from xml.dom import minidom
import xml.etree.ElementTree as ET

import mjcf

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# VAMP's own UR5 maps the unit hypercube onto +-pi per joint, and the nodes wrap every value
# they hand the planner into that box. Keeping the generated UR5e on the same range keeps
# world.wrap meaningful and the sampled volume the size it is today. A revolute joint at q
# and at q +- 2pi is the same pose, so nothing is unreachable -- see README.
JOINT_LIMIT = math.pi

# forcerange from the MJCF's ur5e:size3 / ur5e:size1 actuator classes.
JOINT_EFFORT = {"wrist_1_joint": 28.0, "wrist_2_joint": 28.0, "wrist_3_joint": 28.0}
DEFAULT_EFFORT = 150.0
JOINT_VELOCITY = math.pi

# The MJCF calls the gripper bodies `base`, `base_mount`, `left_driver`... Next to the arm's
# own `base_link` that reads badly in TF and in a collision message, so everything below the
# wrist gets a prefix.
GRIPPER_PREFIX = "gripper_"

# How many spheres to spend on each gripper collision mesh. The budget matches VAMP's own
# Robotiq 85 (17 spheres over 10 links); coverage is verified against the triangles either
# way, so raising a number here only ever tightens the fit.
MESH_SPHERES = {
    "2f85:base_mount": 2,
    "2f85:base": 3,
    "2f85:driver": 1,
    "2f85:coupler": 2,
    "2f85:spring_link": 2,
    "2f85:follower": 2,
}

MESH_PACKAGE = "package://vamp_mr_arms/models"

# A sphere sized to exactly touch the furthest point it must contain leaves that point on
# the surface, where round-off decides whether it is in or out. One micron of slack settles
# it, and is three orders of magnitude below anything the planner cares about.
MARGIN = 1e-6

# The MJCF gives `ur5e_base` visual geoms only -- in MuJoCo the base is welded to the cabinet
# and never collides with anything, so it needs none. Here it does: with two arms 0.48 m
# apart, a base_link carrying no collision geometry is a hole the *other* arm can plan
# straight through. Measured off base_0.obj / base_1.obj, the casting occupies z in
# [0, 0.099] within a radius of 0.0755, and the shoulder's own capsule already covers from
# z = 0.063 up. This capsule covers the rest. It reaches below the mounting face, which is
# why world.build() allow-lists base_link against the cabinet it is bolted to.
BASE_COLLISION = mjcf.Geom(name="base_link_collision", type_="capsule",
                           size=[0.076, 0.026], pos=[0.0, 0.0, 0.05], rot=mjcf.IDENTITY,
                           mesh=None, scale=[1.0, 1.0, 1.0], group=3, material=None)


# --------------------------------------------------------------------------- spherization


def capsule_spheres(radius, half_length, spacing_factor):
    """Spheres along the local Z segment of a capsule of half-length `half_length`.

    Returns (offset_along_z, radius) pairs. The union contains the capsule; see the module
    docstring for why this radius is the right one.
    """
    span = 2.0 * half_length
    if span <= 0.0:
        return [(0.0, radius)]
    count = max(2, math.ceil(span / (spacing_factor * radius)) + 1)
    step = span / (count - 1)
    grown = math.sqrt(radius * radius + (step / 2.0) ** 2)
    return [(-half_length + i * step, grown) for i in range(count)]


def box_spheres(half_sizes):
    """One sphere round a box. Radius is the half-diagonal, so containment is exact."""
    return [([0.0, 0.0, 0.0], math.sqrt(sum(h * h for h in half_sizes)) + MARGIN)]


def read_stl(path):
    """Vertices and triangles of an STL, binary or ASCII, as plain Python lists."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:2048]:
        values, triangles = [], []
        for line in data.decode("ascii", "replace").splitlines():
            parts = line.split()
            if parts[:1] == ["vertex"]:
                values.append([float(v) for v in parts[1:4]])
        for i in range(0, len(values) - 2, 3):
            triangles.append(values[i:i + 3])
        return triangles
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) < 84 + 50 * count:
        raise ValueError(f"{path}: truncated binary STL")
    triangles = []
    for i in range(count):
        base = 84 + 50 * i + 12                      # skip the per-facet normal
        flat = struct.unpack("<9f", data[base:base + 36])
        triangles.append([list(flat[0:3]), list(flat[3:6]), list(flat[6:9])])
    return triangles


def sample_points(triangles):
    """Vertices, edge midpoints and centroids -- dense enough to verify a sphere cover."""
    points = []
    for a, b, c in triangles:
        points.extend((a, b, c))
        points.append([(a[i] + b[i]) / 2 for i in range(3)])
        points.append([(b[i] + c[i]) / 2 for i in range(3)])
        points.append([(c[i] + a[i]) / 2 for i in range(3)])
        points.append([(a[i] + b[i] + c[i]) / 3 for i in range(3)])
    return points


def mesh_spheres(triangles, scale, count):
    """Cover a mesh with `count` spheres, sliced along its longest axis.

    Slabs are cut on the widest extent and each gets the bounding sphere of the points that
    fall in it; the final pass grows whichever sphere is nearest any point still outside, so
    the result provably contains every sampled point rather than merely looking like it does.
    """
    points = [[p[i] * scale[i] for i in range(3)] for p in sample_points(triangles)]
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    axis = max(range(3), key=lambda i: hi[i] - lo[i])
    extent = hi[axis] - lo[axis]

    buckets = [[] for _ in range(count)]
    for point in points:
        index = 0 if extent == 0 else min(count - 1,
                                          int((point[axis] - lo[axis]) / extent * count))
        buckets[index].append(point)

    spheres = []
    for bucket in buckets:
        if not bucket:
            continue
        centre = [sum(p[i] for p in bucket) / len(bucket) for i in range(3)]
        radius = max(math.dist(centre, p) for p in bucket) + MARGIN
        spheres.append([centre, radius])

    for point in points:                             # grow until nothing is left uncovered
        best, gap = None, 0.0
        for sphere in spheres:
            slack = math.dist(sphere[0], point) - sphere[1]
            if slack <= 1e-12:
                best = None
                break
            if best is None or slack < gap:
                best, gap = sphere, slack
        if best is not None:
            best[1] += gap + MARGIN
    return [(centre, radius) for centre, radius in spheres]


def collision_spheres(geom, model, meshdir, spacing_factor):
    """Every sphere one MJCF collision geom turns into, in its body's frame."""
    if geom.type in ("capsule", "cylinder"):
        radius, half_length = geom.size[0], geom.size[1]
        local = [([0.0, 0.0, offset], grown)
                 for offset, grown in capsule_spheres(radius, half_length, spacing_factor)]
    elif geom.type == "box":
        local = box_spheres(geom.size)
    elif geom.type == "sphere":
        local = [([0.0, 0.0, 0.0], geom.size[0])]
    elif geom.type == "mesh":
        filename, scale = model.meshes[geom.mesh]
        triangles = read_stl(meshdir / filename)
        local = mesh_spheres(triangles, scale, MESH_SPHERES.get(geom.mesh, 2))
    else:
        raise NotImplementedError(f"cannot spherize MJCF geom type {geom.type!r}")

    return [(mjcf.matvec(geom.rot, centre), radius) for centre, radius in local], local


# --------------------------------------------------------------------------- URDF emission


def element(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: v for k, v in attrs.items()})


def origin(parent, pos, rot):
    roll, pitch, yaw = mjcf.matrix_to_rpy(rot)
    element(parent, "origin",
            xyz=" ".join(f"{v:.9g}" for v in pos),
            rpy=f"{roll:.9g} {pitch:.9g} {yaw:.9g}")


def link_name(body, gripper_bodies):
    if body.name == "ur5e_base":
        return "base_link"
    return (GRIPPER_PREFIX + body.name) if body.name in gripper_bodies else body.name


def build(model, meshdir, spacing_factor, spherized):
    """One URDF element tree. `spherized` swaps meshes for spheres and drops the visuals."""
    root = ET.Element("robot", name="ur5e_2f85")
    bodies = model.chain("ur5e_base")
    wrist_3 = model.bodies["wrist_3_link"]
    gripper_bodies = {b.name for b in model.chain("base_mount")}

    counts = {}
    for body in bodies:
        name = link_name(body, gripper_bodies)
        link = element(root, "link", name=name)

        if not spherized:
            for geom in body.geoms:
                if geom.is_collision or geom.mesh is None:
                    continue
                visual = element(link, "visual")
                origin(visual, geom.pos, geom.rot)
                filename, scale = model.meshes[geom.mesh]
                geometry = element(visual, "geometry")
                element(geometry, "mesh",
                        filename=f"{MESH_PACKAGE}/{model.meshdir}/{filename}",
                        scale=" ".join(f"{v:.9g}" for v in scale))
                rgba = model.materials.get(geom.material)
                if rgba:
                    material = element(visual, "material", name=geom.material)
                    element(material, "color", rgba=" ".join(f"{v:.4g}" for v in rgba))

        total = 0
        geoms = list(body.geoms)
        if body.name == "ur5e_base":
            geoms.append(BASE_COLLISION)
        for geom in geoms:
            if not geom.is_collision:
                continue
            placed, _ = collision_spheres(geom, model, meshdir, spacing_factor)
            for offset, radius in placed:
                centre = [geom.pos[i] + offset[i] for i in range(3)]
                collision = element(link, "collision")
                origin(collision, centre, mjcf.IDENTITY)
                element(element(collision, "geometry"), "sphere", radius=f"{radius:.9g}")
                total += 1
        if total:
            counts[name] = total

        # Pinocchio refuses a URDF whose moving links have no mass, and cricket goes through
        # Pinocchio. The MJCF's own inertials are not carried over: they play no part in
        # collision checking, and a wrong one would be worse than an obvious placeholder.
        if body.parent is not None:
            inertial = element(link, "inertial")
            element(inertial, "mass", value="1")
            element(inertial, "inertia", ixx="0.01", ixy="0", ixz="0",
                    iyy="0.01", iyz="0", izz="0.01")

        if body.parent is None:
            continue

        parent_name = link_name(body.parent, gripper_bodies)
        arm_joint = next((j for j in body.joints if j.name in ARM_JOINTS), None)
        joint_name = arm_joint.name if arm_joint else f"{name}_fixed_joint"
        joint = element(root, "joint", name=joint_name,
                        type="revolute" if arm_joint else "fixed")
        element(joint, "parent", link=parent_name)
        element(joint, "child", link=name)
        origin(joint, body.pos, body.rot)
        if arm_joint:
            if any(abs(v) > 1e-9 for v in arm_joint.pos):
                raise NotImplementedError(
                    f"{joint_name}: MJCF joint is offset from its body origin, which URDF "
                    "cannot express without an extra link")
            element(joint, "axis", xyz=" ".join(f"{v:.9g}" for v in arm_joint.axis))
            element(joint, "limit",
                    lower=f"{-JOINT_LIMIT:.9g}", upper=f"{JOINT_LIMIT:.9g}",
                    effort=f"{JOINT_EFFORT.get(joint_name, DEFAULT_EFFORT):.9g}",
                    velocity=f"{JOINT_VELOCITY:.9g}")

    # The grasp point, as a frame the planner and RViz can both name. VAMP reports the
    # end-effector here and arms.yaml's routine targets are this point, so a waypoint means
    # "put the pinch point there" instead of "put the gripper's mounting flange there".
    pinch = next((s for s in model.bodies["base"].sites if s[0] == "pinch"), None)
    if pinch is None:
        raise SystemExit("ur5e.xml: no 'pinch' site on the gripper base to hang tcp off")
    element(root, "link", name="tcp")
    tcp = element(root, "joint", name="tcp_fixed_joint", type="fixed")
    element(tcp, "parent", link=GRIPPER_PREFIX + "base")
    element(tcp, "child", link="tcp")
    origin(tcp, pinch[1], pinch[2])

    return root, counts, wrist_3


def write(root, path):
    text = minidom.parseString(ET.tostring(root, "unicode")).toprettyxml(indent="  ")
    path.write_text("\n".join(line for line in text.splitlines() if line.strip()) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mjcf", required=True, help="models/ur5e.xml")
    parser.add_argument("--display", required=True, help="URDF written for RViz")
    parser.add_argument("--spherized", required=True, help="URDF written for cricket")
    parser.add_argument("--spacing-factor", type=float, default=1.0,
                        help="sphere spacing along a capsule, as a multiple of its radius; "
                             "smaller packs more spheres and inflates them less "
                             "(1.0 -> 12%% fatter, 0.6 -> 4%%)")
    args = parser.parse_args()

    source = Path(args.mjcf).resolve()
    model = mjcf.Model(source)
    meshdir = source.parent / model.meshdir

    display, _, _ = build(model, meshdir, args.spacing_factor, spherized=False)
    write(display, Path(args.display))

    spheres, counts, _ = build(model, meshdir, args.spacing_factor, spherized=True)
    write(spheres, Path(args.spherized))

    total = sum(counts.values())
    print(f"{args.display}: display model")
    print(f"{args.spherized}: {total} spheres over {len(counts)} links")
    for name, count in counts.items():
        print(f"    {name:28s} {count}")
    if total > 120:
        print(f"\nwarning: {total} spheres is a lot -- every collision check scales with it. "
              "Raise --spacing-factor to trade accuracy for speed.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
