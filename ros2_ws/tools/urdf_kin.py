"""Forward kinematics and sphere placement for a URDF, with no ROS in the loop.

Used for two jobs that both need to know where a link's spheres actually are: building the
SRDF's allowed-collision matrix, and checking the generated model against MuJoCo's own
kinematics. Deliberately independent of `mjcf.py`'s view of the robot -- it reads the URDF
that was written out, so a mistake in the conversion shows up as a disagreement rather than
being reproduced identically on both sides.
"""

import math
import xml.etree.ElementTree as ET

from mjcf import IDENTITY, compose, matmul, matvec


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def axis_angle_to_matrix(axis, angle):
    norm = math.sqrt(sum(v * v for v in axis))
    x, y, z = (v / norm for v in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _origin(element):
    child = element.find("origin")
    if child is None:
        return ([0.0, 0.0, 0.0], IDENTITY)
    xyz = [float(v) for v in child.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in child.get("rpy", "0 0 0").split()]
    return (xyz, rpy_to_matrix(*rpy))


class Robot:
    """A URDF's tree, its movable joints in order, and each link's collision spheres."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.name = root.get("name")

        self.spheres = {}                    # link -> [(centre in link frame, radius)]
        for link in root.findall("link"):
            placed = []
            for collision in link.findall("collision"):
                sphere = collision.find("geometry/sphere")
                if sphere is None:
                    continue
                centre, _ = _origin(collision)
                placed.append((centre, float(sphere.get("radius"))))
            self.spheres[link.get("name")] = placed

        self.parent = {}                     # link -> (parent link, origin, axis, movable)
        self.joint_names = []
        self.limits = {}
        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            axis = joint.find("axis")
            movable = joint.get("type") in ("revolute", "continuous", "prismatic")
            self.parent[child] = (
                joint.find("parent").get("link"),
                _origin(joint),
                [float(v) for v in axis.get("xyz").split()] if axis is not None else None,
                joint.get("name") if movable else None)
            if movable:
                self.joint_names.append(joint.get("name"))
                limit = joint.find("limit")
                self.limits[joint.get("name")] = (float(limit.get("lower")),
                                                  float(limit.get("upper")))

        children = set(self.parent)
        self.root = next(name for name in self.spheres if name not in children)
        self.links = self._ordered()

    def _ordered(self):
        """Links with every parent ahead of its children, so one pass computes all of FK."""
        out, pending = [self.root], dict(self.parent)
        while pending:
            progressed = False
            for link, (parent, *_rest) in list(pending.items()):
                if parent in out:
                    out.append(link)
                    del pending[link]
                    progressed = True
            if not progressed:
                raise ValueError(f"URDF {self.name}: links detached from {self.root}")
        return out

    def frames(self, configuration, base=None):
        """World pose of every link, given {joint name: value} or a list in joint order."""
        if not isinstance(configuration, dict):
            configuration = dict(zip(self.joint_names, configuration))
        poses = {self.root: base or ([0.0, 0.0, 0.0], IDENTITY)}
        for link in self.links[1:]:
            parent, origin, axis, joint = self.parent[link]
            local = origin
            if joint is not None:
                turned = axis_angle_to_matrix(axis, configuration.get(joint, 0.0))
                local = (origin[0], matmul(origin[1], turned))
            poses[link] = compose(poses[parent], local)
        return poses

    def placed_spheres(self, configuration, base=None):
        """(link, world centre, radius) for every collision sphere."""
        poses = self.frames(configuration, base)
        out = []
        for link, spheres in self.spheres.items():
            position, rotation = poses[link]
            for centre, radius in spheres:
                moved = matvec(rotation, centre)
                out.append((link, [position[i] + moved[i] for i in range(3)], radius))
        return out


def colliding_pairs(spheres, skip=()):
    """Link pairs with at least one overlapping sphere. `skip` is a set of frozenset pairs."""
    hits = set()
    for i, (link_a, centre_a, radius_a) in enumerate(spheres):
        for link_b, centre_b, radius_b in spheres[i + 1:]:
            if link_a == link_b:
                continue
            pair = frozenset((link_a, link_b))
            if pair in hits or pair in skip:
                continue
            if math.dist(centre_a, centre_b) < radius_a + radius_b:
                hits.add(pair)
    return hits
