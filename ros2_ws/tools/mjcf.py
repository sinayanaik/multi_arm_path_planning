"""Just enough MJCF to read the UR5e model: default-class inheritance, the body tree,
and the pose/shape of every geom.

MuJoCo and URDF agree on more than they disagree: a body's pos/quat is the parent-to-child
transform, and with a joint at the body origin (which every UR5e joint has) that is exactly
a URDF joint origin with the joint axis carried over unchanged. What MJCF adds on top, and
what this module exists to undo, is `<default>`: attributes are inherited from a named class
which is itself nested inside another class, and a body's `childclass` silently supplies that
name to everything beneath it.
"""

import math
import xml.etree.ElementTree as ET

# Elements a <default> block can carry defaults for. Anything else in there is ignored.
DEFAULTABLE = ("geom", "joint", "mesh", "site", "general", "material")


def quat_to_matrix(w, x, y, z):
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def matrix_to_rpy(m):
    """URDF fixed-axis roll-pitch-yaw, i.e. the r/p/y with R = Rz(y) Ry(p) Rx(r)."""
    pitch = math.atan2(-m[2][0], math.sqrt(m[0][0] ** 2 + m[1][0] ** 2))
    if abs(math.cos(pitch)) < 1e-9:          # gimbal lock: fold roll into yaw
        return 0.0, pitch, math.atan2(-m[0][1], m[1][1])
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def compose(outer, inner):
    """(pos, rot) of `inner` expressed in the frame `outer` is expressed in."""
    outer_pos, outer_rot = outer
    inner_pos, inner_rot = inner
    moved = matvec(outer_rot, inner_pos)
    return ([outer_pos[i] + moved[i] for i in range(3)], matmul(outer_rot, inner_rot))


def numbers(text):
    return [float(token) for token in text.replace(",", " ").split()]


def orientation(attrs):
    """MJCF offers five ways to write a rotation. Read whichever one is present."""
    if "quat" in attrs:
        return quat_to_matrix(*numbers(attrs["quat"]))
    if "euler" in attrs:                      # radians, because the file says angle="radian"
        roll, pitch, yaw = numbers(attrs["euler"])
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                  math.sin(pitch), math.cos(yaw), math.sin(yaw))
        return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr]]
    if "axisangle" in attrs:
        ax, ay, az, angle = numbers(attrs["axisangle"])
        scale = math.sqrt(ax * ax + ay * ay + az * az)
        half = math.sin(angle / 2.0) / scale
        return quat_to_matrix(math.cos(angle / 2.0), ax * half, ay * half, az * half)
    for unsupported in ("xyaxes", "zaxis"):
        if unsupported in attrs:
            raise NotImplementedError(f"MJCF {unsupported}= is not handled")
    return IDENTITY


def pose(attrs):
    return (numbers(attrs.get("pos", "0 0 0")), orientation(attrs))


class Defaults:
    """The <default> tree, flattened so one lookup gives a class's full inherited attributes."""

    def __init__(self, root):
        self.classes = {}
        for block in root.findall("default"):
            self._walk(block, {})

    def _walk(self, block, inherited):
        merged = {tag: dict(attrs) for tag, attrs in inherited.items()}
        for tag in DEFAULTABLE:
            for element in block.findall(tag):
                merged.setdefault(tag, {}).update(
                    {k: v for k, v in element.attrib.items() if k != "class"})
        name = block.get("class")
        if name:
            self.classes[name] = merged
        for nested in block.findall("default"):
            self._walk(nested, merged)

    def apply(self, element, childclass):
        """The element's own attributes, over its class's, over the enclosing childclass's."""
        attrs = {}
        for name in (childclass, element.get("class")):
            if name:
                if name not in self.classes:
                    raise KeyError(f"unknown MJCF default class {name!r}")
                attrs.update(self.classes[name].get(element.tag, {}))
        attrs.update(element.attrib)
        return attrs


class Geom:
    __slots__ = ("name", "type", "size", "pos", "rot", "mesh", "scale", "group", "material")

    def __init__(self, name, type_, size, pos, rot, mesh, scale, group, material):
        self.name, self.type, self.size = name, type_, size
        self.pos, self.rot = pos, rot
        self.mesh, self.scale, self.group, self.material = mesh, scale, group, material

    @property
    def is_collision(self):
        """MuJoCo convention in this file: group 3 collides, group 2 is for looking at.

        The class defaults make this explicit -- `ur5e:visual` and `2f85:visual` also set
        contype/conaffinity to 0 -- so group alone is enough to tell the two apart.
        """
        return self.group == 3

    def __repr__(self):
        return f"<Geom {self.name or self.type} {self.type} size={self.size}>"


class Body:
    __slots__ = ("name", "pos", "rot", "joints", "geoms", "sites", "children", "parent")

    def __init__(self, name, pos, rot, parent):
        self.name, self.pos, self.rot, self.parent = name, pos, rot, parent
        self.joints, self.geoms, self.sites, self.children = [], [], [], []

    def __repr__(self):
        return f"<Body {self.name} {len(self.geoms)} geoms {len(self.children)} children>"


class Joint:
    __slots__ = ("name", "axis", "range", "pos", "type")

    def __init__(self, name, axis, range_, pos, type_):
        self.name, self.axis, self.range, self.pos, self.type = name, axis, range_, pos, type_


class Model:
    """A parsed MJCF file: mesh assets by name, and the body tree under <worldbody>."""

    def __init__(self, path):
        self.path = path
        root = ET.parse(path).getroot()
        self.name = root.get("model", path.stem)
        self.defaults = Defaults(root)

        compiler = root.find("compiler")
        self.meshdir = compiler.get("meshdir", "") if compiler is not None else ""

        self.meshes = {}
        self.materials = {}
        for asset in root.findall("asset"):
            for mesh in asset.findall("mesh"):
                attrs = self.defaults.apply(mesh, None)
                scale = numbers(attrs.get("scale", "1 1 1"))
                self.meshes[attrs["name"]] = (attrs["file"], scale)
            # The .obj files name .mtl files that are not shipped with them, so MuJoCo's own
            # material assets are the only colour the model actually carries.
            for material in asset.findall("material"):
                attrs = self.defaults.apply(material, None)
                if "rgba" in attrs:
                    self.materials[attrs["name"]] = numbers(attrs["rgba"])

        self.bodies = {}
        self.roots = []
        for worldbody in root.findall("worldbody"):
            for body in worldbody.findall("body"):
                self.roots.append(self._body(body, None, None))

        self.excludes = [(e.get("body1"), e.get("body2"))
                         for contact in root.findall("contact")
                         for e in contact.findall("exclude")]

    def _body(self, element, parent, childclass):
        childclass = element.get("childclass", childclass)
        position, rotation = pose(element.attrib)
        body = Body(element.get("name"), position, rotation, parent)
        self.bodies[body.name] = body

        for joint in element.findall("joint"):
            attrs = self.defaults.apply(joint, childclass)
            body.joints.append(Joint(
                attrs.get("name"),
                numbers(attrs.get("axis", "0 0 1")),
                numbers(attrs["range"]) if "range" in attrs else None,
                numbers(attrs.get("pos", "0 0 0")),
                attrs.get("type", "hinge")))
        if element.find("freejoint") is not None:
            body.joints.append(Joint(element.find("freejoint").get("name"),
                                     [0, 0, 0], None, [0, 0, 0], "free"))

        for geom in element.findall("geom"):
            attrs = self.defaults.apply(geom, childclass)
            geom_pos, geom_rot = pose(attrs)
            mesh = attrs.get("mesh")
            body.geoms.append(Geom(
                attrs.get("name"), attrs.get("type", "sphere"),
                numbers(attrs["size"]) if "size" in attrs else [],
                geom_pos, geom_rot, mesh,
                self.meshes[mesh][1] if mesh in self.meshes else [1.0, 1.0, 1.0],
                int(attrs.get("group", 0)), attrs.get("material")))

        for site in element.findall("site"):
            attrs = self.defaults.apply(site, childclass)
            site_pos, site_rot = pose(attrs)
            body.sites.append((attrs.get("name"), site_pos, site_rot))

        for child in element.findall("body"):
            body.children.append(self._body(child, body, childclass))
        return body

    def chain(self, name):
        """Every body at or below `name`, parents before children."""
        out, stack = [], [self.bodies[name]]
        while stack:
            body = stack.pop(0)
            out.append(body)
            stack.extend(body.children)
        return out
