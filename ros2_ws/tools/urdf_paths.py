"""Leave no package:// or relative mesh URI behind: foam, cricket and pinocchio all want
plain absolute paths, and arms.py turns those into file:// for RViz."""

import xml.etree.ElementTree as ET
from pathlib import Path


def absolutize_meshes(robot, fallback_dir, packages=None):
    # vamp's URDFs say package://meshes/... for files that live in <urdf dir>/meshes.
    packages = {"meshes": Path(fallback_dir) / "meshes", **(packages or {})}
    for mesh in robot.iter("mesh"):
        name = mesh.get("filename")
        if name.startswith("package://"):
            package, _, rest = name[len("package://"):].partition("/")
            mesh.set("filename", str(Path(packages.get(package, fallback_dir)) / rest))
        elif not name.startswith("/"):
            mesh.set("filename", str(Path(fallback_dir) / name))


def rewrite(urdf_path, output_path, packages=None):
    urdf_path = Path(urdf_path).resolve()
    robot = ET.parse(urdf_path).getroot()
    absolutize_meshes(robot, urdf_path.parent, packages)
    ET.indent(robot, "  ")
    Path(output_path).write_text(ET.tostring(robot, encoding="unicode") + "\n")


if __name__ == "__main__":
    import sys

    rewrite(sys.argv[1], sys.argv[2])
    print(f"{sys.argv[2]}: mesh paths absolutized")
