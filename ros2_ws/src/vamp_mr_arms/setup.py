import os
from glob import glob

from setuptools import setup

package_name = "vamp_mr_arms"


def model_files():
    """models/ verbatim, meshes included -- the URDFs reference them by package:// path."""
    out = []
    for directory, _subdirs, names in os.walk("models"):
        if names:
            out.append((f"share/{package_name}/{directory}",
                        [os.path.join(directory, name) for name in names]))
    return out

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/vamp_env", glob("vamp_env/*")),
    ] + model_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sinaya Naik",
    maintainer_email="sinayanaik@gmail.com",
    description="Spawn, hand-teach and VAMP-MR plan two UR5e arms in RViz.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"teach = {package_name}.teach:main",
            f"plan = {package_name}.plan:main",
        ],
    },
)
