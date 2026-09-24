#!/usr/bin/env python3
"""Sanity-check the imported cell against the sphere model, without VAMP in the loop.

VAMP is the authority on collision once the plugin exists, but this answers the question
that comes first: is this cell even workable? A table 5 cm too high, or an allow-list that
is one link short, shows up here as "every pose collides" -- and it shows up before an hour
of building cricket rather than after.

Same geometry VAMP will use: the spheres from ur5e_spherized.urdf against the boxes from
cell.yaml, with each arm at its own mount transform.
"""

import argparse
import math
import random
from pathlib import Path

import yaml

import urdf_kin
from urdf_kin import rpy_to_matrix

# Must match world.py.
MOUNT_OBJECT = "robot_base"
MOUNTED_LINKS = {"base_link", "shoulder_link"}


def box_distance(centre, box_pos, box_rot, half):
    """Distance from a point to an oriented box; 0 inside."""
    local = [sum(box_rot[r][i] * (centre[r] - box_pos[r]) for r in range(3)) for i in range(3)]
    outside = [max(abs(local[i]) - half[i], 0.0) for i in range(3)]
    return math.sqrt(sum(v * v for v in outside))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", required=True, help="models/ur5e_spherized.urdf")
    parser.add_argument("--cell", required=True, help="config/cell.yaml")
    parser.add_argument("--trials", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    robot = urdf_kin.Robot(args.urdf)
    cell = yaml.safe_load(Path(args.cell).read_text())
    boxes = [(e["name"], e["xyz"], rpy_to_matrix(*e.get("rpy", (0, 0, 0))),
              [s / 2.0 for s in e["size"]]) for e in cell["scene"]]
    bases = [([*a["base"]["xyz"]], rpy_to_matrix(*a["base"]["rpy"])) for a in cell["arms"]]
    names = [a["name"] for a in cell["arms"]]

    def hits(configuration, base):
        found = set()
        for link, centre, radius in robot.placed_spheres(configuration, base):
            for box_name, pos, rot, half in boxes:
                if box_name == MOUNT_OBJECT and link in MOUNTED_LINKS:
                    continue
                if box_distance(centre, pos, rot, half) < radius:
                    found.add((link, box_name))
        return found

    tucked = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
    print("tucked pose:")
    for name, base in zip(names, bases):
        found = hits(tucked, base)
        print(f"  {name:6s} {'clear' if not found else sorted(found)}")

    rng = random.Random(args.seed)
    free = 0
    tally = {}
    for _ in range(args.trials):
        configuration = [rng.uniform(-math.pi, math.pi) for _ in robot.joint_names]
        found = hits(configuration, bases[0])
        if not found:
            free += 1
        for _link, box_name in found:
            tally[box_name] = tally.get(box_name, 0) + 1

    print(f"\n{args.trials} random poses for {names[0]}: {free} clear of the cell "
          f"({100.0 * free / args.trials:.1f}%)")
    print("most-hit obstacles:")
    for box_name, count in sorted(tally.items(), key=lambda kv: -kv[1])[:6]:
        print(f"    {box_name:24s} {count}")

    # The workbench is the one thing the arm is inside by construction; if it still reports
    # hits, the allow-list in world.py is naming the wrong links.
    leaked = {link for _ in range(1) for link, box in hits(tucked, bases[0]) if box == MOUNT_OBJECT}
    if leaked:
        print(f"\nwarning: {sorted(leaked)} hit {MOUNT_OBJECT} at the tucked pose and are not "
              f"in MOUNTED_LINKS -- world.build() will reject every pose")
    if free == 0:
        raise SystemExit("\nno random pose was clear: the cell blocks the arm everywhere")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
