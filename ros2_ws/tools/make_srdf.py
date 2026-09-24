#!/usr/bin/env python3
"""Write the SRDF that cricket reads alongside the spherized URDF.

The only part of an SRDF that matters here is `<disable_collisions>`: which link pairs VAMP
should not bother checking against each other. Getting it wrong is expensive in both
directions -- too few disables and the arm is permanently self-colliding at every pose, too
many and it can fold through itself unnoticed.

Four sources, in order of how much they are trusted:

  adjacent   a link and its parent always touch; this is structural, not statistical.
  excluded   the MJCF's own <contact><exclude> pairs. These are the gripper's four-bar
             linkage, which the author already determined overlaps by construction.
  always     pairs that overlap in every configuration sampled. Usually two spheres that
             were fattened past each other by the capsule conversion.
  never      pairs that overlap in none of them. This is the statistical one, so it runs a
             lot of samples and is the reason --trials defaults high.

MoveIt's collisions_updater does the same job, but it is not installed here and it would
reason about the meshes; the spheres are what VAMP actually checks.
"""

import argparse
import random
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

import mjcf
import urdf_kin


def chain_pairs(robot):
    pairs = set()
    for link, (parent, *_rest) in robot.parent.items():
        pairs.add(frozenset((link, parent)))
    return pairs


def sample(robot, trials, seed):
    """Pairs that collide at least once, and pairs that collide every time."""
    rng = random.Random(seed)
    ever, always = set(), None
    for _ in range(trials):
        configuration = [rng.uniform(*robot.limits[name]) for name in robot.joint_names]
        hits = urdf_kin.colliding_pairs(robot.placed_spheres(configuration))
        ever |= hits
        always = hits if always is None else (always & hits)
    return ever, (always or set())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", required=True, help="models/ur5e_spherized.urdf")
    parser.add_argument("--mjcf", required=True, help="models/ur5e.xml, for its <exclude>s")
    parser.add_argument("--output", required=True)
    parser.add_argument("--group", default="manipulator")
    parser.add_argument("--tip", default="tcp")
    parser.add_argument("--trials", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    robot = urdf_kin.Robot(args.urdf)
    model = mjcf.Model(Path(args.mjcf))
    gripper = {b.name for b in model.chain("base_mount")}

    def urdf_name(body):
        return ("gripper_" + body) if body in gripper else body

    adjacent = chain_pairs(robot)
    excluded = {frozenset((urdf_name(a), urdf_name(b))) for a, b in model.excludes}
    missing = {name for pair in excluded for name in pair} - set(robot.spheres)
    if missing:
        raise SystemExit(f"{args.mjcf}: <exclude> names no link in the URDF: {sorted(missing)}")

    ever, always = sample(robot, args.trials, args.seed)
    every_pair = {frozenset((a, b)) for a in robot.spheres for b in robot.spheres if a != b}
    never = every_pair - ever

    reasons = {}
    for pair, reason in ((adjacent, "Adjacent"), (excluded, "Excluded in MJCF"),
                         (always, "Always in collision"), (never, "Never in collision")):
        for entry in pair:
            reasons.setdefault(entry, reason)

    root = ET.Element("robot", name=robot.name)
    group = ET.SubElement(root, "group", name=args.group)
    ET.SubElement(group, "chain", base_link=robot.root, tip_link=args.tip)
    for pair in sorted(reasons, key=lambda p: sorted(p)):
        first, second = sorted(pair)
        ET.SubElement(root, "disable_collisions",
                      link1=first, link2=second, reason=reasons[pair])

    text = minidom.parseString(ET.tostring(root, "unicode")).toprettyxml(indent="  ")
    Path(args.output).write_text(
        "\n".join(line for line in text.splitlines() if line.strip()) + "\n")

    tally = {}
    for reason in reasons.values():
        tally[reason] = tally.get(reason, 0) + 1
    print(f"{args.output}: {len(reasons)} of {len(every_pair)} link pairs disabled "
          f"after {args.trials} samples")
    for reason, count in sorted(tally.items()):
        print(f"    {reason:24s} {count}")

    live = every_pair - set(reasons)
    print(f"    {'still checked':24s} {len(live)}")
    if not live:
        raise SystemExit("every link pair was disabled -- the model would never self-collide")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
