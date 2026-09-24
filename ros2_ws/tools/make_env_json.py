#!/usr/bin/env python3
"""Write the environment JSON mr_planner_core loads, taking the arm names and base
transforms straight from cell.yaml -- the same file the ROS nodes read, itself generated
from the MuJoCo scene -- so ROS and the planner cannot drift apart."""

import argparse
import json
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", required=True, help="config/cell.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--plugin", required=True)
    args = parser.parse_args()

    arms = yaml.safe_load(Path(args.cell).read_text())["arms"]
    Path(args.output).write_text(json.dumps({
        "name": Path(args.output).stem,
        "move_group": "both_arms",
        "robot_groups": [arm["name"] for arm in arms],
        "hand_groups": [],
        "vamp_plugin": str(Path(args.plugin).resolve()),
        "base_transforms": [{"translation": arm["base"]["xyz"], "rpy": arm["base"]["rpy"]}
                            for arm in arms],
    }, indent=2) + "\n")
    print(f"{args.output}: {[arm['name'] for arm in arms]}")


if __name__ == "__main__":
    main()
