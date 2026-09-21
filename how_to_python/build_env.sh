#!/usr/bin/env bash
# Compile a two-UR5 VAMP plugin and write env/ur5_bimanual.json beside it.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

python3 "$HERE/../mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name ur5_bimanual \
  --robot-header /usr/local/include/vamp/robots/ur5.hh \
  --robot-struct UR5 \
  --num-robots 2 \
  --move-group both_arms \
  --robot-groups left_arm,right_arm \
  --base-transforms "$HERE/bases.json" \
  --mr-planner-core-prefix /usr/local \
  --output-dir "$HERE/env"
