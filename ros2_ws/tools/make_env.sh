#!/usr/bin/env bash
# Build the two-arm VAMP environment. VAMP ships one arm model - a UR5 with a Robotiq 85
# gripper on a pedestal - so both arms are planned as UR5s, and the arms shown in RViz
# come straight from ur_description.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORK="$HERE/build"
PKG="$HERE/../src/vamp_mr_arms"
ENV_DIR="$PKG/vamp_env"
PREFIX="${MR_PLANNER_PREFIX:-/usr/local}"
mkdir -p "$WORK" "$ENV_DIR"
WORK="$(cd "$WORK" && pwd)"; ENV_DIR="$(cd "$ENV_DIR" && pwd)"

echo -e "\n== two-UR5 VAMP plugin ==" >&2
python3 "$REPO/mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name two_arms --robot-struct UR5 --num-robots 2 \
  --robot-header "$PREFIX/include/vamp/robots/ur5.hh" \
  --mr-planner-core-prefix "$PREFIX" --output-dir "$WORK"
cp "$WORK"/build/lib/libmr_planner_vamp_plugin_ur5_2r.so "$ENV_DIR/"

echo -e "\n== environment JSON ==" >&2
python3 "$HERE/make_env_json.py" \
  --config "$PKG/config/arms.yaml" \
  --output "$ENV_DIR/two_arms.json" \
  --plugin "$ENV_DIR/libmr_planner_vamp_plugin_ur5_2r.so"

echo -e "\nvamp_env ready. Rebuild the workspace to pick it up:  colcon build --symlink-install"
