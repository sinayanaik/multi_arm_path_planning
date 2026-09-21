#!/usr/bin/env bash
# Build the two-arm VAMP environment. VAMP ships a UR5 collision model and no UR7e, so
# both arms are planned as UR5s; the UR7e's real kinematics are used for display only.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORK="$HERE/build"
PKG="$HERE/../src/vamp_mr_arms"
ENV_DIR="$PKG/vamp_env"
UR_DESCRIPTION="$HERE/../src/ur_description"
DONOR="$REPO/vamp/resources/ur5/ur5.urdf"
PREFIX="${MR_PLANNER_PREFIX:-/usr/local}"
mkdir -p "$WORK" "$ENV_DIR"
WORK="$(cd "$WORK" && pwd)"; ENV_DIR="$(cd "$ENV_DIR" && pwd)"

step() { echo -e "\n== $* ==" >&2; }

step "UR5 display model"
python3 "$HERE/urdf_paths.py" "$DONOR" "$ENV_DIR/ur5.urdf"

step "UR7e display model"
xacro "$UR_DESCRIPTION/urdf/ur.urdf.xacro" ur_type:=ur7e name:=ur7e > "$WORK/ur7e_arm.urdf"
python3 "$HERE/compose_ur7e.py" \
  --arm-urdf "$WORK/ur7e_arm.urdf" --donor-urdf "$DONOR" \
  --ur-description "$UR_DESCRIPTION" --output "$ENV_DIR/ur7e.urdf"

step "two-UR5 VAMP plugin"
python3 "$REPO/mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name two_arms --robot-struct UR5 --num-robots 2 \
  --robot-header "$PREFIX/include/vamp/robots/ur5.hh" \
  --mr-planner-core-prefix "$PREFIX" --output-dir "$WORK"
cp "$WORK"/build/lib/libmr_planner_vamp_plugin_ur5_2r.so "$ENV_DIR/"

step "environment JSON"
python3 "$HERE/make_env_json.py" \
  --config "$PKG/config/arms.yaml" \
  --output "$ENV_DIR/two_arms.json" \
  --plugin "$ENV_DIR/libmr_planner_vamp_plugin_ur5_2r.so"

echo -e "\nvamp_env ready. Rebuild the workspace to pick it up:  colcon build --symlink-install"
