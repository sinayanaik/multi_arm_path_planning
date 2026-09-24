#!/usr/bin/env bash
# Build everything generated from the MuJoCo model: the two URDFs, the SRDF, the UR5e
# collision kernel, the two-arm VAMP plugin, and the environment JSON.
#
# The point of doing it all from one script is that models/ur5e.xml is the only description
# of the robot anywhere in this workspace. RViz draws the URDF this makes; VAMP checks the
# header cricket compiles from the spherized URDF this makes. They cannot be different
# robots, which is what went wrong with the ur_description setup this replaced.
#
# Needs cricket's fkcc_gen. It is not packaged anywhere -- build it once:
#   sudo apt install libcgal-dev cppad libeigen3-dev nlohmann-json3-dev libfmt-dev \
#                    ros-jazzy-pinocchio
#   git clone --recursive https://github.com/CoMMALab/cricket
#   cd cricket && cmake -GNinja -Bbuild -DCRICKET_BUILD_JIT=OFF . && cmake --build build
# then point CRICKET at the checkout (the default below assumes it sits beside this repo).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORK="$HERE/build"
PKG="$HERE/../src/vamp_mr_arms"
MODELS="$PKG/models"
ENV_DIR="$PKG/vamp_env"
PREFIX="${MR_PLANNER_PREFIX:-/usr/local}"
CRICKET="${CRICKET:-$REPO/cricket}"
FKCC="${FKCC_GEN:-$CRICKET/build/fkcc_gen}"
SETTLE="${SETTLE:-4.0}"

mkdir -p "$WORK" "$ENV_DIR"
WORK="$(cd "$WORK" && pwd)"; ENV_DIR="$(cd "$ENV_DIR" && pwd)"

echo -e "\n== URDFs from the MuJoCo model ==" >&2
python3 "$HERE/mjcf_to_urdf.py" \
  --mjcf "$MODELS/ur5e.xml" \
  --display "$MODELS/ur5e.urdf" \
  --spherized "$MODELS/ur5e_spherized.urdf"

echo -e "\n== SRDF allowed-collision matrix ==" >&2
python3 "$HERE/make_srdf.py" \
  --urdf "$MODELS/ur5e_spherized.urdf" \
  --mjcf "$MODELS/ur5e.xml" \
  --output "$MODELS/ur5e.srdf" \
  --tip tcp

echo -e "\n== the cell, settled, into config/cell.yaml ==" >&2
python3 "$HERE/mjcf_to_scene.py" \
  --scene "$MODELS/bimanual_scene.xml" \
  --settle "$SETTLE" \
  --output "$PKG/config/cell.yaml"

echo -e "\n== the generated model against MuJoCo's own ==" >&2
python3 "$HERE/check_model.py" \
  --mjcf "$MODELS/ur5e.xml" \
  --urdf "$MODELS/ur5e.urdf" \
  --spherized "$MODELS/ur5e_spherized.urdf"

echo -e "\n== is the cell workable at all ==" >&2
python3 "$HERE/check_cell.py" \
  --urdf "$MODELS/ur5e_spherized.urdf" \
  --cell "$PKG/config/cell.yaml"

# Everything above this line describes the robot and the cell, and none of it needs cricket.
# Stopping here leaves RViz able to draw both arms in the real cell; only the planner's
# collision kernel is missing.
if [[ ! -x "$FKCC" ]]; then
  echo >&2
  echo "Models and cell are up to date, but cricket's fkcc_gen is not at $FKCC," >&2
  echo "so the UR5e collision kernel and the VAMP plugin were NOT built." >&2
  echo "Build cricket (see the header of this script) or set FKCC_GEN=/path/to/fkcc_gen," >&2
  echo "then re-run. teach/plan will not work until you do." >&2
  exit 1
fi

echo -e "\n== UR5e collision kernel + two-arm VAMP plugin ==" >&2
python3 "$REPO/mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name two_arms --robot-struct UR5e --num-robots 2 \
  --spherized-urdf "$MODELS/ur5e_spherized.urdf" \
  --srdf "$MODELS/ur5e.srdf" \
  --end-effector tcp --resolution 32 \
  --cricket-bin "$FKCC" \
  --cricket-templates "$CRICKET/resources/templates" \
  --mr-planner-core-prefix "$PREFIX" --output-dir "$WORK"
cp "$WORK"/build/lib/libmr_planner_vamp_plugin_ur5e_2r.so "$ENV_DIR/"
rm -f "$ENV_DIR/libmr_planner_vamp_plugin_ur5_2r.so"

echo -e "\n== environment JSON ==" >&2
python3 "$HERE/make_env_json.py" \
  --cell "$PKG/config/cell.yaml" \
  --output "$ENV_DIR/two_arms.json" \
  --plugin "$ENV_DIR/libmr_planner_vamp_plugin_ur5e_2r.so"

echo -e "\nvamp_env ready. Rebuild the workspace to pick it up:  colcon build --symlink-install"
