#!/usr/bin/env bash
# Records the RGB, depth, camera_info, odom, and TF topics needed to later
# build a semantic map (see go2_semantic_nav/scripts/build_semantic_map.py):
# RGB + depth + camera_info give the per-frame image/geometry, and
# /robot1/tf + /robot1/tf_static let the offline map builder look up
# map->camera_face at each frame's own timestamp (no live ROS needed at build
# time -- see that script's own docstring). /robot1/odom is recorded too,
# cheaply, as a fallback/debug signal even though pose is really derived from
# the TF chain.
#
# NOTE: TF in this stack is namespaced (/robot1/tf, /robot1/tf_static), NOT
# the bare /tf, /tf_static a non-namespaced node would expect -- confirmed
# live via `ros2 topic list` against a running stack. This is why
# frontier_explorer.launch.py/coverage_tour.launch.py/semantic_goal.launch.py
# all need their own SetRemap('/tf', 'tf') wrapper (see those launch files'
# docstrings) -- the same reason this script must record the namespaced
# topic, not the bare one.
#
# Usage: ~/go2_nav/src/go2_nav_bringup/scripts/record_bag.sh [output_path]
# Default output path: bags/<timestamp> (repo-root-relative, resolved from
# this script's own location so it works whether run from source or from the
# installed lib/ copy -- same readlink -f pattern as run_stack.sh's venv
# auto-source).
#
# This script only records, it doesn't move the robot -- pair it with something
# that does (manual teleop/goals, or coverage_tour.py, which runs this itself as a
# subprocess around its recorded portion of the tour -- see its docstring). Ctrl+C
# to stop recording if run directly.

set -u

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

OUT="${1:-$REPO_ROOT/bags/$(date +%Y%m%d_%H%M%S)}"

echo "==> Recording to: $OUT"
exec ros2 bag record -s mcap -o "$OUT" --topics \
    /robot1/camera/image \
    /robot1/camera/depth_image \
    /robot1/camera/camera_info \
    /robot1/odom \
    /robot1/tf \
    /robot1/tf_static
