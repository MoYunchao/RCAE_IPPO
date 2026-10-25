#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

ros2 launch simulation_bringup multi_robots_explore.launch.py \
  robot_count:=3 \
  robot_namespace:=robot_ \
  global_frame:=global_map \
  "$@"
