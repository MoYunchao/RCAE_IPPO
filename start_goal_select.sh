#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

ros2 run goal_select multi_robot_goal_selector_node --ros-args -p position_tolerance:=1.0 "$@"
