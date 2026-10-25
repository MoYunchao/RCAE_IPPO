#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

ros2 launch simulation_bringup robot_1_fuzzy_decision.launch.py \
  weight_topic:=/robot_1/rl_adjust_weights \
  "$@"
