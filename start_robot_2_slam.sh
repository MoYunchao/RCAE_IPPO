#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

ros2 launch simulation_bringup robot_2_slam.launch.py use_sim_time:=true robot_name:=robot_2 "$@"
