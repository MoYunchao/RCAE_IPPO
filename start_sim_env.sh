#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

ros2 launch simulation_bringup simulation_bringup.launch.py "$@"
