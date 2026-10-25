#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

RVIZ_CONFIG="$(pwd)/src/simulation_bringup/config/fuzzy_decision.rviz"
rviz2 -d "$RVIZ_CONFIG" "$@"
