#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

DEFAULT_RESULT_ROOT="${RESULT_ROOT:-$(pwd)/output/ablation_runs_20260424_refresh}"
GROUP_NAME="baseline_real_fuzzy"
MAP_DIR="map_real"
RUN_NAME="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$DEFAULT_RESULT_ROOT/$MAP_DIR/$GROUP_NAME/$RUN_NAME"
mkdir -p "$RUN_DIR"

echo "[start_all_real] result dir: $RUN_DIR"

ros2 launch simulation_bringup full_exploration_system_real.launch.py \
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv" \
  metrics_summary_path:="$RUN_DIR/summary.json"
