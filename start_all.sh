#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

MAP_VARIANT="default"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      if [[ $# -lt 2 ]]; then
        echo "[start_all] --map requires a value" >&2
        exit 1
      fi
      MAP_VARIANT="$2"
      shift 2
      ;;
    --help|-h)
      cat <<'EOF'
用法:
  bash start_all.sh [--map default|real|maze40] [additional ros2 launch args...]
EOF
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

DEFAULT_RESULT_ROOT="${RESULT_ROOT:-$(pwd)/output/ablation_runs_20260424_refresh}"
GROUP_NAME="baseline_fuzzy"
MAP_DIR="map_${MAP_VARIANT}"
RUN_NAME="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$DEFAULT_RESULT_ROOT/$MAP_DIR/$GROUP_NAME/$RUN_NAME"
mkdir -p "$RUN_DIR"

echo "[start_all] result dir: $RUN_DIR"

LAUNCH_FILE="full_exploration_system.launch.py"
if [[ "$MAP_VARIANT" == "real" ]]; then
  LAUNCH_FILE="full_exploration_system_real.launch.py"
elif [[ "$MAP_VARIANT" == "maze40" ]]; then
  LAUNCH_FILE="full_exploration_system_maze40.launch.py"
elif [[ "$MAP_VARIANT" != "default" ]]; then
  echo "[start_all] unsupported map: $MAP_VARIANT" >&2
  exit 1
fi

ros2 launch simulation_bringup "$LAUNCH_FILE" \
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv" \
  metrics_summary_path:="$RUN_DIR/summary.json" \
  "$@"
