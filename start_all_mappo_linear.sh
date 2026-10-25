#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法:
  bash start_all_mappo_linear.sh [--map default|maze40|real] [mappo_model_path] [additional ros2 launch args...]

示例:
  bash start_all_mappo_linear.sh --map maze40 \
    robot0_battery_capacity_ah:=0.67 \
    robot1_battery_capacity_ah:=0.71 \
    robot2_battery_capacity_ah:=0.75

默认模型:
  models/inference/mappo/mappo_v3_simplex_battery_random_30k/models/best_model.pt

说明:
  - 保留 RL 动态权重
  - 去掉 fuzzy rule，改为直接线性加权打分
EOF
  exit 0
fi

set +u
source install/setup.bash
set -u

MAP_VARIANT="default"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      if [[ $# -lt 2 ]]; then
        echo "[start_all_mappo_linear] --map requires a value" >&2
        exit 1
      fi
      MAP_VARIANT="$2"
      shift 2
      ;;
    *)
      break
      ;;
  esac
done

DEFAULT_MODEL="$(pwd)/models/inference/mappo/mappo_v3_simplex_battery_random_30k/models/best_model.pt"
DEFAULT_RESULT_ROOT="${RESULT_ROOT:-$(pwd)/output/ablation_runs_20260424_refresh}"
GROUP_NAME="mappo_linear"
MAP_DIR="map_${MAP_VARIANT}"
RUN_NAME="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$DEFAULT_RESULT_ROOT/$MAP_DIR/$GROUP_NAME/$RUN_NAME"
MODEL_PATH="$DEFAULT_MODEL"
if [ $# -gt 0 ] && [[ "$1" != -* ]] && [[ "$1" != *":="* ]]; then
  MODEL_PATH="$1"
  shift
fi

if [ ! -f "$MODEL_PATH" ]; then
  echo "[start_all_mappo_linear] model not found: $MODEL_PATH" >&2
  echo "请显式传入模型路径，例如:" >&2
  echo "  bash start_all_mappo_linear.sh /abs/path/to/best_model.pt" >&2
  exit 1
fi

mkdir -p "$RUN_DIR"
echo "[start_all_mappo_linear] result dir: $RUN_DIR"

LAUNCH_FILE="full_exploration_system_mappo_linear.launch.py"
if [[ "$MAP_VARIANT" == "real" ]]; then
  LAUNCH_FILE="full_exploration_system_mappo_linear_real.launch.py"
elif [[ "$MAP_VARIANT" == "maze40" ]]; then
  LAUNCH_FILE="full_exploration_system_mappo_linear_maze40.launch.py"
elif [[ "$MAP_VARIANT" != "default" ]]; then
  echo "[start_all_mappo_linear] unsupported map: $MAP_VARIANT" >&2
  exit 1
fi

ros2 launch simulation_bringup "$LAUNCH_FILE" \
  mappo_model_path:="$MODEL_PATH" \
  action_mode:=auto \
  infer_save_enabled:=true \
  infer_save_root:="$RUN_DIR" \
  infer_run_name:=rl_infer \
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv" \
  metrics_summary_path:="$RUN_DIR/summary.json" \
  "$@"
