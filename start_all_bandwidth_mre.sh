#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法:
  bash start_all_bandwidth_mre.sh [--map default|maze40|real] [model_path] [additional ros2 launch args...]

默认模型:
  models/inference/bandwidth_mre/sync_gain_topk_h1_80000/models/best_model.pt

说明:
  - 该脚本启动对比试验 3 的 Bandwidth-MRE 推理侧完整链路。
  - 推理节点直接基于 global_map、frontiers_info、odom 构建局部图和中间候选点，
    输出 /robot_i/task_point，不经过 MAPPO 权重节点和 fuzzy goal selector。
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
        echo "[start_all_bandwidth_mre] --map requires a value" >&2
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

DEFAULT_MODEL="$(pwd)/models/inference/bandwidth_mre/sync_gain_topk_h1_80000/models/best_model.pt"
DEFAULT_RESULT_ROOT="${RESULT_ROOT:-$(pwd)/output/comparison_runs_method03}"
GROUP_NAME="bandwidth_mre"
MAP_DIR="map_${MAP_VARIANT}"
RUN_NAME="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$DEFAULT_RESULT_ROOT/$MAP_DIR/$GROUP_NAME/$RUN_NAME"
MODEL_PATH="$DEFAULT_MODEL"

if [[ $# -gt 0 && "$1" != -* && "$1" != *":="* ]]; then
  MODEL_PATH="$1"
  shift
fi

if [[ ! -f "$MODEL_PATH" ]]; then
  echo "[start_all_bandwidth_mre] model not found: $MODEL_PATH" >&2
  echo "请显式传入模型路径，例如:" >&2
  echo "  bash start_all_bandwidth_mre.sh /abs/path/to/best_model.pt" >&2
  exit 1
fi

LAUNCH_FILE="full_exploration_system_bandwidth_mre.launch.py"
if [[ "$MAP_VARIANT" == "real" ]]; then
  LAUNCH_FILE="full_exploration_system_bandwidth_mre_real.launch.py"
elif [[ "$MAP_VARIANT" == "maze40" ]]; then
  LAUNCH_FILE="full_exploration_system_bandwidth_mre_maze40.launch.py"
elif [[ "$MAP_VARIANT" != "default" ]]; then
  echo "[start_all_bandwidth_mre] unsupported map: $MAP_VARIANT" >&2
  exit 1
fi

mkdir -p "$RUN_DIR"
echo "[start_all_bandwidth_mre] result dir: $RUN_DIR"
echo "[start_all_bandwidth_mre] model: $MODEL_PATH"
echo "[start_all_bandwidth_mre] launch: $LAUNCH_FILE"

ros2 launch simulation_bringup "$LAUNCH_FILE" \
  model_path:="$MODEL_PATH" \
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv" \
  metrics_summary_path:="$RUN_DIR/summary.json" \
  infer_save_enabled:=true \
  infer_save_root:="$RUN_DIR" \
  infer_run_name:=bandwidth_mre_infer \
  "$@"
