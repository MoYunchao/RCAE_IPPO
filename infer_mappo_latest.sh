#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法:
  bash infer_mappo_latest.sh [model_path] [additional rl_infer_mappo args...]

默认模型:
  models/inference/mappo/mappo_v3_simplex_battery_random_30k/models/best_model.pt

默认发布:
  /robot_0/rl_adjust_weights
  /robot_1/rl_adjust_weights
  /robot_2/rl_adjust_weights
EOF
  exit 0
fi

DEFAULT_MODEL="$(pwd)/models/inference/mappo/mappo_v3_simplex_battery_random_30k/models/best_model.pt"
MODEL_PATH="$DEFAULT_MODEL"
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then
  MODEL_PATH="$1"
  shift
fi

if [ ! -f "$MODEL_PATH" ]; then
  echo "[infer_mappo_latest] model not found: $MODEL_PATH" >&2
  echo "请显式传入模型路径，例如:" >&2
  echo "  bash infer_mappo_latest.sh /abs/path/to/best_model.pt --rate 2.0" >&2
  exit 1
fi

set +u
source install/setup.bash
set -u

ros2 run rl_training rl_infer_mappo \
  --model "$MODEL_PATH" \
  --weight-topic-template '/{robot_name}/rl_adjust_weights' \
  "$@"
