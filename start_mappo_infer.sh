#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
set +u
source install/setup.bash
set -u

DEFAULT_MODEL="$(pwd)/models/inference/mappo/mappo_v3_simplex_battery_random_30k/models/best_model.pt"
DEFAULT_OUTPUT_ROOT="$(pwd)/output/rl_runs"
MODEL_PATH="$DEFAULT_MODEL"

if [ $# -gt 0 ] && [[ "$1" != -* ]] && [[ "$1" != *":="* ]]; then
  MODEL_PATH="$1"
  shift
fi

if [ ! -f "$MODEL_PATH" ]; then
  echo "[start_mappo_infer] model not found: $MODEL_PATH" >&2
  exit 1
fi

ros2 launch rl_training rl_training.launch.py \
  robot_count:=3 \
  robot_namespace:=robot_ \
  enable_weight_node:=true \
  policy_mode:=mappo \
  action_mode:=auto \
  model_path:="$MODEL_PATH" \
  infer_save_enabled:=true \
  infer_save_root:="$DEFAULT_OUTPUT_ROOT" \
  weight_topic_template:=/{robot_name}/rl_adjust_weights \
  weight_update_rate:=2.0 \
  weight_log_interval:=2.0 \
  "$@"
