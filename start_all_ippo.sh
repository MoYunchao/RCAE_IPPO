#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: bash start_all_ippo.sh [--map default|maze40|real] [checkpoint.pt] [name:=value ...]
Default checkpoint: models/inference/ippo/best_model.pt
Activate a Python environment with torch, source /opt/ros/humble/setup.bash,
then build this copied workspace with bash toBuild.sh before launching.
EOF
  exit 0
fi

MAP_VARIANT=default
if [[ "${1:-}" == "--map" ]]; then
  [[ $# -ge 2 ]] || { echo '--map requires a value' >&2; exit 2; }
  MAP_VARIANT="$2"
  shift 2
fi
case "$MAP_VARIANT" in
  default) LAUNCH_FILE=full_exploration_system_mappo.launch.py ;;
  maze40) LAUNCH_FILE=full_exploration_system_mappo_maze40.launch.py ;;
  real) LAUNCH_FILE=full_exploration_system_mappo_real.launch.py ;;
  *) echo "Unsupported map: $MAP_VARIANT" >&2; exit 2 ;;
esac
MODEL_PATH="$(pwd)/models/inference/ippo/best_model.pt"
if [[ $# -gt 0 && "$1" != -* && "$1" != *":="* ]]; then
  MODEL_PATH="$1"
  shift
fi
[[ -f "$MODEL_PATH" ]] || { echo "IPPO model not found: $MODEL_PATH. Train and copy an IPPO checkpoint first." >&2; exit 1; }

python - "$MODEL_PATH" <<'PY'
import sys
import torch
p = torch.load(sys.argv[1], map_location='cpu', weights_only=True)
if p.get('algorithm') != 'IPPO' or p.get('critic_observation') != 'local':
    raise SystemExit('This entry requires a newly trained IPPO checkpoint, not a historical MAPPO/PACE model.')
if int(p['local_obs_dim']) != 10 or int(p['shared_obs_dim']) != 10:
    raise SystemExit('Expected actor input=10 and local critic input=10.')
if p.get('action_schema_version') != 'fuzzy_weights_v1':
    raise SystemExit('Expected four independently bounded weights, without simplex normalization.')
print('IPPO checkpoint verified: actor=10, critic=10, action=4 independent Beta weights')
PY

set +u
source install/setup.bash
set -u
RESULT_BASE="${RESULT_ROOT:-$(pwd)/output/ippo_runs}"
RUN_DIR="$RESULT_BASE/map_${MAP_VARIANT}/ippo_fuzzy/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR"
python - "$MODEL_PATH" "$RUN_DIR/checkpoint_provenance.json" <<'PY'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]).resolve()
Path(sys.argv[2]).write_text(json.dumps({'algorithm':'IPPO','checkpoint':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()},indent=2)+'\n')
PY
ros2 launch simulation_bringup "$LAUNCH_FILE" \
  mappo_model_path:="$MODEL_PATH" action_mode:=beta \
  infer_save_enabled:=true infer_save_root:="$RUN_DIR" infer_run_name:=ippo_infer \
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv" \
  metrics_summary_path:="$RUN_DIR/summary.json" "$@"
