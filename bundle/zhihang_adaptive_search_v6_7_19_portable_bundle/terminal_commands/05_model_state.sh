#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

MARKER=/tmp/zhihang_v6_7_model_state_started.json
printf '{"wall_time": %.6f, "launcher_pid": %d, "policy": "after_application_ready"}\n' \
  "$(date +%s.%N)" "$$" > "${MARKER}"
echo '[05] Competition target motion model_state.py'
echo '[05] launched only after manager + 3 flight agents + 3 real YOLO pipelines are ready'
echo "[05] script=${ZHIHANG_MODEL_STATE_SCRIPT}"
[[ -f "${ZHIHANG_MODEL_STATE_SCRIPT}" ]] || {
  echo "[ERROR] model_state.py missing: ${ZHIHANG_MODEL_STATE_SCRIPT}" >&2
  exit 1
}
cd "$(dirname "${ZHIHANG_MODEL_STATE_SCRIPT}")"
exec python3 "$(basename "${ZHIHANG_MODEL_STATE_SCRIPT}")"
