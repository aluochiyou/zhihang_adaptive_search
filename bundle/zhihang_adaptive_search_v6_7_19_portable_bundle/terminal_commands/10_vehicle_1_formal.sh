#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"
MODEL="$(zh_expand_user_path "${1:-${ZHIHANG_YOLO_MODEL}}")"
RUNTIME_ENV="$(zh_trim_arg "${2:-${ZHIHANG_YOLO_ENV}}")"
PORT=$((ZHIHANG_YOLO_PORT_BASE + 1))
HOST="$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" 1 127.0.0.1)"
echo "[10] Formal Vehicle 1; model=${MODEL}; runtime=${ZHIHANG_YOLO_RUNTIME}/${RUNTIME_ENV}; endpoint=${HOST}:${PORT}"
cd "${ROOT}"
bash preflight_vehicle.sh 1 "${PORT}" "${HOST}"
sleep 2
exec bash run_vehicle_terminal_formal.sh 1 "${RUNTIME_ENV}" "${MODEL}" "${PORT}" "${HOST}"
