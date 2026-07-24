#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"
MODEL="$(zh_expand_user_path "${1:-${ZHIHANG_YOLO_MODEL}}")"
RUNTIME_ENV="$(zh_trim_arg "${2:-${ZHIHANG_YOLO_ENV}}")"
PORT=$((ZHIHANG_YOLO_PORT_BASE + 2))
HOST="$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" 2 127.0.0.1)"
echo "[11] Vehicle 2; model=${MODEL}; runtime=${ZHIHANG_YOLO_RUNTIME}/${RUNTIME_ENV}; endpoint=${HOST}:${PORT}"
cd "${ROOT}"
bash preflight_vehicle.sh 2 "${PORT}" "${HOST}"
sleep 3
exec bash run_vehicle_terminal.sh 2 "${RUNTIME_ENV}" "${MODEL}" "${PORT}" "${HOST}"
