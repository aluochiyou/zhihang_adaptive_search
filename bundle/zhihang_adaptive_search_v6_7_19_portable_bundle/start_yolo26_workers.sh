#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"

[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Never use sudo.' >&2
  exit 2
}

export ZHIHANG_YOLO_ENV="$(zh_trim_arg "${1:-${ZHIHANG_YOLO_ENV}}")"
MODEL_ARG="$(zh_expand_user_path "${2:-${ZHIHANG_YOLO_MODEL}}")"
if ! MODEL="$(zh_resolve_model_path "${MODEL_ARG}")"; then
  echo "[ERROR] model missing after normalization: ${MODEL_ARG}" >&2
  exit 2
fi
zh_resolve_yolo_runtime >/dev/null

PKG="$(rospack find zhihang_adaptive_search_v6)"
WORKER="${PKG}/scripts/yolo26_single_worker.py"
PID_DIR="${HOME}/.cache/zhihang_adaptive_search_v6"
LOG_ROOT="${HOME}/zhihang_yolo26_logs_v6/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${PID_DIR}" "${LOG_ROOT}"
ln -sfn "${LOG_ROOT}" "${HOME}/zhihang_yolo26_logs_v6/latest"

for id in 0 1 2; do
  port=$((ZHIHANG_YOLO_PORT_BASE + id))
  device="$(zh_resolve_device_for_vehicle "${id}")"
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}$"; then
    echo "[ERROR] port ${port} already listening" >&2
    exit 2
  fi
  log="${LOG_ROOT}/vehicle_${id}.log"
  echo "[YOLO] v${id} -> ${ZHIHANG_YOLO_BIND_HOST}:${port} device=${device} log=${log}"
  nohup bash -lc \
    "$(printf '%q ' "${ROOT}/run_detached_yolo_worker.sh" "${id}" "${MODEL}" "${port}" "${device}")" \
    >"${log}" 2>&1 < /dev/null &
  echo $! > "${PID_DIR}/yolo_v${id}.pid"
done

for port in $((ZHIHANG_YOLO_PORT_BASE)) $((ZHIHANG_YOLO_PORT_BASE+1)) $((ZHIHANG_YOLO_PORT_BASE+2)); do
  ok=0
  for _ in {1..180}; do
    ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}$" && { ok=1; break; }
    sleep 0.5
  done
  [[ ${ok} -eq 1 ]] || {
    echo "[ERROR] port ${port} did not open; inspect ${LOG_ROOT}" >&2
    "${ROOT}/stop_yolo26_workers.sh" || true
    exit 1
  }
done
echo "[OK] three detached YOLO workers listening from port ${ZHIHANG_YOLO_PORT_BASE}"
echo "[OK] logs: ${LOG_ROOT}"
