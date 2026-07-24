#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"

if [[ $# -lt 1 ]]; then
  echo "Usage: bash run_vehicle_terminal.sh VEHICLE_ID [RUNTIME_ENV] [MODEL] [YOLO_PORT] [YOLO_HOST]" >&2
  exit 2
fi

VEHICLE_ID="$(zh_trim_arg "$1")"
RUNTIME_ENV="$(zh_trim_arg "${2:-${ZHIHANG_YOLO_ENV}}")"
MODEL_ARG="$(zh_expand_user_path "${3:-${ZHIHANG_YOLO_MODEL}}")"
DEFAULT_PORT=$((ZHIHANG_YOLO_PORT_BASE + VEHICLE_ID))
YOLO_PORT="$(zh_trim_arg "${4:-${DEFAULT_PORT}}")"
YOLO_HOST="$(zh_trim_arg "${5:-$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" "${VEHICLE_ID}" 127.0.0.1)}")"

case "${VEHICLE_ID}" in
  0|1|2) ;;
  *) echo "[ERROR] VEHICLE_ID must be 0, 1 or 2" >&2; exit 2 ;;
esac

export ZHIHANG_YOLO_ENV="${RUNTIME_ENV}"
zh_resolve_yolo_runtime >/dev/null

PKG_DIR="${ZHIHANG_PKG_DIR}"
WORKER="${PKG_DIR}/scripts/yolo26_single_worker.py"
if ! MODEL="$(zh_resolve_model_path "${MODEL_ARG}")"; then
  echo "[ERROR] model not found after normalization: ${MODEL_ARG}" >&2
  echo "[HINT] configure ZHIHANG_YOLO_MODEL or run setup_yolo_runtime.sh" >&2
  exit 1
fi
[[ -f "${WORKER}" ]] || {
  echo "[ERROR] YOLO worker missing: ${WORKER}" >&2
  exit 1
}

DEVICE="$(zh_resolve_device_for_vehicle "${VEHICLE_ID}")"
QUANTIZE="${ZHIHANG_YOLO_QUANTIZE}"
ADAPTIVE_ARGS=()
[[ "${ZHIHANG_YOLO_ADAPTIVE}" == "1" ]] && ADAPTIVE_ARGS+=(--adaptive)

LOG_ROOT="${HOME}/zhihang_vehicle_runs_v6/terminal_logs"
mkdir -p "${LOG_ROOT}"
STAMP="$(date +%Y%m%d_%H%M%S)"
YOLO_LOG="${LOG_ROOT}/vehicle_${VEHICLE_ID}_yolo_${STAMP}.log"

cleanup() {
  rc=$?
  trap - EXIT INT TERM
  if [[ -n "${WORKER_PID:-}" ]] && kill -0 "${WORKER_PID}" 2>/dev/null; then
    echo "[V${VEHICLE_ID}] stopping local YOLO worker PID=${WORKER_PID}"
    kill -TERM "${WORKER_PID}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${WORKER_PID}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "${WORKER_PID}" 2>/dev/null || true
  fi
  exit "${rc}"
}
trap cleanup EXIT INT TERM

is_local_host() {
  case "$1" in
    127.0.0.1|localhost|0.0.0.0|::1) return 0 ;;
    *) return 1 ;;
  esac
}

wait_tcp_port() {
  local host="$1" port="$2" timeout="${3:-90}"
  python3 - "${host}" "${port}" "${timeout}" <<'PY'
import socket, sys, time
host, port, timeout = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
deadline = time.monotonic() + timeout
while time.monotonic() < deadline:
    try:
        with socket.create_connection((host, port), timeout=1.0):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.5)
raise SystemExit(1)
PY
}

if [[ "${ZHIHANG_YOLO_START_LOCAL_WORKERS}" == "1" ]] && is_local_host "${YOLO_HOST}"; then
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${YOLO_PORT}$"; then
    echo "[ERROR] YOLO port already listening: ${YOLO_PORT}" >&2
    echo "[HINT] inspect with: ss -ltnp | grep :${YOLO_PORT}" >&2
    exit 1
  fi

  echo "[V${VEHICLE_ID}] starting portable YOLO worker: ${ZHIHANG_YOLO_BIND_HOST}:${YOLO_PORT}"
  echo "[V${VEHICLE_ID}] runtime=${ZHIHANG_YOLO_RUNTIME_RESOLVED} env=${ZHIHANG_YOLO_ENV}"
  echo "[V${VEHICLE_ID}] model=${MODEL} device=${DEVICE} log=${YOLO_LOG}"
  zh_yolo_run "${WORKER}" \
    --vehicle-id "${VEHICLE_ID}" \
    --host "${ZHIHANG_YOLO_BIND_HOST}" \
    --port "${YOLO_PORT}" \
    --model "${MODEL}" \
    --device "${DEVICE}" \
    --conf "${ZHIHANG_YOLO_CONFIDENCE}" \
    --iou "${ZHIHANG_YOLO_IOU}" \
    --quantize "${QUANTIZE}" \
    "${ADAPTIVE_ARGS[@]}" \
    --adaptive-sizes "${ZHIHANG_YOLO_ADAPTIVE_SIZES}" \
    --minimum-fps "${ZHIHANG_YOLO_MINIMUM_FPS}" \
    --performance-window "${ZHIHANG_YOLO_PERFORMANCE_WINDOW}" \
    --adaptive-check-interval "${ZHIHANG_YOLO_ADAPTIVE_CHECK_INTERVAL}" \
    --warmup-iterations "${ZHIHANG_YOLO_WARMUP_ITERATIONS}" \
    --socket-timeout "${ZHIHANG_YOLO_SOCKET_TIMEOUT}" \
    > >(tee -a "${YOLO_LOG}") 2>&1 &
  WORKER_PID=$!

  for _ in {1..180}; do
    if ! kill -0 "${WORKER_PID}" 2>/dev/null; then
      echo "[ERROR] vehicle ${VEHICLE_ID} YOLO worker exited; inspect ${YOLO_LOG}" >&2
      wait "${WORKER_PID}" || true
      exit 1
    fi
    if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${YOLO_PORT}$"; then
      break
    fi
    sleep 0.5
  done
  ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${YOLO_PORT}$" || {
    echo "[ERROR] local YOLO port ${YOLO_PORT} did not open" >&2
    exit 1
  }
else
  echo "[V${VEHICLE_ID}] using external YOLO endpoint ${YOLO_HOST}:${YOLO_PORT}"
  wait_tcp_port "${YOLO_HOST}" "${YOLO_PORT}" 90 || {
    echo "[ERROR] external YOLO endpoint unavailable: ${YOLO_HOST}:${YOLO_PORT}" >&2
    exit 1
  }
fi

echo "[OK] v${VEHICLE_ID} YOLO endpoint ready: ${YOLO_HOST}:${YOLO_PORT}"
echo "[V${VEHICLE_ID}] starting independent flight + perception agents"
echo "[V${VEHICLE_ID}] READY means waiting for all three real >=10 Hz pipelines and manager start barrier"

CONFIG_ARGS=()
if [[ -n "${V6_FORMAL_CONFIG:-}" ]]; then
  CONFIG_ARGS+=(config:="${V6_FORMAL_CONFIG}")
fi
roslaunch zhihang_adaptive_search_v6 vehicle_terminal.launch \
  "${CONFIG_ARGS[@]}" \
  vehicle_id:="${VEHICLE_ID}" \
  yolo_host:="${YOLO_HOST}" \
  yolo_port:="${YOLO_PORT}"
