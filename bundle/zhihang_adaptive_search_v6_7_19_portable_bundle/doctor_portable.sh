#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROFILE=""
LIVE=0
RUNTIME_CHECK=1
while (($#)); do
  case "$1" in
    --profile) PROFILE="${2:?missing value}"; shift ;;
    --live) LIVE=1 ;;
    --skip-runtime) RUNTIME_CHECK=0 ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "${PROFILE}" ]] && export ZHIHANG_PROFILE_FILE="${PROFILE}"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/shell_arg_utils.sh"

REPORT="${ROOT}/PORTABLE_DOCTOR_RESULT.txt"
: > "${REPORT}"
log() { echo "$*" | tee -a "${REPORT}"; }
fail=0
check_file() {
  local label="$1" path="$2"
  if [[ -f "${path}" ]]; then log "[PASS] ${label}: ${path}"
  else log "[FAIL] ${label}: ${path}"; fail=1; fi
}
check_dir() {
  local label="$1" path="$2"
  if [[ -d "${path}" ]]; then log "[PASS] ${label}: ${path}"
  else log "[FAIL] ${label}: ${path}"; fail=1; fi
}
check_exec() {
  local label="$1" path="$2"
  if [[ -x "${path}" ]]; then log "[PASS] ${label}: ${path}"
  else log "[FAIL] ${label}: ${path}"; fail=1; fi
}

log "=== Zhihang V6.7.19 Portable Doctor ==="
log "[INFO] profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"
log "[INFO] os=$(lsb_release -ds 2>/dev/null || uname -a)"
log "[INFO] terminal_backend=${ZHIHANG_TERMINAL_BACKEND} DISPLAY=${DISPLAY:-<empty>}"
check_file "ROS setup" "${ZHIHANG_ROS_SETUP}"
check_file "competition message workspace setup" "${ZHIHANG_MESSAGE_WS_SETUP}"
check_dir "competition workspace" "${ZHIHANG_WS}"
check_dir "workspace src" "${ZHIHANG_WS}/src"
check_dir "XTDrone root" "${ZHIHANG_XTDRONE_ROOT}"
check_dir "XTDrone communication" "${ZHIHANG_COMMUNICATION_DIR}"
check_exec "pose script" "${ZHIHANG_POSE_SCRIPT}"
check_file "model_state.py" "${ZHIHANG_MODEL_STATE_SCRIPT}"
check_file "YOLO model" "$(zh_expand_user_path "${ZHIHANG_YOLO_MODEL}")"

COMM="${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_communication.sh"
[[ -x "${COMM}" ]] || COMM="${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_commonication.sh"
check_exec "XTDrone communication initializer" "${COMM}"

if [[ "${ZHIHANG_START_QGC}" == "1" ]]; then
  check_exec "QGroundControl" "${ZHIHANG_QGC_EXECUTABLE}"
else
  log '[SKIP] QGroundControl disabled by profile'
fi

if ((RUNTIME_CHECK)); then
  if bash "${ROOT}/check_yolo26_env.sh" "${ZHIHANG_YOLO_ENV}" 2>&1 | tee -a "${REPORT}"; then
    log '[PASS] YOLO imports/runtime'
  else
    log '[FAIL] YOLO imports/runtime'
    fail=1
  fi
  source "${ROOT}/yolo_runtime_common.sh"
  if zh_resolve_yolo_runtime >/dev/null 2>&1; then
    VERIFY_ARGS=(
      --model "${ZHIHANG_YOLO_MODEL}"
      --device "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 0 auto)"
      --quantize "${ZHIHANG_YOLO_QUANTIZE}"
      --warmup 1
      --iterations 3
      --minimum-fps 0.1
      --report "${ROOT}/PORTABLE_DOCTOR_YOLO.json"
    )
    [[ "${ZHIHANG_YOLO_REQUIRE_CUDA}" == "0" ]] && VERIFY_ARGS+=(--allow-cpu)
    if zh_yolo_run "${ROOT}/verify_yolo_runtime.py" "${VERIFY_ARGS[@]}" 2>&1 | tee -a "${REPORT}"; then
      log '[PASS] real model load/inference'
    else
      log '[FAIL] real model load/inference'
      fail=1
    fi
  fi
fi

READY_ARGS=()
((LIVE)) && READY_ARGS+=(--live)
if bash "${ROOT}/competition_readiness_check.sh" "${READY_ARGS[@]}" 2>&1 | tee -a "${REPORT}"; then
  log '[PASS] competition configuration'
else
  log '[FAIL] competition configuration'
  fail=1
fi

for port in $((ZHIHANG_YOLO_PORT_BASE)) $((ZHIHANG_YOLO_PORT_BASE+1)) $((ZHIHANG_YOLO_PORT_BASE+2)); do
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}$"; then
    log "[WARN] port already in use: ${port}"
  else
    log "[PASS] port available: ${port}"
  fi
done

if ((fail)); then
  log "[ERROR] portable doctor found failures; report=${REPORT}"
  exit 2
fi
log "[OK] portable doctor passed; report=${REPORT}"
