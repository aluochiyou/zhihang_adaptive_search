#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/terminal_launcher_common.sh"
source "${ROOT}/yolo_runtime_common.sh"

RAW_MODEL="${1:-${ZHIHANG_YOLO_MODEL}}"
MODEL="$(zh_expand_user_path "${RAW_MODEL}")"
RUNTIME_ENV="$(zh_trim_arg "${2:-${ZHIHANG_YOLO_ENV}}")"
export ZHIHANG_YOLO_ENV="${RUNTIME_ENV}"

require_desktop_tools

COMM="${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_communication.sh"
[[ -x "${COMM}" ]] || COMM="${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_commonication.sh"
[[ -x "${COMM}" ]] || {
  echo "[ERROR] XTDrone multi-vehicle communication script missing under ${ZHIHANG_COMMUNICATION_DIR}" >&2
  exit 1
}
[[ -x "${ZHIHANG_POSE_SCRIPT}" ]] || {
  echo "[ERROR] pose script missing: ${ZHIHANG_POSE_SCRIPT}" >&2
  exit 1
}
[[ -f "${ZHIHANG_MODEL_STATE_SCRIPT}" ]] || {
  echo "[ERROR] model_state.py missing: ${ZHIHANG_MODEL_STATE_SCRIPT}" >&2
  exit 1
}
if [[ "${ZHIHANG_START_QGC}" == "1" && -n "${ZHIHANG_QGC_EXECUTABLE}" ]]; then
  [[ -x "${ZHIHANG_QGC_EXECUTABLE}" ]] || {
    echo "[WARN] QGroundControl missing or not executable: ${ZHIHANG_QGC_EXECUTABLE}" >&2
  }
fi

if ! MODEL="$(zh_resolve_model_path "${RAW_MODEL}")"; then
  echo "[ERROR] YOLO model not found after normalization: ${MODEL}" >&2
  printf '[DETAIL] raw model argument=%q\n' "${RAW_MODEL}" >&2
  echo "[HINT] bash setup_yolo_runtime.sh --create --install --model \"$HOME/yolo_models/best.pt\"" >&2
  exit 1
fi

zh_yolo_runtime_summary
bash "${ROOT}/check_yolo26_env.sh" "${RUNTIME_ENV}"

if [[ -n "${ZHIHANG_PX4_ENV_SCRIPT}" ]]; then
  [[ -f "${ZHIHANG_PX4_ENV_SCRIPT}" ]] || {
    echo "[ERROR] PX4 environment script missing: ${ZHIHANG_PX4_ENV_SCRIPT}" >&2
    exit 1
  }

# Source PX4 environment if configured
if [[ -n "${ZHIHANG_PX4_ENV_SCRIPT:-}" && -f "${ZHIHANG_PX4_ENV_SCRIPT}" ]]; then
  source "${ZHIHANG_PX4_ENV_SCRIPT}"
fi
fi

command -v roslaunch >/dev/null 2>&1 || {
  echo '[ERROR] roslaunch unavailable after loading the configured ROS environment' >&2
  exit 1
}
rospack find "${ZHIHANG_PX4_ROS_PACKAGE}" >/dev/null 2>&1 || {
  echo "[ERROR] ROS package not visible: ${ZHIHANG_PX4_ROS_PACKAGE}" >&2
  echo '[HINT] configure ZHIHANG_OPTIONAL_UNDERLAY or ZHIHANG_PX4_ENV_SCRIPT.' >&2
  exit 1
}

[[ -f "${ZHIHANG_FORMAL_CONFIG}" ]] || {
  echo "[ERROR] formal config missing: ${ZHIHANG_FORMAL_CONFIG}" >&2
  echo "[HINT] run: bash install_portable.sh" >&2
  exit 1
}

VERIFY_ARGS=(
  --model "${MODEL}"
  --device "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 0 auto)"
  --imgsz 640
  --quantize "${ZHIHANG_YOLO_QUANTIZE}"
  --warmup 1
  --iterations 2
  --minimum-fps 0.1
  --report "${ROOT}/preflight_yolo_model_report.json"
)
[[ "${ZHIHANG_YOLO_REQUIRE_CUDA}" == "0" ]] && VERIFY_ARGS+=(--allow-cpu)
zh_yolo_run "${ROOT}/verify_yolo_runtime.py" "${VERIFY_ARGS[@]}"

echo "[OK] normalized model: ${MODEL}"
echo "[OK] runtime: ${ZHIHANG_YOLO_RUNTIME_RESOLVED:-conda}/${RUNTIME_ENV}"
echo "[OK] communication initializer: ${COMM}"
echo "[OK] terminal backend: $(resolve_terminal_backend)"
echo '[OK] V6.7.19 portable launcher preflight passed'
