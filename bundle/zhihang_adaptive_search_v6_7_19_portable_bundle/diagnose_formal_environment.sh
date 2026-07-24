#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"
RAW_MODEL="${1:-${ZHIHANG_YOLO_MODEL}}"
RUNTIME_ENV="$(zh_trim_arg "${2:-${ZHIHANG_YOLO_ENV}}")"
export ZHIHANG_YOLO_ENV="${RUNTIME_ENV}"

printf '\n===== MACHINE PROFILE =====\n'
echo "profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"
echo "workspace=${ZHIHANG_WS}"
echo "xtdrone=${ZHIHANG_XTDRONE_ROOT}"
echo "terminal=${ZHIHANG_TERMINAL_BACKEND}"
echo "runtime=${ZHIHANG_YOLO_RUNTIME}/${RUNTIME_ENV}"

printf '\n===== ROS OVERLAY =====\n'
echo "ZHIHANG_PKG_DIR=${ZHIHANG_PKG_DIR}"
echo "ZHIHANG_FORMAL_CONFIG=${ZHIHANG_FORMAL_CONFIG}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

printf '\n===== FILES =====\n'
for path in \
  "${ZHIHANG_PKG_DIR}/package.xml" \
  "${ZHIHANG_VALIDATION_CONFIG}" \
  "${ZHIHANG_FORMAL_CONFIG}" \
  "${ZHIHANG_PKG_DIR}/scripts/yolo26_single_worker.py" \
  "${ZHIHANG_POSE_SCRIPT}" \
  "${ZHIHANG_MODEL_STATE_SCRIPT}"; do
  [[ -f "${path}" ]] && echo "[OK] ${path}" || echo "[WARN] missing ${path}"
done

printf '\n===== MODEL / RUNTIME =====\n'
if MODEL="$(zh_resolve_model_path "${RAW_MODEL}")"; then
  echo "[OK] model=${MODEL}"
else
  echo "[ERROR] model not found after normalization: ${MODEL}" >&2
  exit 1
fi
zh_yolo_runtime_summary
bash "${ROOT}/check_yolo26_env.sh" "${RUNTIME_ENV}"

printf '\n===== ROS MASTER / TOPICS =====\n'
TMP="/tmp/zhihang_v6719_topics.$$"
if rostopic list >"${TMP}" 2>/dev/null; then
  echo '[OK] ROS master reachable'
  for id in 0 1 2; do
    for topic in "/standard_vtol_${id}/mavros/state" "/standard_vtol_${id}/camera/image_raw"; do
      grep -qx "${topic}" "${TMP}" && echo "[OK] ${topic}" || echo "[WARN] missing ${topic}"
    done
  done
else
  echo '[WARN] ROS master is not reachable; start the base environment first.'
fi
rm -f "${TMP}"
echo
echo 'V6.7.19 PORTABLE FORMAL ENVIRONMENT DIAGNOSIS COMPLETE'
