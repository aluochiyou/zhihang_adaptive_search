#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

echo '[01] PX4 + Gazebo + three VTOL simulation'
echo "[ENV] profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"

if [[ -n "${ZHIHANG_PX4_ENV_SCRIPT}" ]]; then
  [[ -f "${ZHIHANG_PX4_ENV_SCRIPT}" ]] || {
    echo "[ERROR] PX4 environment script missing: ${ZHIHANG_PX4_ENV_SCRIPT}" >&2
    exit 1
  }
  # shellcheck disable=SC1090
  source "${ZHIHANG_PX4_ENV_SCRIPT}"
fi

command -v roslaunch >/dev/null 2>&1 || {
  echo '[ERROR] roslaunch is unavailable in terminal 01.' >&2
  echo '[HINT] configure ZHIHANG_PX4_ENV_SCRIPT or fix the normal shell environment.' >&2
  exit 1
}
PX4_PKG="$(rospack find "${ZHIHANG_PX4_ROS_PACKAGE}" 2>/dev/null || true)"
[[ -n "${PX4_PKG}" ]] || {
  echo "[ERROR] ROS package unavailable: ${ZHIHANG_PX4_ROS_PACKAGE}" >&2
  echo "[DEBUG] ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-<unset>}" >&2
  exit 1
}
echo "[OK] PX4 ROS package: ${PX4_PKG}"

REQUESTED_LAUNCH="${ZHIHANG_PX4_LAUNCH_FILE}"
if [[ -z "${REQUESTED_LAUNCH}" || "${REQUESTED_LAUNCH}" == "auto" ]]; then
  REQUESTED_LAUNCH=""
  for candidate in zhihang2026.launch zhihang20267.launch; do
    if [[ -f "${PX4_PKG}/launch/${candidate}" ]]; then
      REQUESTED_LAUNCH="${candidate}"
      break
    fi
  done
  if [[ -z "${REQUESTED_LAUNCH}" ]]; then
    echo "[ERROR] no competition launch file found under ${PX4_PKG}/launch" >&2
    echo '[HINT] expected official zhihang2026.launch or legacy zhihang20267.launch.' >&2
    exit 1
  fi
fi
if [[ ! -f "${PX4_PKG}/launch/${REQUESTED_LAUNCH}" ]]; then
  echo "[ERROR] PX4 launch file not found: ${PX4_PKG}/launch/${REQUESTED_LAUNCH}" >&2
  exit 1
fi
echo "[01] roslaunch ${ZHIHANG_PX4_ROS_PACKAGE} ${REQUESTED_LAUNCH} ${ZHIHANG_PX4_LAUNCH_ARGS}"

LAUNCH_ARGS=()
if [[ -n "${ZHIHANG_PX4_LAUNCH_ARGS}" ]]; then
  # Profile is a trusted local file; split launch arguments on shell whitespace.
  read -r -a LAUNCH_ARGS <<<"${ZHIHANG_PX4_LAUNCH_ARGS}"
fi
exec roslaunch "${ZHIHANG_PX4_ROS_PACKAGE}" "${REQUESTED_LAUNCH}" "${LAUNCH_ARGS[@]}"
