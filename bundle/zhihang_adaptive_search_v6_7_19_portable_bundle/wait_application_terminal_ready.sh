#!/usr/bin/env bash
set -euo pipefail
MODE="${1:?Usage: bash wait_application_terminal_ready.sh manager|vehicle ID_OR_DASH PORT_OR_DASH TITLE TIMEOUT}"
ID="${2:--}"
PORT="${3:--}"
TITLE="${4:?missing terminal title}"
TIMEOUT="${5:-120}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/terminal_launcher_common.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
RCFILE="$(terminal_rc_file "${TITLE}")"
DEADLINE=$((SECONDS + TIMEOUT))
while ((SECONDS < DEADLINE)); do
  if [[ -s "${RCFILE}" ]]; then
    rc="$(tr -d '[:space:]' < "${RCFILE}")"
    echo "[ERROR] ${TITLE} exited before becoming ready; rc=${rc}" >&2
    [[ "${rc}" == 124 ]] && echo '[CAUSE] a timeout command terminated the child process' >&2
    exit 1
  fi
  nodes="$(rosnode list 2>/dev/null || true)"
  if [[ "${MODE}" == manager ]]; then
    if grep -qx '/mission_manager_v6' <<<"${nodes}"; then
      echo '[OK] V6 management terminal node is running'
      exit 0
    fi
  elif [[ "${MODE}" == vehicle ]]; then
    flight="/vehicle_flight_agent_v${ID}"
    perception="/vehicle_perception_agent_v${ID}"
    port_ready=0
    ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${PORT}$" && port_ready=1 || true
    if grep -qx "${flight}" <<<"${nodes}" && grep -qx "${perception}" <<<"${nodes}" && ((port_ready)); then
      echo "[OK] vehicle ${ID} terminal ready: YOLO port ${PORT}, flight agent and perception agent"
      exit 0
    fi
  else
    echo "[ERROR] unknown mode: ${MODE}" >&2
    exit 2
  fi
  sleep 1
done
echo "[ERROR] ${TITLE} did not become ready within ${TIMEOUT}s" >&2
[[ -f "${RCFILE}" ]] && echo "[INFO] terminal rc=$(cat "${RCFILE}")" >&2
exit 1
