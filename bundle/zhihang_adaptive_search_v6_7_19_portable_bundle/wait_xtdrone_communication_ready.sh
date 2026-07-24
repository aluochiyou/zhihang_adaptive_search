#!/usr/bin/env bash
set -euo pipefail
TIMEOUT="${1:-45}"
TITLE='[03] XTDrone-Comm'
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/terminal_launcher_common.sh"
RCFILE="$(terminal_rc_file "${TITLE}")"
DEADLINE=$((SECONDS + TIMEOUT))

while (( SECONDS < DEADLINE )); do
  if [[ -s "${RCFILE}" ]]; then
    rc="$(tr -d '[:space:]' < "${RCFILE}")"
    echo "[ERROR] ${TITLE} exited before communication became ready; rc=${rc}" >&2
    exit 1
  fi
  mapfile -t pids < <(pgrep -f '[v]tol_communication\.py' || true)
  if ((${#pids[@]} >= 3)); then
    echo "[OK] original XTDrone communication ready: ${#pids[@]} resident vtol_communication.py processes"
    exit 0
  fi
  echo "[WAIT] XTDrone communication processes=${#pids[@]}/3"
  sleep 1
done

echo "[ERROR] fewer than three vtol_communication.py processes after ${TIMEOUT}s" >&2
exit 1
