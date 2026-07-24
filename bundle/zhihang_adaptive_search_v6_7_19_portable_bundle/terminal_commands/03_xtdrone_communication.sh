#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

echo '[03] XTDrone original multi-vehicle communication'
echo "[03] directory=${ZHIHANG_COMMUNICATION_DIR}"
cd "${ZHIHANG_COMMUNICATION_DIR}"

COMM="./multi_vehicle_communication.sh"
[[ -x "${COMM}" ]] || COMM="./multi_vehicle_commonication.sh"
[[ -x "${COMM}" ]] || {
  echo '[ERROR] neither multi_vehicle_communication.sh nor multi_vehicle_commonication.sh exists' >&2
  exit 1
}
set +e
"${COMM}"
rc=$?
set -e
if (( rc != 0 )); then
  echo "[ERROR] XTDrone communication initializer failed; rc=${rc}" >&2
  exit "${rc}"
fi

echo '[INFO] initializer returned rc=0; verifying three resident communication processes'
deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  mapfile -t comm_pids < <(pgrep -f '[v]tol_communication\.py' || true)
  if ((${#comm_pids[@]} >= 3)); then
    echo "[OK] three VTOL communication processes are resident: ${comm_pids[*]}"
    break
  fi
  echo "[WAIT] resident vtol_communication.py processes=${#comm_pids[@]}/3"
  sleep 1
done

mapfile -t comm_pids < <(pgrep -f '[v]tol_communication\.py' || true)
if ((${#comm_pids[@]} < 3)); then
  echo '[ERROR] fewer than three vtol_communication.py processes remain' >&2
  exit 1
fi

echo '[SUPERVISE] monitoring the original XTDrone communication processes'
while true; do
  mapfile -t live_pids < <(pgrep -f '[v]tol_communication\.py' || true)
  if ((${#live_pids[@]} < 3)); then
    echo "[ERROR] communication process count dropped to ${#live_pids[@]}/3" >&2
    exit 1
  fi
  sleep 2
done
