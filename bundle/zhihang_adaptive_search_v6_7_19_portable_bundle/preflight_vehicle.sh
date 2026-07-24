#!/usr/bin/env bash
set -euo pipefail
ID="${1:?Usage: bash preflight_vehicle.sh VEHICLE_ID [YOLO_PORT] [YOLO_HOST]}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/yolo_runtime_common.sh"

PORT="${2:-$((ZHIHANG_YOLO_PORT_BASE + ID))}"
HOST="${3:-$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" "${ID}" 127.0.0.1)}"
case "${ID}" in 0|1|2) ;; *) echo '[ERROR] ID must be 0/1/2' >&2; exit 2;; esac

rostopic list >/dev/null
rospack find zhihang_adaptive_search_v6 >/dev/null
for node in "/vehicle_flight_agent_v${ID}" "/vehicle_perception_agent_v${ID}"; do
  if rosnode list 2>/dev/null | grep -qx "${node}"; then
    echo "[ERROR] conflicting node already running: ${node}" >&2
    exit 1
  fi
done

for topic in \
  "/standard_vtol_${ID}/mavros/state" \
  "/standard_vtol_${ID}/mavros/local_position/pose" \
  "/standard_vtol_${ID}/camera/image_raw" \
  "/gazebo/model_states"; do
  rostopic list | grep -qx "${topic}" || {
    echo "[ERROR] topic missing: ${topic}" >&2
    exit 1
  }
done

python3 "${ROOT}/check_topic_rate_once.py" \
  "/standard_vtol_${ID}/camera/image_raw" \
  --minimum-hz 10.0 \
  --sample-seconds 3.0 \
  --topic-timeout-seconds 12.0 \
  --vehicle-id "${ID}"

case "${HOST}" in
  127.0.0.1|localhost|0.0.0.0|::1)
    if [[ "${ZHIHANG_YOLO_START_LOCAL_WORKERS}" == "1" ]] && \
       ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${PORT}$"; then
      echo "[ERROR] local YOLO port already in use: ${PORT}" >&2
      echo "[HINT] ss -ltnp | grep :${PORT}" >&2
      exit 1
    fi
    ;;
  *)
    python3 - "${HOST}" "${PORT}" <<'PY'
import socket, sys
host, port = sys.argv[1], int(sys.argv[2])
try:
    with socket.create_connection((host, port), timeout=2.0):
        pass
except OSError as exc:
    raise SystemExit(f"[ERROR] external YOLO endpoint unavailable {host}:{port}: {exc}")
print(f"[OK] external YOLO endpoint reachable: {host}:{port}")
PY
    ;;
esac

echo "[OK] vehicle ${ID} preflight passed"
