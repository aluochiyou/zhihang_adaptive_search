#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
PID_DIR="${HOME}/.cache/zhihang_adaptive_search_v6"
for id in 0 1 2; do
  port=$((ZHIHANG_YOLO_PORT_BASE+id))
  pid=""
  [[ -f "${PID_DIR}/yolo_v${id}.pid" ]] && pid="$(cat "${PID_DIR}/yolo_v${id}.pid")"
  alive=false
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && alive=true
  listening=false
  ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}$" && listening=true
  echo "v${id} pid=${pid:-none} alive=${alive} port=${port} listening=${listening}"
done
