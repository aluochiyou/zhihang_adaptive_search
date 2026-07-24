#!/usr/bin/env bash
set -euo pipefail
PID_DIR="${HOME}/.cache/zhihang_adaptive_search_v6"
for id in 0 1 2; do
  f="${PID_DIR}/yolo_v${id}.pid"
  [[ -f "${f}" ]] || continue
  pid="$(cat "${f}")"
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..30}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "${pid}" 2>/dev/null || true
  rm -f "${f}"
done
pkill -TERM -f 'zhihang_adaptive_search_v6/scripts/yolo26_single_worker.py' 2>/dev/null || true
echo '[OK] V6 YOLO workers stopped'
