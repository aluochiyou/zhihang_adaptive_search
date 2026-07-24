#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/yolo_runtime_common.sh"

ID="${1:?vehicle id}"
MODEL="${2:?model}"
PORT="${3:?port}"
DEVICE="${4:-auto}"
zh_resolve_yolo_runtime >/dev/null
PKG="$(rospack find zhihang_adaptive_search_v6)"
WORKER="${PKG}/scripts/yolo26_single_worker.py"
ADAPTIVE_ARGS=()
[[ "${ZHIHANG_YOLO_ADAPTIVE}" == "1" ]] && ADAPTIVE_ARGS+=(--adaptive)

exec 3>&1
zh_yolo_run "${WORKER}" \
  --vehicle-id "${ID}" \
  --host "${ZHIHANG_YOLO_BIND_HOST}" \
  --port "${PORT}" \
  --model "${MODEL}" \
  --device "${DEVICE}" \
  --conf "${ZHIHANG_YOLO_CONFIDENCE}" \
  --iou "${ZHIHANG_YOLO_IOU}" \
  --quantize "${ZHIHANG_YOLO_QUANTIZE}" \
  "${ADAPTIVE_ARGS[@]}" \
  --adaptive-sizes "${ZHIHANG_YOLO_ADAPTIVE_SIZES}" \
  --minimum-fps "${ZHIHANG_YOLO_MINIMUM_FPS}" \
  --performance-window "${ZHIHANG_YOLO_PERFORMANCE_WINDOW}" \
  --adaptive-check-interval "${ZHIHANG_YOLO_ADAPTIVE_CHECK_INTERVAL}" \
  --warmup-iterations "${ZHIHANG_YOLO_WARMUP_ITERATIONS}" \
  --socket-timeout "${ZHIHANG_YOLO_SOCKET_TIMEOUT}"
