#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/yolo_runtime_common.sh"

[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Do not use sudo with the YOLO runtime.' >&2
  exit 2
}

if [[ $# -ge 1 && -n "${1}" ]]; then
  export ZHIHANG_YOLO_ENV="$1"
fi
zh_yolo_runtime_summary
zh_yolo_run - <<'PY'
import json, sys
import cv2, numpy, torch, ultralytics
from ultralytics import YOLO
print('[OK] python:', sys.version.split()[0], sys.executable)
print('[OK] torch:', torch.__version__)
print('[OK] cuda:', torch.cuda.is_available())
print('[OK] cuda devices:', torch.cuda.device_count())
if torch.cuda.is_available():
    for i in range(torch.cuda.device_count()):
        print(f'[OK] gpu[{i}]:', torch.cuda.get_device_name(i))
print('[OK] ultralytics:', ultralytics.__version__)
print('[OK] cv2:', cv2.__version__)
print('[OK] numpy:', numpy.__version__)
print('[OK] YOLO import:', YOLO)
PY

if [[ "${ZHIHANG_YOLO_REQUIRE_CUDA}" == "1" ]]; then
  zh_yolo_run -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)' || {
    echo '[ERROR] formal profile requires CUDA but CUDA is unavailable.' >&2
    echo '[HINT] use a CUDA-compatible PyTorch environment or explicitly set ZHIHANG_YOLO_REQUIRE_CUDA=0 for validation.' >&2
    exit 1
  }
fi
