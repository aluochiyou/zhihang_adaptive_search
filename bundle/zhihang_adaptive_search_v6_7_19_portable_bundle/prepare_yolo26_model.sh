#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/yolo_runtime_common.sh"
source "${ROOT}/shell_arg_utils.sh"

[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Never use sudo.' >&2
  exit 2
}

ENV_NAME="$(zh_trim_arg "${1:-${ZHIHANG_YOLO_ENV}}")"
INPUT="$(zh_expand_user_path "${2:-yolo26n.pt}")"
MODEL_DIR="${YOLO_MODEL_DIR:-${HOME}/yolo_models}"
export ZHIHANG_YOLO_ENV="${ENV_NAME}"
mkdir -p "${MODEL_DIR}"

if [[ -f "${INPUT}" ]]; then
  TARGET="$(readlink -f "${INPUT}")"
elif [[ "${INPUT}" == */* ]]; then
  TARGET="$(realpath -m "${INPUT}")"
else
  TARGET="${MODEL_DIR}/${INPUT}"
fi

zh_resolve_yolo_runtime >/dev/null
if [[ ! -f "${TARGET}" ]]; then
  echo "[MODEL] requesting Ultralytics model: ${TARGET}"
  zh_yolo_run - "${TARGET}" <<'PY'
from pathlib import Path
from ultralytics import YOLO
import os, sys
p = Path(sys.argv[1]).expanduser().resolve()
p.parent.mkdir(parents=True, exist_ok=True)
os.chdir(p.parent)
YOLO(p.name)
if not p.exists() or p.stat().st_size < 100000:
    raise SystemExit(f'[ERROR] invalid model {p}')
print(p)
PY
fi

[[ -f "${TARGET}" ]] || {
  echo "[ERROR] model was not created: ${TARGET}" >&2
  exit 1
}
echo "MODEL_PATH=${TARGET}"
