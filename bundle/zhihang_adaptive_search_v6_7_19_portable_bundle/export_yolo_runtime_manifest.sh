#!/usr/bin/env bash
# Export a reproducible snapshot of the selected YOLO runtime and machine.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE=""
OUTPUT=""
while (($#)); do
  case "$1" in
    --profile) PROFILE="${2:?missing value}"; shift ;;
    --output) OUTPUT="${2:?missing value}"; shift ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "${PROFILE}" ]] && export ZHIHANG_PROFILE_FILE="${PROFILE}"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/yolo_runtime_common.sh"
source "${ROOT}/shell_arg_utils.sh"
zh_resolve_yolo_runtime >/dev/null
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${OUTPUT:-${ROOT}/runtime_manifest_${STAMP}}"
mkdir -p "${OUTPUT}"

{
  echo "version=6.7.19"
  echo "generated_at=$(date --iso-8601=seconds)"
  echo "profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"
  echo "runtime=${ZHIHANG_YOLO_RUNTIME_RESOLVED}"
  echo "environment=${ZHIHANG_YOLO_ENV}"
  echo "model=$(zh_expand_user_path "${ZHIHANG_YOLO_MODEL}")"
  echo "devices=${ZHIHANG_YOLO_DEVICES}"
  echo "os=$(lsb_release -ds 2>/dev/null || uname -a)"
  echo "kernel=$(uname -r)"
  echo "architecture=$(uname -m)"
} > "${OUTPUT}/machine.txt"

MODEL="$(zh_expand_user_path "${ZHIHANG_YOLO_MODEL}")"
if [[ -f "${MODEL}" ]]; then
  sha256sum "${MODEL}" > "${OUTPUT}/model.sha256"
fi

zh_yolo_run - <<'PY' > "${OUTPUT}/python_runtime.json"
import json, platform, sys
out = {"python": sys.executable, "python_version": platform.python_version()}
for name in ("numpy", "cv2", "torch", "torchvision", "ultralytics"):
    try:
        mod = __import__(name)
        out[name] = getattr(mod, "__version__", "unknown")
    except Exception as exc:
        out[name] = None
        out[name + "_error"] = f"{type(exc).__name__}: {exc}"
try:
    import torch
    out["cuda_available"] = bool(torch.cuda.is_available())
    out["cuda_version"] = getattr(torch.version, "cuda", None)
    out["cudnn_version"] = torch.backends.cudnn.version()
    out["devices"] = [
        {
            "index": i,
            "name": torch.cuda.get_device_name(i),
            "memory_gb": round(torch.cuda.get_device_properties(i).total_memory / 1024**3, 2),
        }
        for i in range(torch.cuda.device_count())
    ]
except Exception as exc:
    out["cuda_probe_error"] = f"{type(exc).__name__}: {exc}"
print(json.dumps(out, ensure_ascii=False, indent=2))
PY
zh_yolo_run -m pip freeze > "${OUTPUT}/pip_freeze.txt"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi -q > "${OUTPUT}/nvidia_smi_q.txt" 2>&1 || true
  nvidia-smi --query-gpu=index,name,driver_version,memory.total,memory.free,compute_cap --format=csv \
    > "${OUTPUT}/nvidia_smi_summary.csv" 2>&1 || true
fi
if [[ "${ZHIHANG_YOLO_RUNTIME_RESOLVED}" == "conda" ]]; then
  CONDA_BIN="$(zh_find_conda_executable)"
  "${CONDA_BIN}" env export -n "${ZHIHANG_YOLO_ENV}" --no-builds \
    > "${OUTPUT}/conda_environment.yml" 2>/dev/null || true
fi

echo "[OK] runtime manifest: ${OUTPUT}"
