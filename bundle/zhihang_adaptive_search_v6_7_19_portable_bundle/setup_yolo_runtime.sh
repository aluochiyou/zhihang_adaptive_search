#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/yolo_runtime_common.sh"
source "${ROOT}/shell_arg_utils.sh"

[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Never use sudo for the YOLO runtime.' >&2
  exit 2
}

CREATE=0
INSTALL=0
ALLOW_CPU=0
PYTHON_VERSION="3.10"
RUNTIME="${ZHIHANG_YOLO_RUNTIME}"
ENV_NAME="${ZHIHANG_YOLO_ENV}"
VENV_PATH="${ZHIHANG_YOLO_VENV}"
MODEL="${ZHIHANG_YOLO_MODEL}"
TORCH_INDEX_URL="${ZHIHANG_TORCH_INDEX_URL:-}"
OFFLINE_WHEEL_DIR="${ZHIHANG_OFFLINE_WHEEL_DIR:-}"

while (($#)); do
  case "$1" in
    --create) CREATE=1 ;;
    --install) INSTALL=1 ;;
    --runtime) RUNTIME="${2:?missing value}"; shift ;;
    --env) ENV_NAME="${2:?missing value}"; shift ;;
    --venv) VENV_PATH="$(zh_expand_user_path "${2:?missing value}")"; shift ;;
    --model) MODEL="$(zh_expand_user_path "${2:?missing value}")"; shift ;;
    --python-version) PYTHON_VERSION="${2:?missing value}"; shift ;;
    --allow-cpu) ALLOW_CPU=1 ;;
    --torch-index-url) TORCH_INDEX_URL="${2:?missing value}"; shift ;;
    --offline-wheel-dir) OFFLINE_WHEEL_DIR="$(zh_expand_user_path "${2:?missing value}")"; shift ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

export ZHIHANG_YOLO_RUNTIME="${RUNTIME}"
export ZHIHANG_YOLO_ENV="${ENV_NAME}"
export ZHIHANG_YOLO_VENV="${VENV_PATH}"

if [[ "${RUNTIME}" == "auto" ]]; then
  if zh_find_conda_executable >/dev/null 2>&1; then
    RUNTIME="conda"
  else
    RUNTIME="venv"
  fi
  export ZHIHANG_YOLO_RUNTIME="${RUNTIME}"
fi

if ((CREATE)); then
  case "${RUNTIME}" in
    conda)
      CONDA_BIN="$(zh_find_conda_executable)" || {
        echo '[ERROR] conda/miniforge/mambaforge was not found.' >&2
        exit 1
      }
      if ! zh_conda_env_exists "${ENV_NAME}"; then
        echo "[CREATE] conda env=${ENV_NAME} python=${PYTHON_VERSION}"
        "${CONDA_BIN}" create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}" pip
      else
        echo "[OK] conda env already exists: ${ENV_NAME}"
      fi
      ;;
    venv)
      if [[ ! -x "${VENV_PATH}/bin/python" ]]; then
        echo "[CREATE] venv=${VENV_PATH}"
        python3 -m venv "${VENV_PATH}"
      else
        echo "[OK] venv already exists: ${VENV_PATH}"
      fi
      ;;
    system|direct)
      echo "[INFO] runtime=${RUNTIME}; no environment is created"
      ;;
    *)
      echo "[ERROR] unsupported runtime: ${RUNTIME}" >&2
      exit 2
      ;;
  esac
fi

zh_resolve_yolo_runtime >/dev/null

if ((INSTALL)); then
  echo '[INSTALL] upgrading pip/setuptools/wheel'
  PIP_EXTRA=()
  if [[ -n "${OFFLINE_WHEEL_DIR}" ]]; then
    [[ -d "${OFFLINE_WHEEL_DIR}" ]] || {
      echo "[ERROR] offline wheel directory missing: ${OFFLINE_WHEEL_DIR}" >&2
      exit 1
    }
    PIP_EXTRA+=(--no-index --find-links "${OFFLINE_WHEEL_DIR}")
  fi
  zh_yolo_pip install "${PIP_EXTRA[@]}" --upgrade pip setuptools wheel

  if ! zh_yolo_run -c 'import torch' >/dev/null 2>&1; then
    if [[ -n "${TORCH_INDEX_URL}" ]]; then
      echo "[INSTALL] installing PyTorch from index: ${TORCH_INDEX_URL}"
      zh_yolo_pip install "${PIP_EXTRA[@]}" --index-url "${TORCH_INDEX_URL}" torch torchvision
    else
      echo '[INSTALL] installing PyTorch with pip default wheel selection'
      echo '[NOTE] For a specific CUDA wheel, rerun with --torch-index-url URL.'
      zh_yolo_pip install "${PIP_EXTRA[@]}" torch torchvision
    fi
  else
    echo '[OK] torch already installed'
  fi

  echo '[INSTALL] installing YOLO core requirements'
  zh_yolo_pip install "${PIP_EXTRA[@]}" -r "${ROOT}/requirements_yolo26_core.txt"
fi

VERIFY_ARGS=(
  --model "${MODEL}"
  --device "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 0 auto)"
  --imgsz 640
  --quantize "${ZHIHANG_YOLO_QUANTIZE}"
  --minimum-fps "${ZHIHANG_YOLO_MINIMUM_FPS}"
  --report "${ROOT}/yolo_runtime_verification.json"
)
((ALLOW_CPU)) && VERIFY_ARGS+=(--allow-cpu)

zh_yolo_runtime_summary
zh_yolo_run "${ROOT}/verify_yolo_runtime.py" "${VERIFY_ARGS[@]}"

echo '[OK] YOLO runtime setup/verification complete'
