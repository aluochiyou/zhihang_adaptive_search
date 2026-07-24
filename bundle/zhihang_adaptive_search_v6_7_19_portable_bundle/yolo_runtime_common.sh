#!/usr/bin/env bash
# Shared runtime abstraction for YOLO. Source this file after load_machine_profile.sh.
# Supported runtimes:
#   conda  - conda run -n ENV python
#   venv   - VENV/bin/python
#   system - python3 from PATH
#   direct - explicit ZHIHANG_YOLO_PYTHON
#   auto   - direct -> conda -> venv -> system

_zh_yolo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_zh_yolo_root}/load_machine_profile.sh"

zh_csv_item() {
  local csv="$1" index="$2" fallback="${3:-}"
  local -a values=()
  IFS=',' read -r -a values <<<"${csv}"
  if (( index >= 0 && index < ${#values[@]} )); then
    local value="${values[$index]}"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s\n' "${value:-${fallback}}"
  else
    printf '%s\n' "${fallback}"
  fi
}

zh_find_conda_executable() {
  if [[ -n "${CONDA_EXE:-}" && -x "${CONDA_EXE}" ]]; then
    printf '%s\n' "${CONDA_EXE}"
    return 0
  fi
  if command -v conda >/dev/null 2>&1; then
    command -v conda
    return 0
  fi
  for candidate in \
    "${HOME}/miniconda3/bin/conda" \
    "${HOME}/anaconda3/bin/conda" \
    "${HOME}/mambaforge/bin/conda" \
    "${HOME}/miniforge3/bin/conda"; do
    if [[ -x "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

zh_conda_env_exists() {
  local env_name="$1" conda_bin
  conda_bin="$(zh_find_conda_executable)" || return 1
  local env_json
  env_json="$("${conda_bin}" env list --json 2>/dev/null)" || return 1
  python3 - "${env_name}" "${env_json}" <<'PY'
import json, os, sys
name = sys.argv[1]
try:
    data = json.loads(sys.argv[2])
except Exception:
    raise SystemExit(1)
for path in data.get("envs", []):
    if os.path.basename(path.rstrip(os.sep)) == name or path == name:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

zh_python_has_yolo_stack() {
  local python_bin="$1"
  [[ -x "${python_bin}" || "${python_bin}" == "python3" || "${python_bin}" == "python" ]] || return 1
  "${python_bin}" -c 'import cv2, numpy, torch, ultralytics; from ultralytics import YOLO' \
    >/dev/null 2>&1
}

zh_resolve_yolo_runtime() {
  local requested="${1:-${ZHIHANG_YOLO_RUNTIME:-auto}}"
  local runtime="${requested}"
  if [[ "${runtime}" == "auto" ]]; then
    if [[ -n "${ZHIHANG_YOLO_PYTHON:-}" ]] && zh_python_has_yolo_stack "${ZHIHANG_YOLO_PYTHON}"; then
      runtime="direct"
    elif zh_conda_env_exists "${ZHIHANG_YOLO_ENV}"; then
      runtime="conda"
    elif [[ -x "${ZHIHANG_YOLO_VENV}/bin/python" ]] && \
         zh_python_has_yolo_stack "${ZHIHANG_YOLO_VENV}/bin/python"; then
      runtime="venv"
    elif command -v python3 >/dev/null 2>&1 && zh_python_has_yolo_stack "$(command -v python3)"; then
      runtime="system"
    else
      echo '[ERROR] unable to auto-detect a usable YOLO Python runtime' >&2
      echo '[HINT] run: bash setup_yolo_runtime.sh --create' >&2
      return 51
    fi
  fi

  case "${runtime}" in
    conda)
      zh_conda_env_exists "${ZHIHANG_YOLO_ENV}" || {
        echo "[ERROR] conda environment not found: ${ZHIHANG_YOLO_ENV}" >&2
        return 52
      }
      ;;
    venv)
      [[ -x "${ZHIHANG_YOLO_VENV}/bin/python" ]] || {
        echo "[ERROR] venv Python not found: ${ZHIHANG_YOLO_VENV}/bin/python" >&2
        return 53
      }
      ;;
    system)
      command -v python3 >/dev/null 2>&1 || {
        echo '[ERROR] system python3 not found' >&2
        return 54
      }
      ;;
    direct)
      [[ -x "${ZHIHANG_YOLO_PYTHON}" ]] || {
        echo "[ERROR] direct YOLO Python is not executable: ${ZHIHANG_YOLO_PYTHON}" >&2
        return 55
      }
      ;;
    *)
      echo "[ERROR] unsupported YOLO runtime: ${runtime}" >&2
      return 56
      ;;
  esac
  export ZHIHANG_YOLO_RUNTIME_RESOLVED="${runtime}"
  printf '%s\n' "${runtime}"
}

zh_yolo_run() {
  local runtime
  runtime="${ZHIHANG_YOLO_RUNTIME_RESOLVED:-}"
  [[ -n "${runtime}" ]] || runtime="$(zh_resolve_yolo_runtime)"
  case "${runtime}" in
    conda)
      local conda_bin
      conda_bin="$(zh_find_conda_executable)" || {
        echo '[ERROR] conda executable not found' >&2
        return 57
      }
      "${conda_bin}" run --no-capture-output -n "${ZHIHANG_YOLO_ENV}" python "$@"
      ;;
    venv)
      "${ZHIHANG_YOLO_VENV}/bin/python" "$@"
      ;;
    system)
      python3 "$@"
      ;;
    direct)
      "${ZHIHANG_YOLO_PYTHON}" "$@"
      ;;
  esac
}

zh_yolo_pip() {
  zh_yolo_run -m pip "$@"
}

zh_resolve_device_for_vehicle() {
  local vehicle_id="$1" requested
  requested="$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" "${vehicle_id}" "auto")"
  printf '%s\n' "${requested:-auto}"
}

zh_yolo_runtime_summary() {
  local runtime
  runtime="$(zh_resolve_yolo_runtime)"
  echo "[YOLO-RUNTIME] type=${runtime}"
  case "${runtime}" in
    conda) echo "[YOLO-RUNTIME] conda_env=${ZHIHANG_YOLO_ENV}" ;;
    venv) echo "[YOLO-RUNTIME] venv=${ZHIHANG_YOLO_VENV}" ;;
    direct) echo "[YOLO-RUNTIME] python=${ZHIHANG_YOLO_PYTHON}" ;;
    system) echo "[YOLO-RUNTIME] python=$(command -v python3)" ;;
  esac
}

unset _zh_yolo_root
