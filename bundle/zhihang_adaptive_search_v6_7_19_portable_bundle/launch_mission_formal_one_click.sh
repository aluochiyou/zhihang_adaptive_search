#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve --profile before loading defaults.
_profile=""
_prev=""
for _arg in "$@"; do
  if [[ "${_prev}" == "--profile" ]]; then
    _profile="${_arg}"
    break
  fi
  _prev="${_arg}"
done
if [[ -n "${_profile}" ]]; then
  export ZHIHANG_PROFILE_FILE="${_profile}"
fi
unset _profile _prev _arg

source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/terminal_launcher_common.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"
require_desktop_tools

SCENE="${ZHIHANG_DEFAULT_SCENE}"
MODEL_RAW="${ZHIHANG_YOLO_MODEL}"
RUNTIME_ENV="${ZHIHANG_YOLO_ENV}"
SEED="${ZHIHANG_DEFAULT_SEED}"
RECORD_BAG="${ZHIHANG_RECORD_BAG}"
POSITIONAL=()

while (($#)); do
  case "$1" in
    --profile) shift ;;  # already applied
    --scene) SCENE="${2:?missing value after --scene}"; shift ;;
    --model) MODEL_RAW="${2:?missing value after --model}"; shift ;;
    --runtime-env|--conda-env) RUNTIME_ENV="${2:?missing value}"; shift ;;
    --seed) SEED="${2:?missing value after --seed}"; shift ;;
    --record-bag) RECORD_BAG=1 ;;
    --no-record-bag) RECORD_BAG=0 ;;
    --*) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
    *) POSITIONAL+=("$1") ;;
  esac
  shift
done

# Backward-compatible positional form:
#   script SCENE MODEL ENV SEED [--record-bag]
if ((${#POSITIONAL[@]} > 0)); then SCENE="${POSITIONAL[0]}"; fi
if ((${#POSITIONAL[@]} > 1)); then MODEL_RAW="${POSITIONAL[1]}"; fi
if ((${#POSITIONAL[@]} > 2)); then RUNTIME_ENV="${POSITIONAL[2]}"; fi
if ((${#POSITIONAL[@]} > 3)); then SEED="${POSITIONAL[3]}"; fi

SCENE="$(zh_trim_arg "${SCENE}")"
RUNTIME_ENV="$(zh_trim_arg "${RUNTIME_ENV}")"
SEED="$(zh_trim_arg "${SEED}")"
if ! MODEL="$(zh_resolve_model_path "${MODEL_RAW}")"; then
  echo "[ERROR] YOLO model not found after argument normalization: $(zh_expand_user_path "${MODEL_RAW}")" >&2
  printf '[DETAIL] raw model argument=%q\n' "${MODEL_RAW}" >&2
  echo '[HINT] run configure_portable_machine.sh and setup_yolo_runtime.sh first.' >&2
  exit 1
fi
[[ -n "${SCENE}" ]] || { echo '[ERROR] empty scene id' >&2; exit 2; }
[[ -n "${RUNTIME_ENV}" ]] || { echo '[ERROR] empty runtime environment name' >&2; exit 2; }
[[ -n "${SEED}" ]] || SEED=-1

export ZHIHANG_YOLO_ENV="${RUNTIME_ENV}"
zh_resolve_yolo_runtime >/dev/null

echo "[ARG-NORMALIZED] profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"
echo "[ARG-NORMALIZED] scene=${SCENE}"
echo "[ARG-NORMALIZED] model=${MODEL}"
echo "[ARG-NORMALIZED] runtime=${ZHIHANG_YOLO_RUNTIME_RESOLVED}/${RUNTIME_ENV} seed=${SEED}"
echo "[ARG-NORMALIZED] devices=${ZHIHANG_YOLO_DEVICES} ports=${ZHIHANG_YOLO_PORT_BASE}..$((ZHIHANG_YOLO_PORT_BASE+2))"

rm -rf "${ZHIHANG_V6_TERMINAL_STATUS_DIR}"
mkdir -p "${ZHIHANG_V6_TERMINAL_STATUS_DIR}"
bash "${ROOT}/wait_base_ready.sh" 30
bash "${ROOT}/safe_cleanup_previous_run.sh"
if pgrep -af '[p]ython3 .*model_state\.py|[p]ython .*model_state\.py' >/dev/null; then
  echo '[ERROR] model_state.py is already running; stop it before delayed-start formal launch.' >&2
  exit 1
fi
rm -f /tmp/zhihang_v6_7_model_state_started.json

open_named_terminal '[08] V6.7.19 Formal Manager' '100x28+0+0' \
  "${ROOT}/terminal_commands/08_manager_formal.sh" "${SCENE}" "${SEED}"
bash "${ROOT}/wait_application_terminal_ready.sh" manager - - \
  '[08] V6.7.19 Formal Manager' 45

open_named_terminal '[09] Formal Vehicle-0' '100x28+960+0' \
  "${ROOT}/terminal_commands/09_vehicle_0_formal.sh" "${MODEL}" "${RUNTIME_ENV}"
open_named_terminal '[10] Formal Vehicle-1' '100x28+0+520' \
  "${ROOT}/terminal_commands/10_vehicle_1_formal.sh" "${MODEL}" "${RUNTIME_ENV}"
open_named_terminal '[11] Formal Vehicle-2' '100x28+960+520' \
  "${ROOT}/terminal_commands/11_vehicle_2_formal.sh" "${MODEL}" "${RUNTIME_ENV}"

PORT0=$((ZHIHANG_YOLO_PORT_BASE))
PORT1=$((ZHIHANG_YOLO_PORT_BASE+1))
PORT2=$((ZHIHANG_YOLO_PORT_BASE+2))
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 0 "${PORT0}" '[09] Formal Vehicle-0' 240
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 1 "${PORT1}" '[10] Formal Vehicle-1' 240
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 2 "${PORT2}" '[11] Formal Vehicle-2' 240

echo '[WAIT] all nodes/endpoints exist; waiting for three real YOLO camera pipelines to sustain >=10 Hz'
python3 "${ROOT}/wait_manager_application_ready.py" --timeout 420

if ((RECORD_BAG)); then
  open_named_terminal '[06] Score1-Bag' '90x12+1000+820' \
    "${ROOT}/terminal_commands/06_score1_bag.sh"
  sleep 1
  assert_terminal_running '[06] Score1-Bag'
fi

open_named_terminal '[05] Target-Motion' '105x20+0+620' \
  "${ROOT}/terminal_commands/05_model_state.sh"
bash "${ROOT}/wait_target_motion_ready.sh" 30
ZHIHANG_ENV_VERBOSE=0 source "${ROOT}/source_zhihang_ros_env.sh"
SETTLE="$(rosparam get /zhihang_search_v6/mission/target_motion_settle_seconds 2>/dev/null || echo 0.5)"
sleep "${SETTLE}"
python3 "${ROOT}/authorize_manager_start.py" \
  --reason 'formal_model_state_started_after_three_real_yolo_ready'
python3 "${ROOT}/wait_manager_start.py" --timeout 60
echo '[OK] V6.7.19 portable formal mission start barrier published.'
