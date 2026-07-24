#!/usr/bin/env bash
# Full portable formal launch: preflight -> base simulation -> application mission.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROFILE=""
SCENE=""
MODEL=""
RUNTIME_ENV=""
SEED=""
RECORD_BAG=""
NO_QGC=0

while (($#)); do
  case "$1" in
    --profile) PROFILE="${2:?missing value}"; shift ;;
    --scene) SCENE="${2:?missing value}"; shift ;;
    --model) MODEL="${2:?missing value}"; shift ;;
    --runtime-env|--conda-env) RUNTIME_ENV="${2:?missing value}"; shift ;;
    --seed) SEED="${2:?missing value}"; shift ;;
    --record-bag) RECORD_BAG=1 ;;
    --no-record-bag) RECORD_BAG=0 ;;
    --no-qgc) NO_QGC=1 ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

[[ -n "${PROFILE}" ]] && export ZHIHANG_PROFILE_FILE="${PROFILE}"
source "${ROOT}/load_machine_profile.sh"

SCENE="${SCENE:-${ZHIHANG_DEFAULT_SCENE}}"
MODEL="${MODEL:-${ZHIHANG_YOLO_MODEL}}"
RUNTIME_ENV="${RUNTIME_ENV:-${ZHIHANG_YOLO_ENV}}"
SEED="${SEED:-${ZHIHANG_DEFAULT_SEED}}"
RECORD_BAG="${RECORD_BAG:-${ZHIHANG_RECORD_BAG}}"

echo '============================================================'
echo ' Zhihang V6.7.19 portable formal mission'
echo " profile=${ZHIHANG_PROFILE_FILE:-<defaults>}"
echo " scene=${SCENE}"
echo " model=${MODEL}"
echo " runtime=${ZHIHANG_YOLO_RUNTIME}/${RUNTIME_ENV}"
echo ' Target motion starts only after all three real YOLO pipelines are ready.'
echo '============================================================'

bash "${ROOT}/preflight_launcher.sh" "${MODEL}" "${RUNTIME_ENV}"
BASE_ARGS=()
((NO_QGC)) && BASE_ARGS+=(--no-qgc)
((RECORD_BAG)) && BASE_ARGS+=(--record-bag)
bash "${ROOT}/launch_base_environment_one_click.sh" "${BASE_ARGS[@]}"
bash "${ROOT}/wait_base_ready.sh" 180

MISSION_ARGS=(
  --scene "${SCENE}"
  --model "${MODEL}"
  --runtime-env "${RUNTIME_ENV}"
  --seed "${SEED}"
)
[[ -n "${PROFILE}" ]] && MISSION_ARGS+=(--profile "${PROFILE}")
((RECORD_BAG)) && MISSION_ARGS+=(--record-bag)
bash "${ROOT}/launch_mission_formal_one_click.sh" "${MISSION_ARGS[@]}"
