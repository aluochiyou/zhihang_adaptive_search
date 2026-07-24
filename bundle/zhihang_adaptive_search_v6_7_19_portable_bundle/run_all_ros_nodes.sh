#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/shell_arg_utils.sh"
SCENE="$(zh_trim_arg "${1:-scene_001_adaptive_v6_7_19}")"
MODEL="$(zh_expand_user_path "${2:-$HOME/yolo_models/best.pt}")"
ENV="$(zh_trim_arg "${3:-yolo26}")"
SEED="$(zh_trim_arg "${4:--1}")"
echo '[INFO] Delegating to the verified delayed-target-start mission launcher.'
exec bash "${ROOT}/launch_mission_one_click.sh" "${SCENE}" "${MODEL}" "${ENV}" "${SEED}"
