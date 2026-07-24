#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT
export HOME="${TMP}/home"
mkdir -p "${HOME}/yolo_models"
printf 'dummy-model\n' > "${HOME}/yolo_models/best.pt"
source "${ROOT}/shell_arg_utils.sh"

[[ "$(zh_trim_arg '  yolo26  ')" == 'yolo26' ]]
[[ "$(zh_expand_user_path '  ~/yolo_models/best.pt  ')" == "${HOME}/yolo_models/best.pt" ]]
[[ "$(zh_expand_user_path '  $HOME/yolo_models/best.pt  ')" == "${HOME}/yolo_models/best.pt" ]]
RESOLVED="$(zh_resolve_model_path '  ~/yolo_models/best.pt  ')"
[[ "${RESOLVED}" == "$(readlink -f "${HOME}/yolo_models/best.pt")" ]]
RESOLVED="$(zh_resolve_model_path ' best.pt ')"
[[ "${RESOLVED}" == "$(readlink -f "${HOME}/yolo_models/best.pt")" ]]

echo 'V6.7.19 LAUNCHER ARGUMENT NORMALIZATION TEST PASSED'
