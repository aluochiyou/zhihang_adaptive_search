#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT
export HOME="${TMP}/home"
mkdir -p "${HOME}"
cat > "${TMP}/profile.env" <<EOF
export ZHIHANG_YOLO_RUNTIME=system
export ZHIHANG_YOLO_ENV=yolo-test
export ZHIHANG_YOLO_DEVICES='0,1,cpu'
export ZHIHANG_YOLO_HOSTS='127.0.0.1,10.0.0.2,localhost'
export ZHIHANG_YOLO_PORT_BASE=18880
EOF
export ZHIHANG_PROFILE_FILE="${TMP}/profile.env"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/yolo_runtime_common.sh"
[[ "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 0 auto)" == "0" ]]
[[ "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 1 auto)" == "1" ]]
[[ "$(zh_csv_item "${ZHIHANG_YOLO_DEVICES}" 2 auto)" == "cpu" ]]
[[ "$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" 1 localhost)" == "10.0.0.2" ]]
[[ "${ZHIHANG_YOLO_PORT_BASE}" == "18880" ]]
echo 'V6.7.19 YOLO RUNTIME COMMON TEST PASSED'
