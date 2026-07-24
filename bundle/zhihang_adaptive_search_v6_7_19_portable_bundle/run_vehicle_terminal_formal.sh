#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 3 ]]; then
  echo "Usage: $0 ID RUNTIME_ENV MODEL [PORT] [HOST]" >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/yolo_runtime_common.sh"

ID="$(zh_trim_arg "$1")"
RUNTIME_ENV="$(zh_trim_arg "$2")"
MODEL="$(zh_expand_user_path "$3")"
PORT="$(zh_trim_arg "${4:-$((ZHIHANG_YOLO_PORT_BASE + ID))}")"
HOST="$(zh_trim_arg "${5:-$(zh_csv_item "${ZHIHANG_YOLO_HOSTS}" "${ID}" 127.0.0.1)}")"

export V6_FORMAL_CONFIG="${ZHIHANG_FORMAL_CONFIG}"
[[ -f "${V6_FORMAL_CONFIG}" ]] || {
  echo "[ERROR] formal configuration not found: ${V6_FORMAL_CONFIG}" >&2
  exit 36
}

echo "[OK] formal vehicle ${ID} package=${ZHIHANG_PKG_DIR}"
echo "[OK] formal vehicle ${ID} config=${V6_FORMAL_CONFIG}"
echo "[OK] formal vehicle ${ID} runtime=${ZHIHANG_YOLO_RUNTIME}/${RUNTIME_ENV}"
ZHIHANG_ENV_VERBOSE=0 exec bash "${ROOT}/run_vehicle_terminal.sh" \
  "${ID}" "${RUNTIME_ENV}" "${MODEL}" "${PORT}" "${HOST}"
