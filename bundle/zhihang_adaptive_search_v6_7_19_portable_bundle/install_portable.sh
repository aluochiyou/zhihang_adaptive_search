#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROFILE=""
SKIP_YOLO_CHECK=0
while (($#)); do
  case "$1" in
    --profile) PROFILE="${2:?missing value}"; shift ;;
    --skip-yolo-check) SKIP_YOLO_CHECK=1 ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

[[ -n "${PROFILE}" ]] && export ZHIHANG_PROFILE_FILE="${PROFILE}"
source "${ROOT}/load_machine_profile.sh"

[[ -f "${ZHIHANG_PROFILE_FILE:-}" ]] || {
  echo '[ERROR] machine profile has not been generated.' >&2
  echo '[HINT] bash configure_portable_machine.sh --model "$HOME/yolo_models/best.pt" --install-user-profile' >&2
  exit 1
}

echo '[STEP 1/4] Package self-validation'
python3 "${ROOT}/validate_package.py"

echo '[STEP 2/4] ROS package backup, install and catkin_make'
bash "${ROOT}/install_to_workspace.sh" "${ZHIHANG_WS}"

echo '[STEP 3/4] Installed package verification'
python3 "${ROOT}/verify_install.py" "${ZHIHANG_WS}"

if ((SKIP_YOLO_CHECK)); then
  echo '[STEP 4/4] YOLO runtime verification skipped by user'
else
  echo '[STEP 4/4] YOLO runtime and real model verification'
  bash "${ROOT}/check_yolo26_env.sh" "${ZHIHANG_YOLO_ENV}"
  ARGS=(
    --model "${ZHIHANG_YOLO_MODEL}"
    --device "$(bash -lc "source '${ROOT}/load_machine_profile.sh'; source '${ROOT}/yolo_runtime_common.sh'; zh_csv_item '${ZHIHANG_YOLO_DEVICES}' 0 auto")"
    --quantize "${ZHIHANG_YOLO_QUANTIZE}"
    --iterations 5
    --minimum-fps 0.1
    --report "${ROOT}/install_yolo_verification.json"
  )
  [[ "${ZHIHANG_YOLO_REQUIRE_CUDA}" == "0" ]] && ARGS+=(--allow-cpu)
  source "${ROOT}/yolo_runtime_common.sh"
  zh_resolve_yolo_runtime >/dev/null
  zh_yolo_run "${ROOT}/verify_yolo_runtime.py" "${ARGS[@]}"
fi

echo '[OK] V6.7.19 portable installation complete'
