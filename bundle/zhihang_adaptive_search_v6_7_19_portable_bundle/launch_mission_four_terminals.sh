#!/usr/bin/env bash
# Validation-mode four-terminal launcher retained for state-machine testing.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/shell_arg_utils.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
source "${ROOT}/terminal_launcher_common.sh"
require_desktop_tools

SCENE="${1:-scene_001_adaptive_v6_7_19_validation}"
MODEL="${2:-${ZHIHANG_YOLO_MODEL}}"
RUNTIME_ENV="${3:-${ZHIHANG_YOLO_ENV}}"
SEED="${4:-${ZHIHANG_DEFAULT_SEED}}"
RECORD_BAG="${5:-0}"

rm -rf "${ZHIHANG_V6_TERMINAL_STATUS_DIR}"
mkdir -p "${ZHIHANG_V6_TERMINAL_STATUS_DIR}"
rm -f /tmp/zhihang_v6_7_model_state_started.json

echo '[WARNING] validation mode may use the truth relay; do not use for formal competition scoring.'
echo '[CHECK] removing only stale V6 application processes; aircraft must be disarmed'
bash "${ROOT}/safe_cleanup_previous_run.sh"
if pgrep -af '[p]ython3 .*model_state\.py|[p]ython .*model_state\.py' >/dev/null; then
  echo '[ERROR] model_state.py is already running before application readiness.' >&2
  exit 1
fi

open_named_terminal '[08] V6.7.19 Validation Manager' '100x28+0+0' \
  "${ROOT}/terminal_commands/08_manager.sh" "${SCENE}" "${SEED}"
bash "${ROOT}/wait_application_terminal_ready.sh" manager - - \
  '[08] V6.7.19 Validation Manager' 45

open_named_terminal '[09] Vehicle-0' '100x28+960+0' \
  "${ROOT}/terminal_commands/09_vehicle_0.sh" "${MODEL}" "${RUNTIME_ENV}"
open_named_terminal '[10] Vehicle-1' '100x28+0+520' \
  "${ROOT}/terminal_commands/10_vehicle_1.sh" "${MODEL}" "${RUNTIME_ENV}"
open_named_terminal '[11] Vehicle-2' '100x28+960+520' \
  "${ROOT}/terminal_commands/11_vehicle_2.sh" "${MODEL}" "${RUNTIME_ENV}"

PORT0=$((ZHIHANG_YOLO_PORT_BASE))
PORT1=$((ZHIHANG_YOLO_PORT_BASE+1))
PORT2=$((ZHIHANG_YOLO_PORT_BASE+2))
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 0 "${PORT0}" '[09] Vehicle-0' 240
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 1 "${PORT1}" '[10] Vehicle-1' 240
bash "${ROOT}/wait_application_terminal_ready.sh" vehicle 2 "${PORT2}" '[11] Vehicle-2' 240
python3 "${ROOT}/wait_manager_application_ready.py" --timeout 420

if [[ "${RECORD_BAG}" == 1 ]]; then
  open_named_terminal '[06] Score1-Bag' '90x12+1000+820' \
    "${ROOT}/terminal_commands/06_score1_bag.sh"
  sleep 1
  assert_terminal_running '[06] Score1-Bag'
fi

open_named_terminal '[05] Target-Motion' '105x20+0+620' \
  "${ROOT}/terminal_commands/05_model_state.sh"
sleep 0.2
assert_terminal_running '[05] Target-Motion'
bash "${ROOT}/wait_target_motion_ready.sh" 30
SETTLE="$(rosparam get /zhihang_search_v6/mission/target_motion_settle_seconds 2>/dev/null || echo 0.5)"
sleep "${SETTLE}"
python3 "${ROOT}/authorize_manager_start.py" \
  --reason 'validation_model_state_started_after_three_yolo_ready'
python3 "${ROOT}/wait_manager_start.py" --timeout 60
echo '[OK] V6.7.19 validation mission start barrier published.'
