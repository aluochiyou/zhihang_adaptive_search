#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/terminal_launcher_common.sh"
require_desktop_tools

START_QGC="${ZHIHANG_START_QGC}"
DEFERRED_RECORD_BAG=0
while (($#)); do
  case "$1" in
    --record-bag) DEFERRED_RECORD_BAG=1 ;;
    --no-qgc) START_QGC=0 ;;
    --qgc) START_QGC=1 ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

echo '[STEP 1/5] PX4/Gazebo'
open_named_terminal '[01] PX4-Gazebo' '105x28+0+0' "${ROOT}/terminal_commands/01_px4_gazebo.sh"
sleep 10
assert_terminal_running '[01] PX4-Gazebo'

echo '[STEP 2/5] Frame Guard'
open_named_terminal '[02] Frame-Guard' '90x15+1000+0' "${ROOT}/terminal_commands/02_guard.sh"
sleep 2
assert_terminal_running '[02] Frame-Guard'

echo '[STEP 3/5] Original XTDrone communication'
open_named_terminal '[03] XTDrone-Comm' '90x15+1000+300' "${ROOT}/terminal_commands/03_xtdrone_communication.sh"
sleep 2
assert_terminal_running_or_clean_exit '[03] XTDrone-Comm'
bash "${ROOT}/wait_xtdrone_communication_ready.sh" 45

echo '[STEP 4/5] Pose relay'
open_named_terminal '[04] Pose-Truth' '90x15+1000+600' "${ROOT}/terminal_commands/04_pose_ground_truth.sh"
sleep 2
assert_terminal_running_or_clean_exit '[04] Pose-Truth'

if ((START_QGC)); then
  echo '[STEP 5/5] QGroundControl'
  open_named_terminal '[07] QGroundControl' '90x15+100+100' "${ROOT}/terminal_commands/07_qgroundcontrol.sh"
  sleep 3
  assert_terminal_running_or_clean_exit '[07] QGroundControl'
else
  echo '[STEP 5/5] QGroundControl skipped'
fi

if ((DEFERRED_RECORD_BAG)); then
  echo '[INFO] score1.bag is deferred until all three real YOLO pipelines are ready.'
fi
echo '[INFO] model_state.py is intentionally NOT started in the base phase.'
echo '[INFO] target motion begins only after manager + 3 flight/perception/YOLO pipelines are ready.'
if [[ -x "${ROOT}/arrange_v6_7_windows.sh" && "$(resolve_terminal_backend)" == "gnome-terminal" ]]; then
  bash "${ROOT}/arrange_v6_7_windows.sh" || true
fi
echo '[OK] base terminals ready; target motion remains stopped.'
