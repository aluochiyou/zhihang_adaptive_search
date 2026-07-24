#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/source_zhihang_ros_env.sh"
echo '[02] XTDrone frame guard'
exec roslaunch "${ZHIHANG_GUARD_ROS_PACKAGE}" "${ZHIHANG_GUARD_LAUNCH_FILE}"
