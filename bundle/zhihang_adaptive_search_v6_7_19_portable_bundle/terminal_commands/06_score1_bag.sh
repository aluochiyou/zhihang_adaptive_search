#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

echo '[06] Optional competition score1.bag recording'
mkdir -p "${ZHIHANG_COMPETITION_DATA_DIR}"
cd "${ZHIHANG_COMPETITION_DATA_DIR}"
if [[ -f score1.bag ]]; then
  mv score1.bag "score1_before_v6_7_19_$(date +%Y%m%d_%H%M%S).bag"
fi
exec rosbag record -O score1 \
  /standard_vtol_0/mavros/state \
  /standard_vtol_1/mavros/state \
  /standard_vtol_2/mavros/state \
  /gazebo/model_states \
  /xtdrone/standard_vtol_0/cmd \
  /xtdrone/standard_vtol_1/cmd \
  /xtdrone/standard_vtol_2/cmd \
  /zhihang2026/static_targets/pose \
  /zhihang2026/dynamic_targets/pose
