#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"

WS="${1:-${ZHIHANG_WS}}"
SRC="${ROOT}/zhihang_adaptive_search_v6"
DEST="${WS}/src/zhihang_adaptive_search_v6"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="${HOME}/xtdrone_backups/zhihang_adaptive_search_v6_before_v6_7_19_${STAMP}"

[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Do not use sudo.' >&2
  exit 2
}
[[ -d "${WS}/src" ]] || {
  echo "[ERROR] missing workspace source directory: ${WS}/src" >&2
  exit 2
}

mkdir -p "${HOME}/xtdrone_backups"
if [[ -e "${DEST}" ]]; then
  cp -a "${DEST}" "${BACKUP}"
  echo "[BACKUP] ${BACKUP}"
fi

declare -A BEFORE_HASH=()
for protected in \
  "${ZHIHANG_COMMUNICATION_DIR}/vtol_communication.py" \
  "${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_communication.sh" \
  "${ZHIHANG_COMMUNICATION_DIR}/multi_vehicle_commonication.sh"; do
  if [[ -f "${protected}" ]]; then
    BEFORE_HASH["${protected}"]="$(sha256sum "${protected}" | awk '{print $1}')"
  fi
done

rm -rf "${DEST}"
cp -a "${SRC}" "${DEST}"
chmod +x "${DEST}"/scripts/*.py "${DEST}"/setup.py

source "${ZHIHANG_ROS_SETUP}"
if [[ -n "${ZHIHANG_OPTIONAL_UNDERLAY}" && -f "${ZHIHANG_OPTIONAL_UNDERLAY}" ]]; then
  source "${ZHIHANG_OPTIONAL_UNDERLAY}"
fi
cd "${WS}"
catkin_make
source "${WS}/devel/setup.bash"

python3 - <<'PY'
from zhihang_adaptive_search_v6.common import build_plan, fov_project_nadir, validate_packet
assert callable(build_plan) and callable(fov_project_nadir) and callable(validate_packet)
print('[OK] shared Python package import')
PY

for protected in "${!BEFORE_HASH[@]}"; do
  after="$(sha256sum "${protected}" | awk '{print $1}')"
  if [[ "${after}" != "${BEFORE_HASH[${protected}]}" ]]; then
    echo "[ERROR] protected XTDrone file changed unexpectedly: ${protected}" >&2
    exit 1
  fi
  echo "[UNCHANGED] ${protected}"
done

echo "[OK] installed ${DEST}"
echo '[UNCHANGED] PX4, Gazebo, XTDrone communication and previous project packages were not modified.'
