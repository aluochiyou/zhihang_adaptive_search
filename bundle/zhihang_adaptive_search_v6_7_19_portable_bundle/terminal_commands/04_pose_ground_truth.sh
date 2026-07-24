#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

echo '[04] Three-vehicle pose relay required by XTDrone flight control'
echo "[04] script=${ZHIHANG_POSE_SCRIPT}"
[[ -x "${ZHIHANG_POSE_SCRIPT}" ]] || {
  echo "[ERROR] pose script missing or not executable: ${ZHIHANG_POSE_SCRIPT}" >&2
  exit 1
}
cd "$(dirname "${ZHIHANG_POSE_SCRIPT}")"
set +e
"./$(basename "${ZHIHANG_POSE_SCRIPT}")"
rc=$?
set -e
if (( rc != 0 )); then
  echo "[ERROR] pose initializer failed; rc=${rc}" >&2
  exit "${rc}"
fi
echo '[OK] pose initializer returned rc=0'
while true; do sleep 60; done
