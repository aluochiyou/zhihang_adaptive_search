#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"

echo '[07] QGroundControl'
[[ -n "${ZHIHANG_QGC_EXECUTABLE}" ]] || {
  echo '[ERROR] QGroundControl executable is not configured' >&2
  exit 1
}
[[ -x "${ZHIHANG_QGC_EXECUTABLE}" ]] || {
  echo "[ERROR] QGroundControl is missing or not executable: ${ZHIHANG_QGC_EXECUTABLE}" >&2
  exit 1
}
QGC_ARGS=()
if [[ -n "${ZHIHANG_QGC_ARGS}" ]]; then
  read -r -a QGC_ARGS <<<"${ZHIHANG_QGC_ARGS}"
fi
exec "${ZHIHANG_QGC_EXECUTABLE}" "${QGC_ARGS[@]}"
