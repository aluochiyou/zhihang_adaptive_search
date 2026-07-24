#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
LIVE=0
while (($#)); do
  case "$1" in
    --live) LIVE=1 ;;
    --profile) export ZHIHANG_PROFILE_FILE="${2:?missing value}"; shift; source "${ROOT}/load_machine_profile.sh" ;;
    *) echo "[ERROR] unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
ARGS=(--config "${ROOT}/zhihang_adaptive_search_v6/config/adaptive_search_formal.yaml")
((LIVE)) && ARGS+=(--live)
exec python3 "${ROOT}/competition_readiness_check.py" "${ARGS[@]}"
