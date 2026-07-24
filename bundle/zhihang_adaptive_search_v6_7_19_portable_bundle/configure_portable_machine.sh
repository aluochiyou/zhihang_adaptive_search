#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ ${EUID} -ne 0 ]] || {
  echo '[ERROR] Do not use sudo. The profile is per-user.' >&2
  exit 2
}
exec python3 "${ROOT}/configure_portable_machine.py" "$@"
