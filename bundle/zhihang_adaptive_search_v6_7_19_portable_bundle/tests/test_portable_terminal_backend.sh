#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT
export HOME="${TMP}/home"
mkdir -p "${HOME}"
cat > "${TMP}/profile.env" <<EOF
export ZHIHANG_TERMINAL_BACKEND=tmux
export ZHIHANG_WS='${TMP}/ws'
EOF
export ZHIHANG_PROFILE_FILE="${TMP}/profile.env"
source "${ROOT}/terminal_launcher_common.sh"
[[ "$(resolve_terminal_backend)" == "tmux" ]]
[[ "$(terminal_slug '[08] V6.7.19 Formal Manager')" == "08_v6_7_19_formal_manager" ]]
echo 'V6.7.19 TERMINAL BACKEND TEST PASSED'
