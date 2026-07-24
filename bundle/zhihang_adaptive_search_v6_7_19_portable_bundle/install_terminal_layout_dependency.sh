#!/usr/bin/env bash
set -euo pipefail
if command -v wmctrl >/dev/null 2>&1; then
  echo '[OK] wmctrl already installed'
  exit 0
fi
echo '[INSTALL] wmctrl is required for precise workspace/window placement.'
echo '[INSTALL] sudo authentication may be requested once.'
sudo apt update
sudo apt install -y wmctrl
command -v wmctrl >/dev/null 2>&1 || { echo '[ERROR] wmctrl installation failed' >&2; exit 1; }
echo '[OK] wmctrl installed'
