#!/usr/bin/env bash
# Compatibility alias for the portable formal full launcher.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/launch_portable_formal_one_click.sh" "$@"
