#!/usr/bin/env bash
echo '[NOTICE] V6.7.18 hotfix entry point is retained only for compatibility.'
echo '[NOTICE] Redirecting to V6.7.19 portable installer.'
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/install_portable.sh" "$@"
