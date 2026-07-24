#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/source_zhihang_ros_env.sh"
RUN="${1:-$(find "${HOME}/zhihang_search_runs_v6" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -n1 | cut -d' ' -f2-)}"
[[ -d "${RUN}" ]] || { echo '[ERROR] run not found' >&2; exit 2; }
rosrun zhihang_adaptive_search_v6 summarize_adaptive_run.py "${RUN}"
