#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/source_zhihang_ros_env.sh"
rostopic pub -1 /zhihang/search_v6/manager/abort std_msgs/Bool 'data: true'
