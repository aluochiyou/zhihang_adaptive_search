#!/usr/bin/env bash
set -euo pipefail
BUNDLE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

HOME_FAKE="${TMP}/home"
ROS_FAKE="${TMP}/ros/setup.bash"
WS_FAKE="${HOME_FAKE}/xtdrone_competition_ws"
UNDERLAY_FAKE="${HOME_FAKE}/catkin_ws/devel/setup.bash"
PKG_FAKE="${WS_FAKE}/src/zhihang_adaptive_search_v6"
BIN_FAKE="${TMP}/bin"
mkdir -p "$(dirname "${ROS_FAKE}")" "${WS_FAKE}/devel" \
  "$(dirname "${UNDERLAY_FAKE}")" "${PKG_FAKE}/config" "${BIN_FAKE}"

cat > "${ROS_FAKE}" <<EOF_ROS
export PATH="${BIN_FAKE}:\${PATH}"
export ORDER="\${ORDER:+\${ORDER},}ros"
EOF_ROS
cat > "${UNDERLAY_FAKE}" <<'EOF_UNDERLAY'
export ORDER="${ORDER:+${ORDER},}underlay"
EOF_UNDERLAY
cat > "${WS_FAKE}/devel/setup.bash" <<'EOF_WS'
export ORDER="${ORDER:+${ORDER},}workspace"
export ROS_PACKAGE_PATH="${ZHIHANG_WS}/src:${ROS_PACKAGE_PATH:-}"
export LD_LIBRARY_PATH="${ZHIHANG_WS}/devel/lib:${LD_LIBRARY_PATH:-}"
EOF_WS
: > "${PKG_FAKE}/config/adaptive_search.yaml"
: > "${PKG_FAKE}/config/adaptive_search_formal.yaml"

cat > "${BIN_FAKE}/rospack" <<'EOF_ROSPACK'
#!/usr/bin/env bash
set -e
case "${1:-}" in
  profile) exit 0 ;;
  find)
    [[ "${2:-}" == 'zhihang_adaptive_search_v6' ]] || exit 1
    case ":${ROS_PACKAGE_PATH:-}:" in
      *":${ZHIHANG_WS}/src:"*) printf '%s\n' "${FAKE_PKG_DIR:?}" ;;
      *) exit 1 ;;
    esac
    ;;
  *) exit 1 ;;
esac
EOF_ROSPACK
chmod +x "${BIN_FAKE}/rospack"

COMMON_ENV=(
  "HOME=${HOME_FAKE}"
  "ZHIHANG_WS=${WS_FAKE}"
  "ZHIHANG_ROS_SETUP=${ROS_FAKE}"
  "ZHIHANG_OPTIONAL_UNDERLAY=${UNDERLAY_FAKE}"
  "FAKE_PKG_DIR=${PKG_FAKE}"
  "ZHIHANG_ENV_VERBOSE=0"
)

# Same-shell idempotency works, but a live-environment check forces repair if
# ROS_PACKAGE_PATH is reset inside the same process.
env "${COMMON_ENV[@]}" ORDER="" bash -c '
  set -euo pipefail
  source "$1/source_zhihang_ros_env.sh"
  first_order="$ORDER"
  source "$1/source_zhihang_ros_env.sh"
  [[ "$ORDER" == "$first_order" ]]
  [[ "$ZHIHANG_PKG_DIR" == "$FAKE_PKG_DIR" ]]
  [[ -f "$ZHIHANG_FORMAL_CONFIG" ]]
  [[ "${ZHIHANG_ENV_READY_BASHPID}" == "${BASHPID}" ]]
  ! export -p | grep -q "ZHIHANG_ENV_READY="

  # Emulate ~/.bashrc replacing the ROS overlay in the current shell.
  ROS_PACKAGE_PATH="/stale/underlay"
  export ROS_PACKAGE_PATH
  source "$1/source_zhihang_ros_env.sh"
  case ":$ROS_PACKAGE_PATH:" in
    *":$ZHIHANG_WS/src:"*) ;;
    *) echo "same-shell repair failed" >&2; exit 1 ;;
  esac
  rospack find zhihang_adaptive_search_v6 >/dev/null
  case "$-" in *u*) : ;; *) echo "nounset not restored" >&2; exit 1;; esac
' _ "${BUNDLE}"

# Reproduce the exact V6.7.12 failure: a new Bash inherits exported READY/config
# values, but an interactive profile has reset ROS_PACKAGE_PATH and LD paths.
# V6.7.13 must ignore the inherited marker and source the overlay again.
env "${COMMON_ENV[@]}" \
  ORDER="" \
  ZHIHANG_ENV_READY=1 \
  ZHIHANG_ENV_READY_WS="${WS_FAKE}" \
  ZHIHANG_ENV_READY_BASHPID=999999 \
  ZHIHANG_PKG_DIR="${PKG_FAKE}" \
  ZHIHANG_FORMAL_CONFIG="${PKG_FAKE}/config/adaptive_search_formal.yaml" \
  ZHIHANG_VALIDATION_CONFIG="${PKG_FAKE}/config/adaptive_search.yaml" \
  ROS_PACKAGE_PATH="/stale/from/bashrc" \
  LD_LIBRARY_PATH="/stale/lib" \
  bash -c '
    set -euo pipefail
    source "$1/source_zhihang_ros_env.sh"
    [[ "${ZHIHANG_ENV_READY_BASHPID}" == "${BASHPID}" ]]
    case ":$ROS_PACKAGE_PATH:" in
      *":$ZHIHANG_WS/src:"*) ;;
      *) echo "child overlay reload failed" >&2; exit 1 ;;
    esac
    [[ "$(rospack find zhihang_adaptive_search_v6)" == "$FAKE_PKG_DIR" ]]
    [[ "$ZHIHANG_FORMAL_CONFIG" == "$FAKE_PKG_DIR/config/adaptive_search_formal.yaml" ]]
  ' _ "${BUNDLE}"

# Terminal wrapper must explicitly remove every inherited derived/readiness key.
for key in \
  ZHIHANG_ENV_READY ZHIHANG_ENV_READY_WS ZHIHANG_ENV_READY_BASHPID \
  ZHIHANG_PKG_DIR ZHIHANG_FORMAL_CONFIG ZHIHANG_VALIDATION_CONFIG; do
  grep -q "unset .*${key}" "${BUNDLE}/terminal_launcher_common.sh"
done

# Missing workspace must fail before any `/config/...` path can be constructed.
set +e
env \
  HOME="${HOME_FAKE}" \
  ZHIHANG_WS="${TMP}/missing_ws" \
  ZHIHANG_ROS_SETUP="${ROS_FAKE}" \
  ZHIHANG_OPTIONAL_UNDERLAY="${TMP}/missing_underlay" \
  FAKE_PKG_DIR="${PKG_FAKE}" \
  ZHIHANG_ENV_VERBOSE=0 \
  bash -c 'source "$1/source_zhihang_ros_env.sh"' _ "${BUNDLE}" >/dev/null 2>&1
RC=$?
set -e
[[ "${RC}" == 32 ]] || { echo "expected missing workspace rc=32, got ${RC}" >&2; exit 1; }

echo 'V6.7.13 CHILD-SHELL ROS OVERLAY RELOAD TEST PASSED'
