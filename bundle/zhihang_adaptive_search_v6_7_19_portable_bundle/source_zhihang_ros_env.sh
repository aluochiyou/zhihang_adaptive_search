#!/usr/bin/env bash
# V6.7.19 portable shared ROS overlay loader.
# Source this file from every manager/vehicle/application terminal.
# It does not modify PX4/Gazebo or original XTDrone communication scripts.


_ZHIHANG_SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_ZHIHANG_SOURCE_ROOT}/load_machine_profile.sh"
unset _ZHIHANG_SOURCE_ROOT

# Callers use `set -u`; ROS/catkin setup files may inspect optional variables.
# Temporarily disable nounset and restore it before returning.
_ZHIHANG_ENV_HAD_NOUNSET=0
case "$-" in
  *u*) _ZHIHANG_ENV_HAD_NOUNSET=1; set +u ;;
esac

_zhihang_restore_nounset() {
  if [[ "${_ZHIHANG_ENV_HAD_NOUNSET:-0}" == '1' ]]; then
    set -u
  fi
}

_zhihang_cleanup() {
  _zhihang_restore_nounset
  unset -f _zhihang_restore_nounset _zhihang_cleanup _zhihang_env_is_live 2>/dev/null || true
  unset _ZHIHANG_ENV_HAD_NOUNSET _ZHIHANG_ENV_THIS_SHELL 2>/dev/null || true
}

# Cross-computer overrides:
#   export ZHIHANG_WS=/path/to/xtdrone_competition_ws
#   export ZHIHANG_ROS_SETUP=/opt/ros/noetic/setup.bash
ZHIHANG_WS="${ZHIHANG_WS:-${HOME}/xtdrone_competition_ws}"
ZHIHANG_ROS_SETUP="${ZHIHANG_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
ZHIHANG_OPTIONAL_UNDERLAY="${ZHIHANG_OPTIONAL_UNDERLAY:-${HOME}/catkin_ws/devel/setup.bash}"
ZHIHANG_MESSAGE_WS_SETUP="${ZHIHANG_MESSAGE_WS_SETUP:-${HOME}/zhihang_ws/devel/setup.bash}"
export ZHIHANG_WS ZHIHANG_ROS_SETUP ZHIHANG_OPTIONAL_UNDERLAY ZHIHANG_MESSAGE_WS_SETUP

# V6.7.12 exported a process-readiness flag.  A gnome-terminal child inherited
# that flag, then ~/.bashrc reset ROS_PACKAGE_PATH/LD_LIBRARY_PATH.  The child
# incorrectly skipped the overlay source step.  V6.7.19 makes readiness local
# to the current Bash process and validates the live ROS environment before an
# idempotent early return.
_ZHIHANG_ENV_THIS_SHELL="${BASHPID:-$$}"
if [[ "${ZHIHANG_ENV_READY_BASHPID:-}" != "${_ZHIHANG_ENV_THIS_SHELL}" ]]; then
  unset ZHIHANG_ENV_READY ZHIHANG_ENV_READY_WS ZHIHANG_ENV_READY_BASHPID
fi

_zhihang_env_is_live() {
  [[ "${ZHIHANG_ENV_READY:-0}" == '1' ]] || return 1
  [[ "${ZHIHANG_ENV_READY_WS:-}" == "${ZHIHANG_WS}" ]] || return 1
  [[ "${ZHIHANG_ENV_READY_BASHPID:-}" == "${_ZHIHANG_ENV_THIS_SHELL}" ]] || return 1
  [[ -n "${ZHIHANG_PKG_DIR:-}" ]] || return 1
  [[ -f "${ZHIHANG_FORMAL_CONFIG:-/nonexistent}" ]] || return 1
  [[ -f "${ZHIHANG_VALIDATION_CONFIG:-/nonexistent}" ]] || return 1
  case ":${ROS_PACKAGE_PATH:-}:" in
    *":${ZHIHANG_WS}/src:"*) ;;
    *) return 1 ;;
  esac
  command -v rospack >/dev/null 2>&1 || return 1
  local found
  found="$(rospack find zhihang_adaptive_search_v6 2>/dev/null || true)"
  [[ -n "${found}" && "${found}" == "${ZHIHANG_PKG_DIR}" ]] || return 1
  return 0
}

if _zhihang_env_is_live; then
  if [[ "${ZHIHANG_ENV_VERBOSE:-1}" == '1' ]]; then
    echo "[OK] ROS overlay already loaded in current shell: ${ZHIHANG_WS}"
  fi
  _zhihang_cleanup
  return 0 2>/dev/null || exit 0
fi

# Any stale paths/derived values from a parent terminal must not be trusted.
unset ZHIHANG_PKG_DIR ZHIHANG_FORMAL_CONFIG ZHIHANG_VALIDATION_CONFIG

if [[ ! -f "${ZHIHANG_ROS_SETUP}" ]]; then
  printf '[ERROR] ROS Noetic setup file not found: %s\n' "${ZHIHANG_ROS_SETUP}" >&2
  _zhihang_cleanup
  return 31 2>/dev/null || exit 31
fi

# 1) ROS base.
# shellcheck disable=SC1090
source "${ZHIHANG_ROS_SETUP}"

# 2) Optional project underlay, retained for the user's PX4/Gazebo environment.
if [[ -f "${ZHIHANG_OPTIONAL_UNDERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${ZHIHANG_OPTIONAL_UNDERLAY}"
fi

# 3) Official competition custom-message workspace. It is kept separate from
# the algorithm workspace and is never overwritten by this package.
if [[ -f "${ZHIHANG_MESSAGE_WS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ZHIHANG_MESSAGE_WS_SETUP}"
fi

# 4) Competition algorithm workspace MUST be last so its package wins over older overlays.
if [[ ! -f "${ZHIHANG_WS}/devel/setup.bash" ]]; then
  printf '[ERROR] competition workspace setup not found: %s\n' \
    "${ZHIHANG_WS}/devel/setup.bash" >&2
  printf '[HINT] cd %q && source /opt/ros/noetic/setup.bash && catkin_make\n' \
    "${ZHIHANG_WS}" >&2
  _zhihang_cleanup
  return 32 2>/dev/null || exit 32
fi
# shellcheck disable=SC1090
source "${ZHIHANG_WS}/devel/setup.bash"

# Prefix the current source space if inherited terminal state is stale.
case ":${ROS_PACKAGE_PATH:-}:" in
  *":${ZHIHANG_WS}/src:"*) ;;
  *) export ROS_PACKAGE_PATH="${ZHIHANG_WS}/src${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}" ;;
esac

if ! command -v rospack >/dev/null 2>&1; then
  printf '[ERROR] rospack is unavailable after sourcing ROS Noetic.\n' >&2
  _zhihang_cleanup
  return 33 2>/dev/null || exit 33
fi

rospack profile >/dev/null 2>&1 || true
ZHIHANG_PKG_DIR="$(rospack find zhihang_adaptive_search_v6 2>/dev/null || true)"
if [[ -z "${ZHIHANG_PKG_DIR}" ]]; then
  printf "[ERROR] package 'zhihang_adaptive_search_v6' is not visible\n" >&2
  printf '[DEBUG] shell_pid=%s\n' "${_ZHIHANG_ENV_THIS_SHELL}" >&2
  printf '[DEBUG] ZHIHANG_WS=%s\n' "${ZHIHANG_WS}" >&2
  printf '[DEBUG] ROS_PACKAGE_PATH=%s\n' "${ROS_PACKAGE_PATH:-<empty>}" >&2
  printf '[DEBUG] LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-<empty>}" >&2
  printf '[HINT] reinstall/build the package, then retry from the V6.7.19 bundle.\n' >&2
  _zhihang_cleanup
  return 34 2>/dev/null || exit 34
fi

ZHIHANG_FORMAL_CONFIG="${ZHIHANG_PKG_DIR}/config/adaptive_search_formal.yaml"
ZHIHANG_VALIDATION_CONFIG="${ZHIHANG_PKG_DIR}/config/adaptive_search.yaml"
export ZHIHANG_PKG_DIR ZHIHANG_FORMAL_CONFIG ZHIHANG_VALIDATION_CONFIG

if [[ ! -f "${ZHIHANG_VALIDATION_CONFIG}" ]]; then
  printf '[ERROR] validation config missing: %s\n' "${ZHIHANG_VALIDATION_CONFIG}" >&2
  _zhihang_cleanup
  return 35 2>/dev/null || exit 35
fi
if [[ ! -f "${ZHIHANG_FORMAL_CONFIG}" ]]; then
  printf '[ERROR] formal config missing: %s\n' "${ZHIHANG_FORMAL_CONFIG}" >&2
  _zhihang_cleanup
  return 36 2>/dev/null || exit 36
fi

# Deliberately do NOT export readiness markers.  Every new Bash process must
# verify/source its own ROS overlay.  Only path/config outputs are exported.
ZHIHANG_ENV_READY=1
ZHIHANG_ENV_READY_WS="${ZHIHANG_WS}"
ZHIHANG_ENV_READY_BASHPID="${_ZHIHANG_ENV_THIS_SHELL}"

if [[ "${ZHIHANG_ENV_VERBOSE:-1}" == '1' ]]; then
  echo "[OK] ROS overlay: ${ZHIHANG_WS}/devel/setup.bash"
  echo "[OK] V6 package: ${ZHIHANG_PKG_DIR}"
  echo "[OK] validation config: ${ZHIHANG_VALIDATION_CONFIG}"
  echo "[OK] formal config: ${ZHIHANG_FORMAL_CONFIG}"
fi

_zhihang_cleanup
  return 0 2>/dev/null || exit 0
