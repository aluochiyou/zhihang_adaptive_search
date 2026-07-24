#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"

ZHIHANG_V6_TERMINAL_STATUS_DIR="${ZHIHANG_V6_TERMINAL_STATUS_DIR:-${HOME}/.cache/zhihang_v6_7_19_terminal_status}"
export ZHIHANG_V6_TERMINAL_STATUS_DIR
mkdir -p "${ZHIHANG_V6_TERMINAL_STATUS_DIR}"

terminal_slug() {
  printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9]+/_/g; s/^_+//; s/_+$//'
}

terminal_rc_file() {
  local slug
  slug="$(terminal_slug "$1")"
  printf '%s/%s.rc\n' "${ZHIHANG_V6_TERMINAL_STATUS_DIR}" "${slug}"
}

resolve_terminal_backend() {
  local requested="${ZHIHANG_TERMINAL_BACKEND:-auto}"
  if [[ "${requested}" != "auto" ]]; then
    printf '%s\n' "${requested}"
    return 0
  fi
  if [[ -n "${DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null 2>&1; then
    echo gnome-terminal
  elif [[ -n "${DISPLAY:-}" ]] && command -v xterm >/dev/null 2>&1; then
    echo xterm
  elif [[ -n "${DISPLAY:-}" ]] && command -v konsole >/dev/null 2>&1; then
    echo konsole
  elif command -v tmux >/dev/null 2>&1; then
    echo tmux
  else
    echo none
  fi
}

_build_terminal_wrapper() {
  local title="$1" script="$2"; shift 2
  local slug rcfile wrapper arg
  slug="$(terminal_slug "${title}")"
  rcfile="${ZHIHANG_V6_TERMINAL_STATUS_DIR}/${slug}.rc"
  wrapper="${ZHIHANG_V6_TERMINAL_STATUS_DIR}/${slug}_wrapper.sh"
  rm -f "${rcfile}" "${wrapper}"

  {
    echo '#!/usr/bin/env bash'
    echo 'set +e'
    printf 'title=%q\n' "${title}"
    printf 'rcfile=%q\n' "${rcfile}"
    cat <<'WRAPPER_ENV'
dedupe_colon_var() {
  local name="$1" value part output=""
  local -A seen=()
  value="${!name-}"
  IFS=':' read -r -a parts <<<"${value}"
  for part in "${parts[@]}"; do
    [[ -n "${part}" ]] || continue
    [[ -n "${seen[${part}]+x}" ]] && continue
    seen["${part}"]=1
    if [[ -z "${output}" ]]; then output="${part}"; else output="${output}:${part}"; fi
  done
  printf -v "${name}" '%s' "${output}"
  export "${name}"
}
for _v in GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH LD_LIBRARY_PATH ROS_PACKAGE_PATH PYTHONPATH; do
  dedupe_colon_var "${_v}"
done
unset _v

unset ZHIHANG_ENV_READY ZHIHANG_ENV_READY_WS ZHIHANG_ENV_READY_BASHPID
unset ZHIHANG_PKG_DIR ZHIHANG_FORMAL_CONFIG ZHIHANG_VALIDATION_CONFIG
WRAPPER_ENV
    if [[ -n "${ZHIHANG_PROFILE_FILE:-}" ]]; then
      printf 'export ZHIHANG_PROFILE_FILE=%q\n' "${ZHIHANG_PROFILE_FILE}"
    fi
    printf 'export ZHIHANG_WS=%q\n' "${ZHIHANG_WS}"
    printf 'bash %q' "${script}"
    for arg in "$@"; do printf ' %q' "${arg}"; done
    printf '\n'
    cat <<'WRAPPER_EOF'
rc=$?
printf '%s\n' "${rc}" > "${rcfile}"
echo
echo "[TERMINAL EXIT] ${title} rc=${rc}"
if [[ "${rc}" == 124 ]]; then
  echo '[DIAGNOSIS] rc=124 means a timeout command terminated the child process.'
fi
echo '[INFO] Terminal is kept open for inspection.'
if [[ "${ZHIHANG_TERMINAL_KEEP_OPEN:-1}" == "1" ]]; then
  exec bash --noprofile --norc -i
fi
exit "${rc}"
WRAPPER_EOF
  } > "${wrapper}"
  chmod +x "${wrapper}"
  printf '%s\n' "${wrapper}"
}

open_named_terminal() {
  local title="$1" geometry="$2" script="$3"; shift 3
  local backend wrapper slug
  backend="$(resolve_terminal_backend)"
  wrapper="$(_build_terminal_wrapper "${title}" "${script}" "$@")"
  slug="$(terminal_slug "${title}")"

  case "${backend}" in
    gnome-terminal)
      gnome-terminal --window --title="${title}" --geometry="${geometry}" -- \
        bash -i "${wrapper}" >/dev/null 2>&1 &
      ;;
    xterm)
      xterm -T "${title}" -geometry "${geometry}" -e bash -i "${wrapper}" \
        >/dev/null 2>&1 &
      ;;
    konsole)
      konsole --new-tab -p "tabtitle=${title}" -e bash -i "${wrapper}" \
        >/dev/null 2>&1 &
      ;;
    tmux)
      command -v tmux >/dev/null 2>&1 || {
        echo '[ERROR] tmux backend selected but tmux is not installed' >&2
        return 1
      }
      if ! tmux has-session -t "${ZHIHANG_TMUX_SESSION}" 2>/dev/null; then
        tmux new-session -d -s "${ZHIHANG_TMUX_SESSION}" -n controller
      fi
      tmux new-window -t "${ZHIHANG_TMUX_SESSION}" -n "${slug:0:20}" \
        "bash -i $(printf '%q' "${wrapper}")"
      echo "[TMUX] ${title}: tmux attach -t ${ZHIHANG_TMUX_SESSION}"
      ;;
    *)
      echo '[ERROR] no supported terminal backend is available.' >&2
      echo '[HINT] install gnome-terminal/xterm/konsole, or install tmux.' >&2
      return 1
      ;;
  esac
}

terminal_exit_code() {
  local rcfile
  rcfile="$(terminal_rc_file "$1")"
  [[ -s "${rcfile}" ]] || return 1
  tr -d '[:space:]' < "${rcfile}"
}

assert_terminal_running() {
  local title="$1" rc
  if rc="$(terminal_exit_code "${title}")"; then
    echo "[ERROR] ${title} exited during startup; rc=${rc}" >&2
    exit 1
  fi
}

assert_terminal_running_or_clean_exit() {
  local title="$1" rc
  if rc="$(terminal_exit_code "${title}")"; then
    if [[ "${rc}" == 0 ]]; then
      echo "[OK] ${title} initializer returned rc=0; background service readiness will be verified"
      return 0
    fi
    echo "[ERROR] ${title} exited during startup; rc=${rc}" >&2
    exit 1
  fi
}

require_desktop_tools() {
  local backend
  backend="$(resolve_terminal_backend)"
  case "${backend}" in
    gnome-terminal|xterm|konsole)
      [[ -n "${DISPLAY:-}" ]] || {
        echo "[ERROR] ${backend} requires DISPLAY; use tmux or run from the Ubuntu desktop." >&2
        exit 1
      }
      ;;
    tmux)
      command -v tmux >/dev/null 2>&1 || {
        echo '[ERROR] tmux not found' >&2
        exit 1
      }
      ;;
    *)
      echo '[ERROR] no supported terminal backend found' >&2
      exit 1
      ;;
  esac
  echo "[OK] terminal backend: ${backend}"
}
