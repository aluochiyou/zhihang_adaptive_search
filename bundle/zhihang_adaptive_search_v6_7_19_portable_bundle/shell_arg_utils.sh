#!/usr/bin/env bash
# V6.7.19: normalize launcher arguments without eval.
# Handles accidental surrounding whitespace and quoted ~/... paths.

zh_trim_arg() {
  local value="${1-}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

zh_expand_user_path() {
  local value
  value="$(zh_trim_arg "${1-}")"
  case "${value}" in
    '~') value="${HOME}" ;;
    '~/'*) value="${HOME}/${value:2}" ;;
    '$HOME') value="${HOME}" ;;
    '$HOME/'*) value="${HOME}/${value:6}" ;;
    '${HOME}') value="${HOME}" ;;
    '${HOME}/'*) value="${HOME}/${value:8}" ;;
  esac
  printf '%s' "${value}"
}

zh_resolve_model_path() {
  local raw="${1-}" candidate
  candidate="$(zh_expand_user_path "${raw}")"
  if [[ -f "${candidate}" ]]; then
    readlink -f -- "${candidate}"
    return 0
  fi
  if [[ "${candidate}" != */* && -f "${HOME}/yolo_models/${candidate}" ]]; then
    readlink -f -- "${HOME}/yolo_models/${candidate}"
    return 0
  fi
  printf '%s' "${candidate}"
  return 1
}
