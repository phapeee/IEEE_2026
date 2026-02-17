#!/usr/bin/env bash
set -euo pipefail

WATCH_DIR="/home/ubuntu"
PATTERN="bundle_device-*.tar.gz"
POLL_SEC="${BUNDLE_WATCH_POLL_SEC:-2}"
STABLE_POLLS_REQUIRED="${BUNDLE_WATCH_STABLE_POLLS:-2}"

declare -A LAST_SIG
declare -A STABLE_POLLS

log() {
  echo "[bundle-autodeploy] $*"
}

cleanup_except_archive() {
  local archive_name="$1"
  find "${WATCH_DIR}" -mindepth 1 -maxdepth 1 ! -name "${archive_name}" -exec rm -rf {} +
}

run_install_all_if_present() {
  local install_script="${WATCH_DIR}/install_all.sh"
  if [[ ! -f "${install_script}" ]]; then
    return 0
  fi

  log "Refreshing APT package indexes"
  local apt_log
  apt_log="$(mktemp)"
  if apt-get update 2>&1 | tee "${apt_log}"; then
    if grep -Eq "Failed to fetch|Temporary failure resolving|Some index files failed to download" "${apt_log}"; then
      log "apt-get update completed with repository fetch errors"
    else
      log "APT package indexes refreshed"
    fi
  else
    log "apt-get update failed; install_all.sh may fail due missing indexes"
  fi
  rm -f "${apt_log}"

  log "Found install_all.sh, executing"
  if (cd "${WATCH_DIR}" && bash "${install_script}"); then
    log "install_all.sh completed successfully"
  else
    log "install_all.sh failed; watcher will continue running"
  fi
}

while true; do
  shopt -s nullglob
  archives=("${WATCH_DIR}"/${PATTERN})
  shopt -u nullglob

  if (( ${#archives[@]} == 0 )); then
    sleep "${POLL_SEC}"
    continue
  fi

  # Bundle names include timestamps, so lexicographic max is newest.
  IFS=$'\n' archives=($(printf '%s\n' "${archives[@]}" | sort))
  unset IFS
  archive="${archives[$((${#archives[@]} - 1))]}"
  [[ -f "${archive}" ]] || { sleep "${POLL_SEC}"; continue; }

  sig="$(stat -c '%s:%Y' "${archive}" 2>/dev/null || true)"
  if [[ -z "${sig}" ]]; then
    sleep "${POLL_SEC}"
    continue
  fi

  if [[ "${LAST_SIG[${archive}]:-}" == "${sig}" ]]; then
    STABLE_POLLS["${archive}"]=$(( ${STABLE_POLLS["${archive}"]:-0} + 1 ))
  else
    LAST_SIG["${archive}"]="${sig}"
    STABLE_POLLS["${archive}"]=0
  fi

  if (( ${STABLE_POLLS["${archive}"]:-0} < STABLE_POLLS_REQUIRED )); then
    sleep "${POLL_SEC}"
    continue
  fi

  if ! tar -tzf "${archive}" >/dev/null 2>&1; then
    log "Skipping invalid archive: ${archive}"
    sleep "${POLL_SEC}"
    continue
  fi

  archive_name="$(basename "${archive}")"
  log "Processing ${archive_name}"

  cleanup_except_archive "${archive_name}"

  if tar -xzf "${archive}" -C "${WATCH_DIR}"; then
    rm -f "${archive}"
    log "Extracted and removed ${archive_name}"
    run_install_all_if_present
  else
    log "Extraction failed for ${archive_name}; keeping archive for retry"
  fi

  sleep "${POLL_SEC}"
done
