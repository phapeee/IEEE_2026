#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/dist"
source "${SCRIPT_DIR}/docker_env.sh"

usage() {
  cat <<'USAGE'
Usage: ./build.sh [--all] [--ws <name>]...

Options:
  --all         Build all valid WS_* workspaces (see scripts/list_workspaces.sh)
  --ws <name>   Build a specific workspace (repeatable)
  -h, --help    Show this help

Defaults:
  If no flags are provided, builds all valid WS_* workspaces when present,
  otherwise defaults to the "smacc2" workspace.
USAGE
}

ALL=false
WORKSPACES=()
BUILT_WORKSPACES=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      ALL=true
      shift
      ;;
    --ws)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--ws requires a workspace name" >&2
        usage
        exit 1
      fi
      WORKSPACES+=("$1")
      shift
      ;;
    --ws=*)
      WORKSPACES+=("${1#*=}")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if $ALL && ((${#WORKSPACES[@]} > 0)); then
  echo "Use --all or --ws, not both." >&2
  exit 1
fi

list_valid_workspaces() {
  local helper="${SCRIPT_DIR}/scripts/list_workspaces.sh"
  if [[ -x "${helper}" ]]; then
    "${helper}"
    return 0
  fi

  shopt -s nullglob
  for dir in "${SCRIPT_DIR}"/WS_*; do
    [[ -d "${dir}" ]] || continue
    [[ -f "${dir}/build.sh" ]] || continue
    [[ -d "${dir}/src" ]] || continue
    basename "${dir}"
  done
}

run_workspace() {
  local ws="$1"
  local script="${SCRIPT_DIR}/${ws}/build.sh"
  if [[ ! -f "$script" ]]; then
    echo "No build.sh found for workspace: ${ws}" >&2
    exit 1
  fi
  echo "==> Building ${ws}"
  (cd "${SCRIPT_DIR}/${ws}" && bash "./build.sh")
  BUILT_WORKSPACES+=("${ws}")
}

build_base_image() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker is not installed or not on PATH" >&2
    exit 1
  fi

  if ! docker buildx inspect >/dev/null 2>&1; then
    echo "docker buildx is not available. Install or enable buildx." >&2
    exit 1
  fi

  docker run --privileged --rm tonistiigi/binfmt --install arm64 >/dev/null

  docker buildx build \
    --platform "${PLATFORM}" \
    --tag "${BASE_IMAGE}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --load \
    "${SCRIPT_DIR}"
}


if $ALL; then
  build_base_image
  mapfile -t WORKSPACES < <(list_valid_workspaces)
  if (( ${#WORKSPACES[@]} == 0 )); then
    echo "No valid WS_* workspaces found." >&2
    exit 1
  fi
  for ws in "${WORKSPACES[@]}"; do
    run_workspace "${ws}"
  done
  exit 0
fi

if (( ${#WORKSPACES[@]} == 0 )); then
  mapfile -t WORKSPACES < <(list_valid_workspaces)
  if (( ${#WORKSPACES[@]} == 0 )); then
    WORKSPACES=("smacc2")
  fi
fi

build_base_image
for ws in "${WORKSPACES[@]}"; do
  run_workspace "$ws"
done
