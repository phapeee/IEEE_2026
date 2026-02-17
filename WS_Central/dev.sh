#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT_DIR}/docker_env.sh"

IMAGE="${IEEE_2026_CENTRAL_IMAGE}"
PLATFORM="${PLATFORM}"
CONTAINER_NAME="ieee_2026_central_dev_${ROS_DISTRO}"
SMACC2_DIR="${SMACC2_DIR:-${ROOT_DIR}/smacc2}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

usage() {
  cat <<'USAGE'
Usage: ./dev.sh [--reset]

Options:
  --reset   Remove any existing dev container and start fresh.
USAGE
}

RESET=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --reset)
      RESET=true
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

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH" >&2
  exit 1
fi

if ! docker buildx inspect >/dev/null 2>&1; then
  echo "docker buildx is not available. Install or enable buildx." >&2
  exit 1
fi

docker run --privileged --rm tonistiigi/binfmt --install arm64 >/dev/null

if $RESET; then
  if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi
fi

if ! docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1; then
  docker buildx build \
    --platform "${PLATFORM}" \
    --tag "${BASE_IMAGE}" \
    --file "${ROOT_DIR}/Dockerfile" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --load \
    "${ROOT_DIR}"
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker buildx build \
    --platform "${PLATFORM}" \
    --tag "${IMAGE}" \
    --file "${DOCKERFILE}" \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    --load \
    "${SCRIPT_DIR}"
fi

if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  docker start -ai "${CONTAINER_NAME}"
  exit 0
fi

VOLUME_ARGS=(
  "--volume" "${SCRIPT_DIR}:/ws"
)

if [[ -d "${SMACC2_DIR}" ]]; then
  VOLUME_ARGS+=("--volume" "${SMACC2_DIR}:/ws_smacc2:ro")
fi

docker run -it \
  --name "${CONTAINER_NAME}" \
  --platform "${PLATFORM}" \
  "${VOLUME_ARGS[@]}" \
  --workdir /ws \
  "${IMAGE}" \
  bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -f /ws_smacc2/install/setup.bash ]; then \
      source /ws_smacc2/install/setup.bash; \
    fi; \
    exec bash"
