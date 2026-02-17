#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT_DIR}/docker_env.sh"
IMAGE="${SMACC2_IMAGE}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

if [[ ! -d "${SCRIPT_DIR}/src/SMACC2" ]]; then
  echo "Expected workspace at ${SCRIPT_DIR}/src/SMACC2" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH" >&2
  exit 1
fi

if ! docker buildx inspect >/dev/null 2>&1; then
  echo "docker buildx is not available. Install or enable buildx." >&2
  exit 1
fi

docker run --privileged --rm tonistiigi/binfmt --install arm64 >/dev/null

if ! docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1; then
  docker buildx build \
    --platform "${PLATFORM}" \
    --tag "${BASE_IMAGE}" \
    --file "${ROOT_DIR}/Dockerfile" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --load \
    "${ROOT_DIR}"
fi

docker buildx build \
  --platform "${PLATFORM}" \
  --tag "${IMAGE}" \
  --file "${DOCKERFILE}" \
  --build-arg BASE_IMAGE="${BASE_IMAGE}" \
  --load \
  "${SCRIPT_DIR}"

docker run --rm -it \
  --platform "${PLATFORM}" \
  --volume "${SCRIPT_DIR}:/ws" \
  --workdir /ws \
  "${IMAGE}" \
  bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release"
