#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT_DIR}/docker_env.sh"
IMAGE="${IEEE_2026_CENTRAL_IMAGE}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

if [[ ! -d "${SCRIPT_DIR}/src" ]]; then
  echo "Expected workspace at ${SCRIPT_DIR}/src" >&2
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
    if [ -d /ws/src/rmf_ros2 ]; then \
      RMF_BRANCH=\"\${RMF_REPOS_BRANCH:-\${ROS_DISTRO}}\"; \
      RMF_REPOS_URL=\"https://raw.githubusercontent.com/open-rmf/rmf/\${RMF_BRANCH}/rmf.repos\"; \
      if ! curl -fsSL \"\${RMF_REPOS_URL}\" -o /tmp/rmf.repos; then \
        echo \"RMF repos not found for '\${RMF_BRANCH}', falling back to 'main'.\" >&2; \
        curl -fsSL \"https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos\" -o /tmp/rmf.repos; \
      fi; \
      awk '\
        \$0 ~ /^  [^ ]*rmf_ros2:/ {skip=1; next} \
        skip && \$0 ~ /^  [^ ]/ {skip=0} \
        !skip {print} \
      ' /tmp/rmf.repos > /tmp/rmf.filtered.repos; \
      vcs import /ws/src --skip-existing < /tmp/rmf.filtered.repos; \
      rm -f /tmp/rmf.repos /tmp/rmf.filtered.repos; \
    fi && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
