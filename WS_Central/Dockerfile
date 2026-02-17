# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=ros2-jazzy-base-arm64

FROM busybox AS local_package_manifests
WORKDIR /tmp/ws
# Keep only package manifests so dependency resolution is invalidated
# only when package.xml changes.
COPY src ./src
RUN rm -rf src/demonstrations
RUN find src -type f ! -name package.xml -delete

FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

ARG RMF_REPOS_BRANCH

WORKDIR /tmp/ws

# Keep toolchain packages in a stable layer so local source edits do not force reinstall.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      curl \
      python3-colcon-mixin \
      python3-pip \
      python3-vcstool \
      ros-dev-tools

# Fetch external RMF sources once; this only re-runs if base image or branch changes.
RUN RMF_BRANCH="${RMF_REPOS_BRANCH:-${ROS_DISTRO}}" && \
    mkdir -p src && \
    RMF_REPOS_URL="https://raw.githubusercontent.com/open-rmf/rmf/${RMF_BRANCH}/rmf.repos" && \
    if ! curl -fsSL "${RMF_REPOS_URL}" -o /tmp/rmf.repos; then \
      echo "RMF repos not found for '${RMF_BRANCH}', falling back to 'main'." >&2; \
      curl -fsSL "https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos" -o /tmp/rmf.repos; \
    fi && \
    awk '\
      $0 ~ /^  [^ ]*rmf_ros2:/ {skip=1; next} \
      skip && $0 ~ /^  [^ ]/ {skip=0} \
      !skip {print} \
    ' /tmp/rmf.repos > /tmp/rmf.filtered.repos && \
    vcs import src --skip-existing < /tmp/rmf.filtered.repos && \
    rm -rf src/demonstrations && \
    rm -f /tmp/rmf.repos /tmp/rmf.filtered.repos

COPY --from=local_package_manifests /tmp/ws/src ./src

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/root/.ros/rosdep,sharing=locked \
    apt-get update && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y -r --rosdistro "${ROS_DISTRO}" \
      --skip-keys "ament_python"

# Copy full source after rosdep so regular code edits do not force reinstall.
COPY src ./src
RUN rm -rf src/demonstrations

WORKDIR /ws

# Keep container running for long-lived interactive dev sessions.
CMD ["bash", "-lc", "trap : TERM INT; sleep infinity & wait"]
