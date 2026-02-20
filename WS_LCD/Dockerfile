# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=ros2-jazzy-base-arm64
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      libgpiod2 \
      libgpiod-dev \
      libopencv-dev \
      gpiod && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws

CMD ["bash"]
