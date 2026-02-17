# syntax=docker/dockerfile:1.6
ARG ROS_DISTRO=jazzy
FROM --platform=$TARGETPLATFORM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /ws

CMD ["bash"]
