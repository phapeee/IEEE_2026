#!/usr/bin/env bash
# Shared Docker build settings. Override by exporting before running scripts.

# Force jazzy by default (ignore host ROS_DISTRO unless explicitly overridden).
: "${ROS_DISTRO_OVERRIDE:=}"
if [[ -n "${ROS_DISTRO_OVERRIDE}" ]]; then
  ROS_DISTRO="${ROS_DISTRO_OVERRIDE}"
else
  ROS_DISTRO="jazzy"
fi

: "${PLATFORM:=linux/arm64}"

: "${BASE_IMAGE:=ros2-${ROS_DISTRO}-base-arm64}"
: "${SMACC2_IMAGE:=smacc2-ros2-${ROS_DISTRO}-arm64}"
: "${IEEE_2026_IMAGE:=ieee-2026-ros2-${ROS_DISTRO}-arm64}"
: "${IEEE_2026_CENTRAL_IMAGE:=ieee-2026-central-ros2-${ROS_DISTRO}-arm64}"
: "${ARDUCAM_IMAGE:=arducam-ros2-${ROS_DISTRO}-arm64}"
