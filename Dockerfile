# Dev container for ROS 2 Humble + Nav2 + RMF + SMACC2 + ros2_tracing + tools
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=42

# Basic tools & build deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim nano \
    build-essential cmake gdb lcov \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    # ros2_tracing prerequisites (LTTng) :contentReference[oaicite:3]{index=3}
    lttng-tools liblttng-ust-dev python3-lttng python3-babeltrace babeltrace \
    # Convenience tools
    tmux htop iproute2 net-tools \
    && rm -rf /var/lib/apt/lists/*

# Core navigation + localization + control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# LIDAR + RGB pipelines, RViz2, rosbag2 (many are already in desktop-full)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-tf2-sensor-msgs \
    ros-humble-depthimage-to-laserscan \
    ros-humble-image-pipeline \
    ros-humble-image-tools \
    ros-humble-rviz2 \
    ros-humble-rosbag2* \
    && rm -rf /var/lib/apt/lists/*

# Open-RMF minimal dev set (aggregated via rmf_dev) :contentReference[oaicite:4]{index=4}
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmf-dev \
    && rm -rf /var/lib/apt/lists/*

# SMACC2 (behavioral state machines) :contentReference[oaicite:5]{index=5}
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-smacc2 \
    ros-humble-smacc2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Diagnostics, rosbridge (for Foxglove / web UIs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator \
    ros-humble-rosbridge-server \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install ipython ipykernel numpy scipy matplotlib black isort pytest

# ros2_tracing overlay workspace :contentReference[oaicite:7]{index=7}
ENV COLCON_WS=/ws
RUN mkdir -p $COLCON_WS/src
WORKDIR $COLCON_WS

# Clone ros2_tracing (humble branch)
RUN git clone --branch humble https://gitlab.com/ros-tracing/ros2_tracing.git src/ros2_tracing

# Build ros2_tracing overlay
RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install"

RUN /bin/bash -c "printf '%s\n' '' '# ROS 2 environment' \
    'if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then' \
    '  source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'fi' \
    'if [ -f /ws/install/setup.bash ]; then' \
    '  source /ws/install/setup.bash' \
    'fi' >> /root/.bashrc"

ENV ROS_WS=$COLCON_WS
WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["tail","-f","/dev/null"]
