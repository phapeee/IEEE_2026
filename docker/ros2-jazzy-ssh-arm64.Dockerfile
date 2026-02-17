# syntax=docker/dockerfile:1.6
ARG ROS_DISTRO=jazzy
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
      iproute2 \
      iptables \
      iputils-ping \
      less \
      nano \
      net-tools \
      openssh-server \
      procps \
      sudo \
    && rm -rf /var/lib/apt/lists/*

RUN if ! id -u ubuntu >/dev/null 2>&1; then \
      useradd -m -s /bin/bash ubuntu; \
    fi \
    && echo "ubuntu:ubuntu" | chpasswd \
    && if ! getent group sudo >/dev/null 2>&1; then \
      groupadd sudo; \
    fi \
    && usermod -aG sudo ubuntu \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-ubuntu \
    && chmod 0440 /etc/sudoers.d/90-ubuntu

RUN UBUNTU_HOME="$(getent passwd ubuntu | cut -d: -f6)" \
    && mkdir -p /var/run/sshd "${UBUNTU_HOME}/.ssh" \
    && chown -R ubuntu:ubuntu "${UBUNTU_HOME}/.ssh" \
    && chmod 0700 "${UBUNTU_HOME}/.ssh"

RUN sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication yes/' /etc/ssh/sshd_config \
    && sed -i 's/^#\?PubkeyAuthentication .*/PubkeyAuthentication yes/' /etc/ssh/sshd_config \
    && sed -i 's/^#\?PermitRootLogin .*/PermitRootLogin no/' /etc/ssh/sshd_config \
    && echo 'AllowUsers ubuntu' >> /etc/ssh/sshd_config \
    && echo 'UseDNS no' >> /etc/ssh/sshd_config

COPY docker/start_ros_ssh.sh /usr/local/bin/start_ros_ssh.sh
COPY docker/bundle_autodeploy.sh /usr/local/bin/bundle_autodeploy.sh
COPY docker/network_bootstrap.sh /usr/local/bin/network_bootstrap.sh
RUN UBUNTU_HOME="$(getent passwd ubuntu | cut -d: -f6)" \
    && chmod +x /usr/local/bin/start_ros_ssh.sh \
    && chmod +x /usr/local/bin/bundle_autodeploy.sh \
    && chmod +x /usr/local/bin/network_bootstrap.sh \
    && touch "${UBUNTU_HOME}/.bashrc" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${UBUNTU_HOME}/.bashrc" \
    && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}' >> "${UBUNTU_HOME}/.bashrc" \
    && chown ubuntu:ubuntu "${UBUNTU_HOME}/.bashrc"

EXPOSE 22

ENTRYPOINT ["/usr/local/bin/start_ros_ssh.sh"]
