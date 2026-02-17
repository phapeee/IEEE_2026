#!/usr/bin/env bash
set -euo pipefail

mkdir -p /var/run/sshd
ssh-keygen -A

if [[ -n "${AUTHORIZED_KEYS:-}" ]]; then
  UBUNTU_HOME="$(getent passwd ubuntu | cut -d: -f6)"
  install -d -m 0700 -o ubuntu -g ubuntu "${UBUNTU_HOME}/.ssh"
  printf '%s\n' "${AUTHORIZED_KEYS}" > "${UBUNTU_HOME}/.ssh/authorized_keys"
  chown ubuntu:ubuntu "${UBUNTU_HOME}/.ssh/authorized_keys"
  chmod 0600 "${UBUNTU_HOME}/.ssh/authorized_keys"
fi

if [[ -n "${SSH_ALLOW_FROM:-}" ]]; then
  install -d -m 0755 /etc/ssh/sshd_config.d
  echo "AllowUsers ubuntu@${SSH_ALLOW_FROM}" > /etc/ssh/sshd_config.d/90-allow-from.conf
fi

/usr/local/bin/network_bootstrap.sh || true
/usr/local/bin/bundle_autodeploy.sh &

exec /usr/sbin/sshd -D -e
