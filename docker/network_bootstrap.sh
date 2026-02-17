#!/usr/bin/env bash
set -euo pipefail

log() {
  echo "[network-bootstrap] $*"
}

setup_central_nat() {
  if [[ "${ENABLE_CENTRAL_NAT:-0}" != "1" ]]; then
    return 0
  fi

  local bot_subnet uplink_if bot_if
  bot_subnet="${BOT_LAN_SUBNET:-192.168.50.0/24}"
  uplink_if="$(ip -4 route show default | awk '{print $5; exit}')"
  bot_if="$(ip -4 route show | awk -v target="${bot_subnet}" '$1==target {for(i=1;i<=NF;i++){if($i=="dev"){print $(i+1); exit}}}')"

  if [[ -z "${uplink_if}" || -z "${bot_if}" ]]; then
    log "Unable to detect interfaces for NAT (uplink='${uplink_if}', bot='${bot_if}')"
    return 0
  fi

  if ! command -v iptables >/dev/null 2>&1; then
    log "iptables not found; cannot enable central NAT"
    return 0
  fi

  sysctl -w net.ipv4.ip_forward=1 >/dev/null 2>&1 || true

  iptables -t nat -C POSTROUTING -s "${bot_subnet}" -o "${uplink_if}" -j MASQUERADE 2>/dev/null \
    || iptables -t nat -A POSTROUTING -s "${bot_subnet}" -o "${uplink_if}" -j MASQUERADE
  iptables -C FORWARD -i "${bot_if}" -o "${uplink_if}" -j ACCEPT 2>/dev/null \
    || iptables -A FORWARD -i "${bot_if}" -o "${uplink_if}" -j ACCEPT
  iptables -C FORWARD -i "${uplink_if}" -o "${bot_if}" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null \
    || iptables -A FORWARD -i "${uplink_if}" -o "${bot_if}" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT

  log "Enabled NAT from ${bot_subnet} (${bot_if}) to uplink interface ${uplink_if}"
}

route_bot_via_central() {
  if [[ "${ROUTE_VIA_CENTRAL:-0}" != "1" ]]; then
    return 0
  fi

  local gw dev bot_subnet
  gw="${CENTRAL_GATEWAY_IP:-}"
  bot_subnet="${BOT_LAN_SUBNET:-192.168.50.0/24}"
  if [[ -z "${gw}" ]]; then
    log "ROUTE_VIA_CENTRAL is enabled but CENTRAL_GATEWAY_IP is empty"
    return 0
  fi

  # Internal-only networks typically have no default route until we add one.
  # Detect the LAN interface from the gateway route or subnet route instead.
  dev="$(ip -4 route get "${gw}" 2>/dev/null | awk '{for(i=1;i<=NF;i++){if($i=="dev"){print $(i+1); exit}}}')"
  if [[ -z "${dev}" ]]; then
    dev="$(ip -4 route show | awk -v target="${bot_subnet}" '$1==target {for(i=1;i<=NF;i++){if($i=="dev"){print $(i+1); exit}}}')"
  fi
  if [[ -z "${dev}" ]]; then
    log "No LAN interface detected (gw=${gw}, subnet=${bot_subnet}); cannot route via central"
    return 0
  fi

  if ip -4 route show default 2>/dev/null | grep -q "via ${gw} dev ${dev}"; then
    log "Default route already via central gateway ${gw} on ${dev}"
    return 0
  fi

  if ip route replace default via "${gw}" dev "${dev}"; then
    log "Default route set via central gateway ${gw} on ${dev}"
  else
    log "Failed to set default route via ${gw} on ${dev}"
  fi
}

setup_central_nat
route_bot_via_central
