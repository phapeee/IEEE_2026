#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/dist"

usage() {
  cat <<'USAGE'
Usage: ./package.sh [--all] [--ws <name>]... [--device <name>]

Options:
  --all         Package all valid WS_* workspaces (see scripts/list_workspaces.sh)
  --ws <name>   Package a specific workspace (repeatable)
  -d, --device <name>  Package the workspaces defined for a device in package.conf (auto-ship to the same host)
  --ship-path <path>  Remote path for auto-ship (default: ~/)
  --ssh-config <path> Path to ssh config (default: ./package.conf ssh_config or ~/.ssh/config)
  -h, --help    Show this help
USAGE
}

ALL=false
WORKSPACES=()
DEVICE=""
SHIP_HOST=""
SHIP_PATH=""
SSH_CONFIG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      ALL=true
      shift
      ;;
    --ws)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--ws requires a workspace name" >&2
        usage
        exit 1
      fi
      WORKSPACES+=("$1")
      shift
      ;;
    --ws=*)
      WORKSPACES+=("${1#*=}")
      shift
      ;;
    -d|--device)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--device requires a device name" >&2
        usage
        exit 1
      fi
      DEVICE="$1"
      shift
      ;;
    --device=*)
      DEVICE="${1#*=}"
      shift
      ;;
    --ship-path)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--ship-path requires a remote path" >&2
        usage
        exit 1
      fi
      SHIP_PATH="$1"
      shift
      ;;
    --ssh-config)
      shift
      if [[ $# -eq 0 ]]; then
        echo "--ssh-config requires a path" >&2
        usage
        exit 1
      fi
      SSH_CONFIG="$1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

list_valid_workspaces() {
  local helper="${SCRIPT_DIR}/scripts/list_workspaces.sh"
  if [[ -x "${helper}" ]]; then
    "${helper}"
    return 0
  fi

  shopt -s nullglob
  for dir in "${SCRIPT_DIR}"/WS_*; do
    [[ -d "${dir}" ]] || continue
    [[ -f "${dir}/build.sh" ]] || continue
    [[ -d "${dir}/src" ]] || continue
    basename "${dir}"
  done
}

list_devices() {
  local conf="${SCRIPT_DIR}/package.conf"
  if [[ ! -f "${conf}" ]]; then
    return 0
  fi
  awk '
    BEGIN {in_devices=0}
    /^devices:[[:space:]]*$/ {in_devices=1; next}
    in_devices {
      if ($0 ~ /^[^[:space:]]/) {exit}
      if ($0 ~ /^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$/) {
        name=$0
        sub(/^[[:space:]][[:space:]]/, "", name)
        sub(/:[[:space:]]*$/, "", name)
        print name
      }
    }
  ' "${conf}"
}

load_device_workspaces() {
  local device="$1"
  local conf="${SCRIPT_DIR}/package.conf"
  if [[ ! -f "${conf}" ]]; then
    echo "package.conf not found at ${conf}; required for --device." >&2
    exit 1
  fi

  mapfile -t WORKSPACES < <(awk -v target="${device}" '
    BEGIN {in_devices=0; in_target=0}
    /^[[:space:]]*#/ {next}
    /^devices:[[:space:]]*$/ {in_devices=1; next}
    in_devices {
      if ($0 ~ /^[^[:space:]]/) {exit}
      if ($0 ~ /^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$/) {
        name=$0
        sub(/^[[:space:]][[:space:]]/, "", name)
        sub(/:[[:space:]]*$/, "", name)
        in_target=(name==target)
        next
      }
      if (in_target && $0 ~ /^[[:space:]][[:space:]][[:space:]][[:space:]]-[[:space:]]+/) {
        item=$0
        sub(/^[[:space:]][[:space:]][[:space:]][[:space:]]-[[:space:]]+/, "", item)
        if (item != "") print item
      }
    }
  ' "${conf}")

  if (( ${#WORKSPACES[@]} == 0 )); then
    echo "No workspaces found for device '${device}' in ${conf}." >&2
    local devices
    devices="$(list_devices | paste -sd ',' - | sed 's/,/, /g')"
    if [[ -n "${devices}" ]]; then
      echo "Available devices: ${devices}" >&2
    fi
    exit 1
  fi
}

slugify() {
  local input="$1"
  local slug
  slug="$(printf '%s' "${input}" | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9._-' '-' | sed 's/^-*//; s/-*$//')"
  printf '%s' "${slug}"
}

stage_workspace() {
  local ws="$1"
  local ws_dir="${SCRIPT_DIR}/${ws}"
  local out_dir="${DIST_DIR}/${ws}"

  case "${ws}" in
    WS_*)
      if [[ ! -f "${ws_dir}/build.sh" ]]; then
        echo "Expected build.sh at ${ws_dir}/build.sh" >&2
        exit 1
      fi
      if [[ ! -d "${ws_dir}/src" ]]; then
        echo "Expected workspace at ${ws_dir}/src" >&2
        exit 1
      fi
      rm -rf "${out_dir}"
      mkdir -p "${out_dir}"
      if [[ -d "${ws_dir}/install" ]]; then
        cp -a "${ws_dir}/install" "${out_dir}/"
      fi
      if [[ -d "${ws_dir}/config" ]]; then
        cp -a "${ws_dir}/config" "${out_dir}/"
      fi
      if [[ -d "${ws_dir}/scripts" ]]; then
        cp -a "${ws_dir}/scripts" "${out_dir}/"
      fi
      cp -a "${ws_dir}/src" "${out_dir}/"

      cat <<'SCRIPT' > "${out_dir}/install_deps.sh"
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if ! command -v rosdep >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y python3-rosdep
fi

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --ignore-src --from-paths "${SCRIPT_DIR}/src" -y -r --rosdistro "${ROS_DISTRO}" \
  --skip-keys "ament_python"
SCRIPT
      if [[ -d "${ws_dir}/src/arducam_rclpy_tof_pointcloud" ]]; then
        cat <<'SCRIPT' >> "${out_dir}/install_deps.sh"

# Arducam ToF runtime dependencies (SDK + Python bindings).
sudo apt-get update
sudo apt-get install -y \
  ca-certificates \
  cmake \
  curl \
  libopencv-dev \
  lsb-release \
  python3-numpy \
  python3-opencv \
  python3-pip

if [ $(dpkg -l | grep -c arducam-tof-sdk-dev) -lt 1 ]; then
  echo "Add Arducam_ppa repositories."
  if [ "$(lsb_release -r | awk '{print $2}' | cut -d. -f1)" -ge 13 ]; then
    sudo rm -f /etc/apt/sources.list.d/arducam_list_files.list
    sudo curl -s --compressed -o /usr/share/keyrings/arducam-keyring.pgp "https://arducam.github.io/arducam_ppa/KEY.gpg"
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.sources "https://arducam.github.io/arducam_ppa/arducam_list_files.sources"
  else
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  fi
fi

sudo apt-get update
sudo apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev

if ! sudo python3 -m pip install ArducamDepthCamera >/dev/null 2>&1; then
  sudo python3 -m pip install ArducamDepthCamera --break-system-packages >/dev/null 2>&1 || true
fi
SCRIPT
      fi
      if [[ "${ws}" == "WS_Mini_Bot" ]]; then
        cat <<'SCRIPT' >> "${out_dir}/install_deps.sh"

BOT_NAME="${BOT_NAME:?BOT_NAME environment variable must be set}"

for dir in "${SCRIPT_DIR}/config" "${SCRIPT_DIR}/install" "${SCRIPT_DIR}/scripts" "${SCRIPT_DIR}/src"; do
  [[ -d "${dir}" ]] || continue
  while IFS= read -r -d '' file; do
    BOT_NAME="${BOT_NAME}" perl -i -pe 's/\bbot_name\b/$ENV{BOT_NAME}/g' "${file}"
  done < <(grep -RIlZ --binary-files=without-match -w -- 'bot_name' "${dir}" || true)
done

# Keep ros2_control runtime libraries aligned with the packaged WS_Mini_Bot binaries.
sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-controller-interface" \
  "ros-${ROS_DISTRO}-controller-manager" \
  "ros-${ROS_DISTRO}-controller-manager-msgs" \
  "ros-${ROS_DISTRO}-hardware-interface" \
  "ros-${ROS_DISTRO}-mecanum-drive-controller" \
  "ros-${ROS_DISTRO}-pluginlib" \
  "ros-${ROS_DISTRO}-ros2-control" \
  "ros-${ROS_DISTRO}-ros2-controllers"

if ! command -v nm >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y binutils
fi

HARDWARE_INTERFACE_LIB="/opt/ros/${ROS_DISTRO}/lib/libhardware_interface.so"
REQUIRED_SYMBOL="_ZTIN18hardware_interface26HardwareComponentInterfaceE"
CONTROLLER_INTERFACE_LIB="/opt/ros/${ROS_DISTRO}/lib/libcontroller_interface.so"
CONTROLLER_INTERFACE_REQUIRED_SYMBOL="_ZNK20controller_interface23ControllerInterfaceBase16get_lifecycle_idEv"
CONTROLLER_MANAGER_MSGS_LIB="/opt/ros/${ROS_DISTRO}/lib/libcontroller_manager_msgs__rosidl_typesupport_cpp.so"
CONTROLLER_MANAGER_REQUIRED_SYMBOL="_ZN22rosidl_typesupport_cpp31get_service_type_support_handleIN23controller_manager_msgs3srv17CleanupControllerEEEPK29rosidl_service_type_support_tv"
I2C_PLUGIN_LIB="${SCRIPT_DIR}/install/lib/libi2c_velocity_hardware.so"

if [[ ! -f "${HARDWARE_INTERFACE_LIB}" ]]; then
  echo "[ERROR] Missing ${HARDWARE_INTERFACE_LIB}" >&2
  exit 1
fi

if [[ ! -f "${CONTROLLER_MANAGER_MSGS_LIB}" ]]; then
  echo "[ERROR] Missing ${CONTROLLER_MANAGER_MSGS_LIB}" >&2
  exit 1
fi

if [[ ! -f "${CONTROLLER_INTERFACE_LIB}" ]]; then
  echo "[ERROR] Missing ${CONTROLLER_INTERFACE_LIB}" >&2
  exit 1
fi

if ! nm -D "${HARDWARE_INTERFACE_LIB}" | grep -q "${REQUIRED_SYMBOL}"; then
  echo "[ERROR] Incompatible ${HARDWARE_INTERFACE_LIB} (missing ${REQUIRED_SYMBOL})." >&2
  echo "[ERROR] ros2_control runtime on this target cannot load i2c_velocity_hardware." >&2
  exit 1
fi

if ! nm -D "${CONTROLLER_INTERFACE_LIB}" | grep -q "${CONTROLLER_INTERFACE_REQUIRED_SYMBOL}"; then
  echo "[ERROR] Incompatible ${CONTROLLER_INTERFACE_LIB} (missing get_lifecycle_id)." >&2
  echo "[ERROR] controller_interface is out of sync with the installed controller stack." >&2
  exit 1
fi

if ! nm -D "${CONTROLLER_MANAGER_MSGS_LIB}" | grep -q "${CONTROLLER_MANAGER_REQUIRED_SYMBOL}"; then
  echo "[ERROR] Incompatible ${CONTROLLER_MANAGER_MSGS_LIB} (missing CleanupController type support)." >&2
  echo "[ERROR] controller_manager and controller_manager_msgs packages are out of sync." >&2
  exit 1
fi

if [[ -f "${I2C_PLUGIN_LIB}" ]]; then
  if ! LD_LIBRARY_PATH="${SCRIPT_DIR}/install/lib:/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH:-}" \
    ldd -r "${I2C_PLUGIN_LIB}" >/dev/null 2>&1; then
    echo "[ERROR] ${I2C_PLUGIN_LIB} failed dynamic linker checks (ldd -r)." >&2
    echo "[ERROR] Check target ROS packages and rerun install_all.sh." >&2
    exit 1
  fi
fi
SCRIPT
      fi
      cat <<'SCRIPT' >> "${out_dir}/install_deps.sh"

if [[ -d "${SCRIPT_DIR}/src" ]]; then
  rm -rf "${SCRIPT_DIR}/src"
fi
SCRIPT
      chmod +x "${out_dir}/install_deps.sh"
      ;;
    smacc2)
      if [[ ! -d "${ws_dir}/src/SMACC2" ]]; then
        echo "Expected workspace at ${ws_dir}/src/SMACC2" >&2
        exit 1
      fi
      rm -rf "${out_dir}"
      mkdir -p "${out_dir}"
      if [[ -d "${ws_dir}/install" ]]; then
        cp -a "${ws_dir}/install" "${out_dir}/"
      fi
      cp -a "${ws_dir}/src" "${out_dir}/"

      cat <<'SCRIPT' > "${out_dir}/install_deps.sh"
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if ! command -v rosdep >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y python3-rosdep
fi

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --ignore-src --from-paths "${SCRIPT_DIR}/src" -y -r --rosdistro "${ROS_DISTRO}"

if [[ -d "${SCRIPT_DIR}/src" ]]; then
  rm -rf "${SCRIPT_DIR}/src"
fi
SCRIPT
      chmod +x "${out_dir}/install_deps.sh"
      ;;
    Arducam_tof_camera)
      if [[ ! -d "${ws_dir}" ]]; then
        echo "Expected workspace at ${ws_dir}" >&2
        exit 1
      fi
      rm -rf "${out_dir}"
      mkdir -p "${out_dir}"
      if [[ -d "${ws_dir}/ros2_publisher/install" ]]; then
        mkdir -p "${out_dir}/ros2_publisher"
        cp -a "${ws_dir}/ros2_publisher/install" "${out_dir}/ros2_publisher/"
      fi

      cat <<'SCRIPT' > "${out_dir}/install_deps.sh"
#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  ca-certificates \
  cmake \
  curl \
  libopencv-dev \
  lsb-release \
  python3-numpy \
  python3-opencv \
  python3-pip

if [ $(dpkg -l | grep -c arducam-tof-sdk-dev) -lt 1 ]; then
  echo "Add Arducam_ppa repositories."
  if [ "$(lsb_release -r | awk '{print $2}' | cut -d. -f1)" -ge 13 ]; then
    sudo rm -f /etc/apt/sources.list.d/arducam_list_files.list
    sudo curl -s --compressed -o /usr/share/keyrings/arducam-keyring.pgp "https://arducam.github.io/arducam_ppa/KEY.gpg"
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.sources "https://arducam.github.io/arducam_ppa/arducam_list_files.sources"
  else
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list"
  fi
fi

sudo apt-get update
sudo apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev

if ! sudo python3 -m pip install ArducamDepthCamera >/dev/null 2>&1; then
  sudo python3 -m pip install ArducamDepthCamera --break-system-packages >/dev/null 2>&1 || true
fi

if [[ -d "${SCRIPT_DIR}/src" ]]; then
  rm -rf "${SCRIPT_DIR}/src"
fi
SCRIPT
      chmod +x "${out_dir}/install_deps.sh"
      ;;
    *)
      if [[ -d "${out_dir}" ]]; then
        echo "==> Using existing dist for ${ws}"
      else
        echo "No packaging rule for workspace: ${ws}" >&2
        echo "Add a packaging rule in package.sh or pre-stage dist/${ws}." >&2
        exit 1
      fi
      ;;
  esac
}

create_setup_script() {
  local dest="$1"
  shift

  cat <<'SCRIPT' > "${dest}"
#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced, e.g.:"
  echo "  source ${BASH_SOURCE[0]}"
  exit 0
fi

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

workspaces=(
SCRIPT

  local ws
  for ws in "$@"; do
    printf '  %q\n' "${ws}" >> "${dest}"
  done

  cat <<'SCRIPT' >> "${dest}"
)

for ws in "${workspaces[@]}"; do
  if [[ -f "${SCRIPT_DIR}/${ws}/install/local_setup.bash" ]]; then
    source "${SCRIPT_DIR}/${ws}/install/local_setup.bash"
  elif [[ -f "${SCRIPT_DIR}/${ws}/install/setup.bash" ]]; then
    source "${SCRIPT_DIR}/${ws}/install/setup.bash"
  fi
done
SCRIPT

  chmod +x "${dest}"
}

if [[ -n "${DEVICE}" ]] && { $ALL || ((${#WORKSPACES[@]} > 0)); }; then
  echo "Use --device or --all/--ws, not both." >&2
  exit 1
fi

if $ALL && ((${#WORKSPACES[@]} > 0)); then
  echo "Use --all or --ws, not both." >&2
  exit 1
fi

if [[ -n "${DEVICE}" ]]; then
  load_device_workspaces "${DEVICE}"
  if [[ -z "${SHIP_HOST}" ]]; then
    SHIP_HOST="${DEVICE}"
  fi
fi

if $ALL; then
  mapfile -t WORKSPACES < <(list_valid_workspaces)
  if (( ${#WORKSPACES[@]} == 0 )); then
    echo "No valid WS_* workspaces found." >&2
    exit 1
  fi
fi

if (( ${#WORKSPACES[@]} == 0 )); then
  echo "No workspaces specified." >&2
  usage
  exit 1
fi

bundle_dir="${DIST_DIR}/bundle"
ts="$(date +%Y%m%d_%H%M%S)"
bundle_label=""
if [[ -n "${DEVICE}" ]]; then
  bundle_label="device-${DEVICE}"
elif $ALL; then
  bundle_label="all"
elif (( ${#WORKSPACES[@]} > 0 )); then
  bundle_label="$(IFS=+; echo "${WORKSPACES[*]}")"
fi
bundle_slug="$(slugify "${bundle_label}")"
if [[ -n "${bundle_slug}" ]]; then
  archive="${DIST_DIR}/bundle_${bundle_slug}_${ts}.tar.gz"
else
  archive="${DIST_DIR}/bundle_${ts}.tar.gz"
fi

mkdir -p "${DIST_DIR}"
for ws in "${WORKSPACES[@]}"; do
  stage_workspace "${ws}"
done

rm -rf "${bundle_dir}"
mkdir -p "${bundle_dir}"

cat <<'SCRIPT' > "${bundle_dir}/install_all.sh"
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GPIO_CHIP_NAME="gpiochip4"
GPIO_CHIP_DEV="/dev/${GPIO_CHIP_NAME}"
GPIO_LINE="12"

set_gpio_line() {
  local value="$1"

  [[ -e "${GPIO_CHIP_DEV}" ]] || return 0

  if ! command -v gpioset >/dev/null 2>&1; then
    echo "[WARN] ${GPIO_CHIP_DEV} present but gpioset is not installed; skipping GPIO ${GPIO_LINE}=${value}" >&2
    return 0
  fi

  if ! sudo gpioset "${GPIO_CHIP_NAME}" "${GPIO_LINE}=${value}"; then
    echo "[WARN] Failed to set ${GPIO_CHIP_NAME} line ${GPIO_LINE}=${value} with sudo gpioset" >&2
  fi
}

set_gpio_line 1

for ws_dir in "${SCRIPT_DIR}"/*; do
  if [[ -f "${ws_dir}/install_deps.sh" ]]; then
    echo "==> Installing deps for $(basename "${ws_dir}")"
    (cd "${ws_dir}" && bash "./install_deps.sh")
  fi
done

set_gpio_line 0
SCRIPT
chmod +x "${bundle_dir}/install_all.sh"

for ws in "${WORKSPACES[@]}"; do
  ws_dir="${DIST_DIR}/${ws}"
  if [[ ! -d "${ws_dir}" ]]; then
    echo "Missing packaged workspace at ${ws_dir}" >&2
    exit 1
  fi
  cp -a "${ws_dir}" "${bundle_dir}/"
done

create_setup_script "${bundle_dir}/setup.sh" "${WORKSPACES[@]}"
cp -a "${bundle_dir}/setup.sh" "${bundle_dir}/setup.bash"

tar -C "${bundle_dir}" -czf "${archive}" .
echo "==> Bundle created: ${archive}"

if [[ -n "${SHIP_HOST}" ]]; then
  SSH_CONFIG_TMP=""
  if [[ -z "${SSH_CONFIG}" ]]; then
    if [[ -f "${SCRIPT_DIR}/package.conf" ]]; then
      SSH_CONFIG_TMP="$(mktemp)"
      awk '
        function ltrim(s){sub(/^[[:space:]]+/, "", s); return s}
        function rtrim(s){sub(/[[:space:]]+$/, "", s); return s}
        function trim(s){return rtrim(ltrim(s))}
        BEGIN {in_block=0; mode=""; first=1}
        /^ssh_config:[[:space:]]*\|/ {in_block=1; mode="block"; next}
        /^ssh_config:[[:space:]]*$/ {in_block=1; mode="map"; next}
        in_block {
          if ($0 ~ /^[^[:space:]]/) {exit}
          if (mode=="block") {
            line=$0
            sub(/^[[:space:]][[:space:]]/, "", line)
            print line
            next
          }
          if ($0 ~ /^[[:space:]][[:space:]][^[:space:]].*:[[:space:]]*$/) {
            name=$0
            sub(/^[[:space:]][[:space:]]/, "", name)
            sub(/:[[:space:]]*$/, "", name)
            if (!first) {print ""} else {first=0}
            print "Host " name
            next
          }
          if ($0 ~ /^[[:space:]][[:space:]][[:space:]][[:space:]][^[:space:]].*:[[:space:]]*.*$/) {
            line=$0
            sub(/^[[:space:]][[:space:]][[:space:]][[:space:]]/, "", line)
            key=line
            val=line
            sub(/:.*/, "", key)
            sub(/^[^:]*:[[:space:]]*/, "", val)
            key=trim(key)
            val=trim(val)
            if (key != "" && val != "") {
              print "  " key " " val
            }
            next
          }
        }
      ' "${SCRIPT_DIR}/package.conf" > "${SSH_CONFIG_TMP}"
      if [[ -s "${SSH_CONFIG_TMP}" ]]; then
        SSH_CONFIG="${SSH_CONFIG_TMP}"
      else
        rm -f "${SSH_CONFIG_TMP}"
        SSH_CONFIG_TMP=""
      fi
    fi
    if [[ -z "${SSH_CONFIG}" && -f "${HOME}/.ssh/config" ]]; then
      SSH_CONFIG="${HOME}/.ssh/config"
    fi
  fi

  scp_cmd=(scp)
  if [[ -n "${SSH_CONFIG}" ]]; then
    scp_cmd+=(-F "${SSH_CONFIG}")
  fi

  if [[ -z "${SHIP_PATH}" ]]; then
    SHIP_PATH="~/"
  fi

  echo "==> Copying bundle to ${SHIP_HOST}:${SHIP_PATH}"
  if "${scp_cmd[@]}" "${archive}" "${SHIP_HOST}:${SHIP_PATH}"; then
    echo "==> Bundle copied to ${SHIP_HOST}:${SHIP_PATH}"
    rm -f "${archive}"
    echo "==> Removed local bundle: ${archive}"
  else
    echo "==> Warning: failed to copy bundle to ${SHIP_HOST}. Skipping ship."
  fi
  if [[ -n "${SSH_CONFIG_TMP}" ]]; then
    rm -f "${SSH_CONFIG_TMP}"
  fi
fi
