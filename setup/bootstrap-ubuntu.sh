#!/usr/bin/env bash
set -euo pipefail

setup_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=versions.env
source "$setup_dir/versions.env"
mode="${1:-help}"
usage() {
  printf '%s\n' 'Usage: bash setup/bootstrap-ubuntu.sh {install|check}'
  printf '%s\n' 'Use a dedicated Ubuntu 24.04 machine or VM and a fresh shell (no Python venv).'
  printf '%s\n' 'Overrides: UAV_SIM_DEPS, UAV_BUILD_JOBS, PX4_ROOT, PX4_MSGS_SETUP, XRCE_AGENT, XRCE_LIB_DIR, ROS_SETUP, UAV_XRCE_SOURCE.'
}
die() { printf '%s\n' "$*" >&2; exit 1; }
[[ $# -le 1 ]] || { usage >&2; exit 2; }
case "$mode" in
  help|-h|--help) usage; exit 0;;
  install|check) ;;
  *) usage >&2; exit 2;;
esac
[[ "$(uname -s)" == Linux && -r /etc/os-release ]] || die 'Use Ubuntu 24.04 directly or inside a VM.'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == "$UAV_UBUNTU_VERSION" ]] || die "Expected Ubuntu $UAV_UBUNTU_VERSION."
[[ -z "${VIRTUAL_ENV:-}" && -z "${CONDA_PREFIX:-}" ]] || die 'Open a fresh shell outside Python virtual environments.'

deps="${UAV_SIM_DEPS:-$HOME/.local/share/uav-nav-sim}"
case "$deps" in /*) ;; *) deps="$PWD/$deps";; esac
px4="${PX4_ROOT:-$deps/PX4-Autopilot}"
ros_ws="$deps/ros_ws"
msgs="$ros_ws/src/px4_msgs"
ros="${ROS_SETUP:-/opt/ros/$UAV_ROS_DISTRO/setup.bash}"
overlay="${PX4_MSGS_SETUP:-$ros_ws/install/setup.bash}"
agent="${XRCE_AGENT:-$deps/xrce/bin/MicroXRCEAgent}"
agent_lib="${XRCE_LIB_DIR:-$deps/xrce/lib}"
xrce_source="${UAV_XRCE_SOURCE:-$deps/Micro-XRCE-DDS-Agent}"
jobs="${UAV_BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die 'UAV_BUILD_JOBS must be a positive integer.'

validate_tag() {
  local dir="$1" tag="$2" actual
  actual="$(git -C "$dir" describe --tags --exact-match HEAD 2>/dev/null || true)"
  [[ "$actual" == "$tag" ]] || die "Expected `$tag` at $dir; found ${actual:-no exact tag}. Use a fresh prefix or select the correct checkout."
}

ensure_checkout() {
  local url="$1" tag="$2" dir="$3"
  if [[ ! -e "$dir" ]]; then
    git clone --depth 1 --branch "$tag" "$url" "$dir"
  fi
  validate_tag "$dir" "$tag"
  [[ "$(git -C "$dir" config --get remote.origin.url)" == "$url" ]] ||
    die "Unexpected Git origin for $dir. Select a fresh UAV_SIM_DEPS prefix."
}

version_report() (
  [[ -x "$px4/build/px4_sitl_default/bin/px4" ]] || die "Missing PX4 SITL: $px4"
  [[ -x "$agent" ]] || die "Missing Micro XRCE-DDS Agent: $agent"
  [[ -r "$px4/Tools/simulation/gz/models/x500/model.sdf" ]] || die "Missing the x500 Gazebo model; initialize PX4 submodules."
  [[ -r "$ros" && -r "$overlay" ]] || die "Missing ROS 2 setup or px4_msgs overlay: $ros ; $overlay"
  validate_tag "$px4" "$UAV_PX4_TAG"
  # Resolve the source workspace from the overlay when using an existing install.
  local overlay_ws agent_version
  overlay_ws="$(cd -- "$(dirname -- "$overlay")/.." && pwd -P)"
  validate_tag "$overlay_ws/src/px4_msgs" "$UAV_PX4_MSGS_TAG"
  agent_version="$(LD_LIBRARY_PATH="$agent_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" timeout 5s "$agent" --version 2>&1 || true)"
  # v2.4.3 entrypoints may lack --version; verify the source tag as a fallback for managed installations.
  if [[ "$agent_version" != *"${UAV_XRCE_TAG#v}"* ]]; then
    validate_tag "$xrce_source" "$UAV_XRCE_TAG"
  fi
  set +u
  # shellcheck disable=SC1090
  source "$ros"
  # shellcheck disable=SC1090
  source "$overlay"
  set -u
  [[ "${ROS_DISTRO:-}" == "$UAV_ROS_DISTRO" ]] || die "Expected ROS 2 $UAV_ROS_DISTRO."
  command -v xvfb-run >/dev/null || die 'Missing xvfb-run.'
  command -v xauth >/dev/null || die 'Missing xauth.'
  command -v setsid >/dev/null || die 'Missing setsid (util-linux).'
  gz_versions="$(gz sim --versions)"
  printf '%s\n' "$gz_versions" | grep -Eq "(^|[^0-9])$UAV_GAZEBO_SIM_MAJOR\." ||
    die "Expected Gazebo Sim $UAV_GAZEBO_SIM_MAJOR ($UAV_GAZEBO_RELEASE); found: $gz_versions"
  ros2 pkg prefix ros_gz_bridge
  PYTHONNOUSERSITE=1 /usr/bin/python3 - <<'PY'
import rclpy
from rclpy.type_support import check_for_type_support
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from sensor_msgs.msg import LaserScan

for message in (VehicleOdometry, VehicleStatus, LaserScan):
    check_for_type_support(message)
print("ROS 2 message type support passed.")
PY
  printf 'Environment ready: Ubuntu %s, ROS 2 %s, Gazebo Sim %s, PX4 %s, px4_msgs %s, XRCE-DDS Agent %s\n' \
    "$VERSION_ID" "$ROS_DISTRO" "$gz_versions" "$UAV_PX4_TAG" "$UAV_PX4_MSGS_TAG" "$UAV_XRCE_TAG"
)

if [[ "$mode" == install ]]; then
  [[ "$EUID" -ne 0 ]] || die 'Run as a regular user with sudo access.'
  for var in PX4_ROOT PX4_MSGS_SETUP XRCE_AGENT XRCE_LIB_DIR ROS_SETUP UAV_XRCE_SOURCE; do
    [[ -z "${!var:-}" ]] || die "Unset $var for install. Use it only to check an existing installation."
  done
  mkdir -p "$deps" "$ros_ws/src"
  sudo apt-get update
  sudo apt-get install -y curl ca-certificates locales software-properties-common lsb-release wget git
  sudo locale-gen en_US.UTF-8
  export LANG=en_US.UTF-8
  sudo add-apt-repository -y universe
  if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q '^install ok installed$'; then
    apt_tag="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
      /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    [[ "$apt_tag" =~ ^v?[0-9]+(\.[0-9]+)+$ ]] || die "Unexpected ROS apt-source release tag: $apt_tag"
    apt_tmp="$(mktemp -d)"
    trap 'rm -f -- "$apt_tmp/ros2-apt-source.deb"; rmdir -- "$apt_tmp"' EXIT
    curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$apt_tag/ros2-apt-source_${apt_tag}.noble_all.deb" -o "$apt_tmp/ros2-apt-source.deb"
    sudo dpkg -i "$apt_tmp/ros2-apt-source.deb"
  fi
  sudo apt-get update
  sudo apt-get install -y "ros-$UAV_ROS_DISTRO-desktop" "ros-$UAV_ROS_DISTRO-ros-gz" ros-dev-tools \
    python3-pip python3-venv python3-numpy cmake build-essential bc cppzmq-dev libunwind-dev \
    xvfb xauth libgl1-mesa-dri

  ensure_checkout https://github.com/PX4/PX4-Autopilot.git "$UAV_PX4_TAG" "$px4"
  git -C "$px4" submodule update --init --recursive --depth 1
  bash "$px4/Tools/setup/ubuntu.sh" --no-nuttx
  make -C "$px4" px4_sitl_default -j "$jobs"

  ensure_checkout https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$UAV_XRCE_TAG" "$xrce_source"
  cmake -S "$xrce_source" -B "$deps/agent-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$deps/xrce"
  cmake --build "$deps/agent-build" --parallel "$jobs"
  cmake --install "$deps/agent-build"

  ensure_checkout https://github.com/PX4/px4_msgs.git "$UAV_PX4_MSGS_TAG" "$msgs"
  (
    set +u
    # shellcheck disable=SC1090
    source "$ros"
    cd -- "$ros_ws"
    PYTHONNOUSERSITE=1 colcon build --symlink-install
  )
fi
version_report
