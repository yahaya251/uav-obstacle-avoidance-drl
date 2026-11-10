#!/usr/bin/env bash
set -eo pipefail

simulation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi

usage() {
  echo 'Usage: bash simulation/run.sh start [--output DIR] [--duration SECONDS]'
  echo '       bash simulation/run.sh {check|scan|gui}'
  echo '       See simulation/README.md for the Ubuntu setup.'
}

case "$action" in
  help|-h|--help) usage; exit ;;
  start|check|scan|gui) ;;
  *) usage >&2; exit 2 ;;
esac
if [[ "$action" != start && $# -ne 0 ]]; then
  usage >&2
  exit 2
fi
if [[ "$action" == start && "$*" == --help ]]; then
  usage
  exit 0
fi
if [[ "$(uname -s)" != Linux || ! -r /etc/os-release ]]; then
  echo 'Use Ubuntu 24.04 directly or inside a VM. See simulation/README.md.' >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'The live stack targets Ubuntu 24.04.' >&2
  exit 1
fi

export UAV_SIM_DEPS="${UAV_SIM_DEPS:-$HOME/.local/share/uav-nav-sim}"
export GZ_PARTITION="${GZ_PARTITION:-uav_nav_sitl}"
if [[ "${UAV_SOFTWARE_GL:-0}" == 1 ]]; then
  export QT_QPA_PLATFORM=xcb QT_XCB_GL_INTEGRATION=xcb_glx QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LIBGL_DRI3_DISABLE=1
  unset WAYLAND_DISPLAY
fi

if [[ "$action" == gui ]]; then
  exec gz sim -g
fi

ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
px4_msgs_setup="${PX4_MSGS_SETUP:-$UAV_SIM_DEPS/ros_ws/install/setup.bash}"
for setup in "$ros_setup" "$px4_msgs_setup"; do
  if [[ ! -r "$setup" ]]; then
    echo "Missing $setup. Follow simulation/README.md." >&2
    exit 1
  fi
  source "$setup"
done
export PYTHONNOUSERSITE=1
export PYTHONPATH="$simulation_dir/../src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY

case "$action" in
  start) exec /usr/bin/python3 "$simulation_dir/launch.py" "$@" ;;
  check)
    /usr/bin/python3 "$simulation_dir/check.py"
    exec /usr/bin/python3 "$simulation_dir/verify_graph.py"
    ;;
  scan) exec /usr/bin/python3 "$simulation_dir/verify_scan.py" ;;
esac
