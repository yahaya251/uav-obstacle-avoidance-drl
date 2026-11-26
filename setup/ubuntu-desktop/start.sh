#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
action="${1:-start}"
if [[ $# -gt 0 ]]; then shift; fi

case "$action" in
  full-gui|start|start-gui|capture|capture-gui|verify|check|scan|gui|dashboard|status|stop|results|active-run) ;;
  help|-h|--help)
    printf '%s\n' 'Usage: bash start.sh [full-gui | start [--duration SECONDS] | start-gui | capture | capture-gui]' \
      '       bash start.sh [verify RUN_DIRECTORY | check [--record] | scan [--record] | gui | dashboard | results | status | stop]'
    exit 0 ;;
  *) echo 'Unknown Ubuntu action. Run bash start.sh help.' >&2; exit 2 ;;
esac

if [[ "$(uname -s)" != Linux || ! -r /etc/os-release ]]; then
  echo 'Run this entrypoint inside Ubuntu 24.04.' >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'The PX4/ROS simulation requires Ubuntu 24.04.' >&2
  exit 1
fi

repo="$(cd -- "$script_dir/../.." && pwd -P)"
default_project="$script_dir/project"
default_state="$HOME/uav-nav-runtime"
if [[ ! -f "$default_project/simulation/run.sh" && -f "$repo/simulation/run.sh" ]]; then
  default_project="$repo"
  default_state="$repo/.ubuntu-runtime"
  workspace="$(cd -- "$repo/../.." && pwd -P)"
  if [[ "$repo" == "$workspace/github/uav-obstacle-avoidance-drl" && -f "$workspace/START_HERE.md" ]]; then
    default_state="$workspace/.ubuntu-runtime"
  fi
fi
export UAV_RUNTIME_STATE="$(realpath -m -- "${UAV_RUNTIME_STATE:-$default_state}")"

control="$script_dir/control.py"
case "$action" in
  full-gui|start-gui|capture|capture-gui)
    if [[ $# -ne 0 ]]; then exit 2; fi
    exec bash "$script_dir/desktop-action.sh" "$action" --no-pause
    ;;
  status|stop|active-run)
    if [[ $# -ne 0 ]]; then exit 2; fi
    exec /usr/bin/python3 "$control" "$action"
    ;;
  dashboard)
    if [[ $# -gt 1 || ( $# -eq 1 && "$1" != zh && "$1" != en ) ]]; then
      echo 'Usage: dashboard' >&2
      exit 2
    fi
    url='http://127.0.0.1:8086/'
    printf '%s\n' "$url"
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
      # GIO URL dispatch can miss the default snap handler and attempt HTTP HEAD
      # on the URL itself. Launch the configured browser entry directly in that case.
      default_browser="$(xdg-mime query default x-scheme-handler/http 2>/dev/null || true)"
      snap_firefox=/var/lib/snapd/desktop/applications/firefox_firefox.desktop
      if [[ "$default_browser" == firefox_firefox.desktop && -r "$snap_firefox" && -x /snap/bin/firefox ]] &&
          command -v gio >/dev/null; then
        exec gio launch "$snap_firefox" "$url"
      fi
      exec xdg-open "$url"
    fi
    exit 0
    ;;
  results)
    if [[ $# -ne 0 ]]; then exit 2; fi
    runs="$UAV_RUNTIME_STATE/runs"
    mkdir -p "$runs"
    printf '%s\n' "$runs"
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
      exec xdg-open "$runs"
    fi
    exit 0
    ;;
esac

project="${UAV_PROJECT_DIR:-$default_project}"
if [[ ! -f "$project/simulation/run.sh" ]]; then
  echo 'Set UAV_PROJECT_DIR to the path of the uav-obstacle-avoidance-drl checkout.' >&2
  exit 1
fi
project="$(cd -- "$project" && pwd -P)"

# Reuse the original Ubuntu install when present; otherwise use the repo defaults.
installed="$HOME/uav_project"
if [[ -x "$installed/src/PX4-Autopilot/build/px4_sitl_default/bin/px4" &&
      -r "$installed/ros2_ws/install/setup.bash" &&
      -x "$installed/install/xrce/bin/MicroXRCEAgent" && -z "${UAV_SIM_DEPS:-}" ]]; then
  export PX4_ROOT="${PX4_ROOT:-$installed/src/PX4-Autopilot}"
  export PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-$installed/ros2_ws/install/setup.bash}"
  export XRCE_AGENT="${XRCE_AGENT:-$installed/install/xrce/bin/MicroXRCEAgent}"
  export XRCE_LIB_DIR="${XRCE_LIB_DIR:-$installed/install/xrce/lib}"
fi
if [[ "$default_state" != "$HOME/uav-nav-runtime" && -z "${UAV_SIM_DEPS:-}${PX4_ROOT:-}" ]]; then
  existing_deps="$HOME/.local/share/uav-nav-sim"
  if [[ -d "$UAV_RUNTIME_STATE/deps" || ! -d "$existing_deps" ]]; then
    export UAV_SIM_DEPS="$UAV_RUNTIME_STATE/deps"
  fi
fi

launcher="$project/simulation/run.sh"
case "$action" in
  verify)
    if [[ $# -ne 1 ]]; then
      echo 'Usage: bash start.sh verify RUN_DIRECTORY' >&2
      exit 2
    fi
    output="$(realpath -m -- "$1")"
    if [[ "${output%/*}" != "$UAV_RUNTIME_STATE/runs" || ! -f "$output/monitor/run_summary.json" ]]; then
      echo 'Select a completed managed capture from the Ubuntu results directory.' >&2
      exit 1
    fi
    report=()
    if [[ ! -e "$output/independent_verification.json" ]]; then
      report=(--output "$output/independent_verification.json")
    fi
    exec /usr/bin/python3 "$project/simulation/verify_capture.py" "$output" "${report[@]}"
    ;;
  check|scan)
    if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --record ) ]]; then
      printf 'Usage: %s [--record]\n' "$action" >&2
      exit 2
    fi
    output="$(/usr/bin/python3 "$control" active-run)" || {
      echo 'Start the managed simulation before running checks.' >&2
      exit 1
    }
    if [[ -n "${UAV_EXPECTED_RUN:-}" && "$output" != "$UAV_EXPECTED_RUN" ]]; then
      echo 'The active run changed before validation.' >&2
      exit 1
    fi
    if [[ $# -eq 1 && "$1" == --record ]]; then
      validation="$output/validation"
      mkdir -p "$validation"
      report="$validation/$action-$(date -u +%Y%m%dT%H%M%S.%NZ).txt"
      bash "$launcher" "$action" | tee "$report"
      printf 'Validation log: %s\n' "$report" >&2
      exit 0
    fi
    exec bash "$launcher" "$action"
    ;;
  gui)
    if [[ $# -ne 0 || -z "${DISPLAY:-}" ]]; then
      echo 'Open a terminal on the Ubuntu graphical desktop to use Gazebo GUI.' >&2
      exit 1
    fi
    output="$(/usr/bin/python3 "$control" active-run)" || {
      echo 'Start the managed simulation before opening Gazebo GUI.' >&2
      exit 1
    }
    if [[ -n "${UAV_EXPECTED_RUN:-}" && "$output" != "$UAV_EXPECTED_RUN" ]]; then
      echo 'The active run changed before Gazebo GUI attached.' >&2
      exit 1
    fi
    ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
    if [[ -r "$ros_setup" ]]; then
      # ROS Jazzy may provide gz through its vendor packages.
      set +u
      source "$ros_setup"
      set -u
    fi
    if ! command -v gz >/dev/null; then
      echo 'Gazebo CLI is missing. Run the Ubuntu dependency setup first.' >&2
      exit 1
    fi
    px4_root="${PX4_ROOT:-${UAV_SIM_DEPS:-$HOME/.local/share/uav-nav-sim}/PX4-Autopilot}"
    export GZ_SIM_RESOURCE_PATH="$output/assets/models:$px4_root/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    export GZ_PARTITION="${GZ_PARTITION:-uav_nav_sitl}"
    export QT_QPA_PLATFORM=xcb QT_XCB_GL_INTEGRATION=xcb_glx QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LIBGL_DRI3_DISABLE=1
    export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
    unset WAYLAND_DISPLAY
    # Keep the software viewer from competing with time-sensitive sensor callbacks.
    exec nice -n 10 gz sim -g --render-engine-gui ogre
    ;;
esac

if [[ "${1:-}" == --help ]]; then exec bash "$launcher" start --help; fi
for argument in "$@"; do
  case "$argument" in
    --output|--output=*)
      printf 'Managed starts use a new folder in %s/runs. For a custom folder use simulation/run.sh directly.\n' "$UAV_RUNTIME_STATE" >&2
      exit 2 ;;
  esac
done
output="$UAV_RUNTIME_STATE/runs/live-$(date -u +%Y%m%dT%H%M%S.%NZ)"
command=(/usr/bin/python3 "$control" run "$output" "$launcher" "$@")
if [[ "${UAV_USE_X11:-0}" == 1 && -n "${DISPLAY:-}" ]]; then
  exec "${command[@]}"
fi
if ! command -v xvfb-run >/dev/null; then
  echo 'Install xvfb and xauth as documented in simulation/README.md, or use UAV_USE_X11=1 with X11.' >&2
  exit 1
fi
exec xvfb-run -a env UAV_SOFTWARE_GL=1 "${command[@]}"
