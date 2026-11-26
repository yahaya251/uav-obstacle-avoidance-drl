#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ $# -eq 1 && "$1" == --workspace ]]; then
  repo="$(cd -- "$script_dir/../.." && pwd -P)"
  [[ -r "$repo/simulation/run.sh" ]] || {
    echo 'Run the workspace installer from a project checkout.' >&2
    exit 1
  }
  state="$repo/.ubuntu-runtime"
  workspace="$(cd -- "$repo/../.." && pwd -P)"
  if [[ "$repo" == "$workspace/github/uav-obstacle-avoidance-drl" && -f "$workspace/START_HERE.md" ]]; then
    state="$workspace/.ubuntu-runtime"
  fi
  runner="$script_dir/desktop-action.sh"
  entrypoint="$script_dir/start.sh"
elif [[ $# -eq 0 ]]; then
  state="$HOME/uav-nav-runtime"
  runner="$state/current/desktop-action.sh"
  entrypoint="$state/current/ubuntu-start.sh"
else
  echo 'Usage: bash install-desktop.sh [--workspace]' >&2
  exit 2
fi
[[ -r "$runner" && -r "$entrypoint" ]] || { echo 'The Ubuntu desktop runtime is missing.' >&2; exit 1; }
desktop="$(xdg-user-dir DESKTOP)"
if [[ -z "$desktop" || "$desktop" == "$HOME" ]]; then desktop="$HOME/Desktop"; fi
applications="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$state" "$desktop" "$applications"
staging="$(mktemp -d "$state/.desktop-install.XXXXXXXX")"
trap 'rm -rf -- "$staging"' EXIT
backup="$state/desktop-backups/$(date -u +%Y%m%dT%H%M%SZ)-$$"

write_entry() {
  local file="$1" action="$2" icon="$3" title="$4"
  cat > "$staging/$file" <<DESKTOP
[Desktop Entry]
Version=1.0
Type=Application
Name=$title
Exec=/bin/bash "$runner" $action
Icon=$icon
Terminal=true
StartupNotify=false
Categories=Science;
X-UAV-Managed=true
DESKTOP
  if command -v desktop-file-validate >/dev/null; then
    desktop-file-validate "$staging/$file"
  fi
}

write_entry uav-nav-00-full-gui.desktop full-gui applications-science '00 Verify Capture and Keep Gazebo Open'
write_entry uav-nav-01-start.desktop start-gui media-playback-start '01 Start UAV Simulator and Gazebo'
write_entry uav-nav-02-check.desktop check utilities-system-monitor '02 Check UAV Status and ROS'
write_entry uav-nav-03-scan.desktop scan network-wireless '03 Verify 360° LiDAR'
write_entry uav-nav-04-gazebo.desktop gui applications-science '04 Open Gazebo Viewer'
write_entry uav-nav-05-dashboard.desktop dashboard applications-internet '05 Open UAV Dashboard'
write_entry uav-nav-07-stop.desktop stop media-playback-stop '07 Stop UAV Simulator'
write_entry uav-nav-08-capture.desktop capture-gui media-record '08 Capture 45 Seconds with Viewer'
write_entry uav-nav-09-results.desktop results folder '09 Open UAV Results'

backup_item() {
  local path="$1" category="$2"
  mkdir -p "$backup/$category"
  mv -- "$path" "$backup/$category/"
}

for dir in "$desktop" "$applications"; do
  for file in "$staging"/*.desktop; do
    path="$dir/${file##*/}"
    if [[ -e "$path" || -L "$path" ]]; then
      if [[ -L "$path" || ! -f "$path" ]] || ! grep -qx 'X-UAV-Managed=true' "$path"; then
        printf 'Existing unmanaged file conflicts with a new shortcut: %s\n' "$path" >&2
        exit 1
      fi
    fi
  done
done

legacy=(
  uav-dashboard-en.desktop uav-dashboard-zh.desktop uav-project-check.desktop
  uav-project-restart.desktop uav-project-results.desktop uav-project-rl-terminal.desktop
  uav-project-simulator.desktop uav-project-stop.desktop uav-project-terminal.desktop
  uav-project-tests.desktop
)
for category in Desktop Applications; do
  dir="$desktop"
  if [[ "$category" == Applications ]]; then dir="$applications"; fi
  for name in "${legacy[@]}"; do
    path="$dir/$name"
    if [[ -f "$path" ]] && grep -q '^Exec=.*[/]uav_project[/]' "$path"; then
      backup_item "$path" "$category"
    fi
  done
  for name in uav-nav-05-dashboard-zh.desktop uav-nav-06-dashboard-en.desktop; do
    path="$dir/$name"
    if [[ -f "$path" && ! -L "$path" ]] && grep -qx 'X-UAV-Managed=true' "$path"; then
      backup_item "$path" "$category"
    fi
  done
  for file in "$staging"/*.desktop; do
    path="$dir/${file##*/}"
    if [[ -f "$path" ]] && ! cmp -s "$file" "$path"; then
      backup_item "$path" "$category"
    fi
    if [[ ! -f "$path" ]]; then install -m 755 "$file" "$path"; fi
    chmod u+x "$path"
  done
done

commands="$desktop/UAV_Project_Commands_ZH_EN.txt"
if [[ -f "$commands" || -L "$commands" ]]; then backup_item "$commands" Desktop; fi
old_link="$desktop/UAV_Project_Files"
if [[ -L "$old_link" && "$(readlink -f -- "$old_link")" == "$HOME/uav_project" ]]; then
  backup_item "$old_link" Desktop
fi

# On GNOME, Desktop Icons NG reads per-file metadata from the user's session.
# On Xfce, the executable mode also permits launchers to be trusted by the user.
session_dir="/run/user/$(id -u)"
if [[ -S "$session_dir/bus" ]]; then
  export XDG_RUNTIME_DIR="$session_dir"
  export DBUS_SESSION_BUS_ADDRESS="unix:path=$session_dir/bus"
fi
untrusted=0
for file in "$staging"/*.desktop; do
  if ! gio set -t string "$desktop/${file##*/}" metadata::trusted true 2>/dev/null; then
    untrusted=$((untrusted + 1))
  fi
done
if command -v update-desktop-database >/dev/null; then update-desktop-database "$applications"; fi
printf 'Installed 9 UAV launchers in %s and %s.\n' "$desktop" "$applications"
if [[ -d "$backup" ]]; then printf 'Previous project shortcuts: %s\n' "$backup"; fi
if [[ "$untrusted" -gt 0 ]]; then
  printf 'Desktop trust could not be recorded for %s icons. If prompted, right-click the icon and choose Allow Launching.\n' "$untrusted"
fi
