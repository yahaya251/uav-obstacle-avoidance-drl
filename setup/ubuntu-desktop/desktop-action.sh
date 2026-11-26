#!/usr/bin/env bash
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
action="${1:-}"
entrypoint="$script_dir/ubuntu-start.sh"
if [[ ! -r "$entrypoint" ]]; then entrypoint="$script_dir/start.sh"; fi
status=0
pause=1
if [[ $# -eq 2 && "$2" == --no-pause ]]; then
  pause=0
elif [[ $# -ne 1 ]]; then
  echo 'Usage: desktop-action.sh {full-gui|start|start-gui|check|scan|gui|dashboard|stop|capture|capture-gui|results} [--no-pause]' >&2
  exit 2
fi

run_managed() (
  graphical=0
  timed=0
  verify_capture=0
  case "$1" in
    start-gui) graphical=1 ;;
    capture) timed=1 ;;
    capture-gui) graphical=1; timed=1; verify_capture=1 ;;
    verified-capture) timed=1; verify_capture=1 ;;
  esac
  if [[ "$graphical" == 1 && -z "${DISPLAY:-}" ]]; then
    printf '%s\n' 'Open this launcher on the Ubuntu graphical desktop with an X11 display.' >&2
    exit 1
  fi
  if existing="$(bash "$entrypoint" active-run)"; then
    printf 'Stop the current simulation before starting another.\n%s\n' "$existing" >&2
    exit 1
  fi
  launch_log="$(mktemp)" || exit 1
  run=''
  started=''
  viewer=''
  check_pid=''
  scan_pid=''
  emitted_run() { sed -n 's@^Starting passive simulation. Logs: @@p' "$launch_log"; }
  stop_managed() {
    local active
    active="$(bash "$entrypoint" active-run)" || return 1
    [[ -n "$run" && "$run" == "$active" ]] || return 1
    bash "$entrypoint" stop
  }
  close_viewer() {
    [[ -n "$viewer" ]] || return 0
    kill -TERM -- "-$viewer" 2>/dev/null || kill -TERM "$viewer" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 "$viewer" 2>/dev/null || break
      sleep .2
    done
    if kill -0 "$viewer" 2>/dev/null; then kill -KILL -- "-$viewer" 2>/dev/null || true; fi
    wait "$viewer" 2>/dev/null || true
  }
  cleanup() {
    local validator_pid
    for validator_pid in "$check_pid" "$scan_pid"; do
      if [[ -n "$validator_pid" ]]; then
        kill -TERM -- "-$validator_pid" 2>/dev/null || kill -TERM "$validator_pid" 2>/dev/null || true
        wait "$validator_pid" || true
      fi
    done
    if [[ -z "$run" ]]; then run="$(emitted_run)"; fi
    if [[ -n "$started" ]]; then
      if ! stop_managed; then
        kill -TERM -- "-$started" 2>/dev/null || kill -TERM "$started" 2>/dev/null || true
      fi
      wait "$started" || true
    fi
    close_viewer
    if [[ -n "$run" && -d "$run" ]]; then cp -- "$launch_log" "$run/launcher.log"; fi
    rm -f -- "$launch_log" "$launch_log.check" "$launch_log.scan"
  }
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  start_args=()
  if [[ "$timed" == 1 ]]; then
    printf '%s\n' 'Starting 45-second capture after readiness.'
    start_args=(--duration 45)
  fi
  setsid bash "$entrypoint" start "${start_args[@]}" > "$launch_log" 2>&1 &
  started=$!
  until grep -q '^Ready:' "$launch_log" || ! kill -0 "$started" 2>/dev/null; do sleep 1; done
  cat "$launch_log"
  run="$(emitted_run)"
  if [[ -z "$run" ]] || ! grep -q '^Ready:' "$launch_log"; then
    printf '%s\n' 'Simulation did not reach readiness.' >&2
    exit 1
  fi
  export UAV_EXPECTED_RUN="$run"
  if [[ "$graphical" == 1 ]]; then
    printf 'Opening Gazebo GUI. Log: %s\n' "$run/gazebo-gui.log"
    setsid bash "$entrypoint" gui > "$run/gazebo-gui.log" 2>&1 &
    viewer=$!
  fi

  result=0
  if [[ "$timed" == 1 ]]; then
    # Both validators need the same live capture, especially on a slow guest.
    setsid bash "$entrypoint" check --record > "$launch_log.check" 2>&1 &
    check_pid=$!
    setsid bash "$entrypoint" scan --record > "$launch_log.scan" 2>&1 &
    scan_pid=$!
    wait "$check_pid" || result=$?
    check_pid=''
    wait "$scan_pid" || result=$?
    scan_pid=''
    cat "$launch_log.check" "$launch_log.scan"
  fi
  if [[ -n "$viewer" ]]; then
    while kill -0 "$started" 2>/dev/null && kill -0 "$viewer" 2>/dev/null; do sleep 1; done
    if ! kill -0 "$viewer" 2>/dev/null; then
      gui_status=0
      wait "$viewer" || gui_status=$?
      viewer=''
      if [[ "$gui_status" != 0 ]]; then
        printf 'Gazebo GUI failed. Log: %s\n' "$run/gazebo-gui.log" >&2
        exit "$gui_status"
      fi
      if [[ "$timed" == 0 ]] && kill -0 "$started" 2>/dev/null; then
        stop_managed || result=$?
      fi
    fi
  fi

  server_status=0
  wait "$started" || server_status=$?
  started=''
  if [[ "$timed" == 0 && "$server_status" == 130 ]]; then server_status=0; fi
  if [[ "$server_status" != 0 ]]; then result="$server_status"; fi
  if [[ "$timed" == 0 ]]; then exit "$result"; fi

  if [[ "$result" == 0 && "$verify_capture" == 1 ]]; then
    bash "$entrypoint" verify "$run" || result=$?
  fi
  if [[ "$result" != 0 || ! -f "$run/monitor/run_summary.json" ]]; then
    printf '%s\n' 'Capture or validation did not complete.' >&2
    tail -n 5 "$launch_log" >&2
    if [[ "$result" == 0 ]]; then result=1; fi
    exit "$result"
  fi
  run_id="${run##*/}"
  if [[ ! "$run_id" =~ ^live-[0-9]{8}T[0-9]{6}\.[0-9]{9}Z$ ]]; then
    printf 'Capture returned an unexpected run ID: %s\n' "$run_id" >&2
    exit 1
  fi
  printf 'Capture completed: %s\n' "$run"
  printf 'UAV_CAPTURE_RUN_ID=%s\n' "$run_id"
)

case "$action" in
  start|stop|gui|results)
    bash "$entrypoint" "$action" || status=$?
    ;;
  check|scan)
    bash "$entrypoint" "$action" --record || status=$?
    ;;
  start-gui|capture|capture-gui)
    run_managed "$action" || status=$?
    ;;
  full-gui)
    if [[ -z "${DISPLAY:-}" ]]; then
      echo 'Open this launcher from the Ubuntu graphical desktop.' >&2
      status=1
    else
      # Start the viewer after the verified capture so its cold start cannot starve sensor checks.
      run_managed verified-capture && run_managed start-gui || status=$?
    fi
    ;;
  dashboard|zh|en)
    bash "$entrypoint" dashboard || status=$?
    ;;
  *)
    echo 'Unknown UAV desktop action.' >&2
    exit 2
    ;;
esac

case "$action:$status" in
  dashboard:0|zh:0|en:0|gui:0|results:0) ;;
  *)
    printf '\nExit status %s\n' "$status"
    if [[ "$pause" -eq 1 && -t 0 ]]; then
      read -r -p 'Press Enter to close. ' _ || true
    fi
    ;;
esac
exit "$status"
