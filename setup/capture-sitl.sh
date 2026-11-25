#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  printf '%s\n' 'Usage: bash setup/capture-sitl.sh [NEW_OUTPUT_DIRECTORY]'
  printf '%s\n' 'Ubuntu only. Start a headless passive run, record checks during 45 seconds, and audit the saved capture.'
  exit 0
fi
[[ $# -le 1 ]] || { echo 'Expected at most one output directory.' >&2; exit 2; }
[[ "$(uname -s)" == Linux ]] || { echo 'Run this script inside Ubuntu 24.04.' >&2; exit 1; }
[[ -z "${VIRTUAL_ENV:-}" && -z "${CONDA_PREFIX:-}" ]] || {
  echo 'Open a fresh shell outside Python virtual environments.' >&2; exit 1;
}
out="${1:-$root/runs/live-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
case "$out" in /*) ;; *) out="$PWD/$out";; esac
[[ ! -e "$out" && ! -L "$out" ]] || { echo "Output already exists: $out" >&2; exit 2; }

launch_log="$(mktemp)"
pid=''
cleanup() {
  if [[ -n "$pid" ]]; then
    # The runner owns a separate process group. TERM lets launch.py close its services.
    kill -TERM -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
  rm -f -- "$launch_log"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd -- "$root"
setsid xvfb-run -a env UAV_SOFTWARE_GL=1 bash simulation/run.sh start \
  --duration 45 --output "$out" > "$launch_log" 2>&1 < /dev/null &
pid=$!
until grep -q '^Ready:' "$launch_log" || ! kill -0 "$pid" 2>/dev/null; do sleep 1; done
cat "$launch_log"
if ! grep -q '^Ready:' "$launch_log"; then
  wait "$pid" || true
  pid=''
  exit 1
fi
mkdir -p "$out/validation"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
rc=0
bash simulation/run.sh check | tee "$out/validation/check-$stamp.txt" || rc=1
bash simulation/run.sh scan | tee "$out/validation/scan-$stamp.txt" || rc=1
wait "$pid" || rc=1
pid=''
cp "$launch_log" "$out/launcher.log"
/usr/bin/python3 simulation/verify_capture.py "$out" --output "$out/independent_verification.json" || rc=1
[[ "$rc" == 0 ]] || exit "$rc"
printf 'Verified passive capture: %s\n' "$out"
