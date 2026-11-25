#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
venv="${UAV_PLANAR_VENV:-$root/.venv}"
runs="${UAV_PLANAR_RUNS:-$root/runs}"
case "$venv" in /*) ;; *) venv="$PWD/$venv";; esac
case "$runs" in /*) ;; *) runs="$PWD/$runs";; esac
action="${1:-help}"

usage() {
  printf '%s\n' 'Usage: bash setup/planar.sh {setup|core|smoke|exploratory|confirmation|full}'
  printf '%s\n' 'Overrides: UAV_PYTHON, UAV_PLANAR_VENV, UAV_PLANAR_RUNS.'
}
die() { printf '%s\n' "$*" >&2; exit 1; }
[[ $# -le 1 ]] || { usage >&2; exit 2; }
supported_python() {
  "$1" -c 'import sys; sys.exit(sys.version_info[:2] not in ((3, 11), (3, 12)))' >/dev/null 2>&1
}
python_path() {
  local item path
  for item in "${UAV_PYTHON:-python3.12}" python3.11 python3; do
    path="$(command -v -- "$item" || true)"
    if [[ -n "$path" ]] && supported_python "$path"; then printf '%s\n' "$path"; return; fi
    if [[ -n "${UAV_PYTHON:-}" ]]; then break; fi
  done
  return 1
}
python="$venv/bin/python"
setup() {
  if [[ ! -x "$python" ]]; then
    local base
    base="$(python_path)" || die 'Install Python 3.11 or 3.12, or set UAV_PYTHON to its executable.'
    "$base" -m venv "$venv"
  fi
  supported_python "$python" || die 'The environment must use Python 3.11 or 3.12. Select a fresh UAV_PLANAR_VENV.'
  if ! "$python" -c 'import gymnasium, numpy, torch, stable_baselines3, matplotlib' 2>/dev/null ||
     ! "$python" -m pip check; then
    if [[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]]; then
      "$python" -m pip install torch==2.6.0 --index-url https://download.pytorch.org/whl/cpu
    fi
    "$python" -m pip install "$root[benchmark,plots]"
  fi
  "$python" -c 'from importlib.metadata import version; expected={"numpy":"1.26.4","gymnasium":"1.0.0","torch":"2.6.0","stable-baselines3":"2.5.0"}; wrong={p:version(p) for p in expected if version(p).split("+")[0]!=expected[p]}; assert not wrong, f"Dependency pins do not match: {wrong}"' ||
    die 'Use a fresh UAV_PLANAR_VENV or reinstall the benchmark extras at the pinned versions.'
  "$python" -m pip check
}
require_python() {
  [[ -x "$python" ]] && supported_python "$python" || die 'Run bash setup/planar.sh setup first.'
}
new_path() {
  printf '%s/%s-%s-%s\n' "$runs" "$1" "$(date -u +%Y%m%dT%H%M%SZ)" "$$"
}
core() {
  local o
  o="$(new_path core)"
  "$python" -m uav_nav.checks --output "$o"
}
benchmark() {
  local profile="$1" o
  o="$(new_path "$profile")"
  "$python" -m uav_nav.benchmark run --profile "$profile" --output "$o"
  "$python" -m uav_nav.benchmark verify "$o"
  "$python" -c 'import json,sys; sys.exit(json.load(open(sys.argv[1]))["passed"] is not True)' "$o/independent_verification.json"
  "$python" -m uav_nav.benchmark plot "$o"
  printf 'Verified benchmark: %s\n' "$o"
}

case "$action" in
  help|-h|--help) usage; exit 0;;
  setup|core|smoke|exploratory|confirmation|full) ;;
  *) usage >&2; exit 2;;
esac

cd -- "$root"
export PYTHONPATH="$root/src" PYTHONDONTWRITEBYTECODE=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-$runs/.matplotlib}"
case "$action" in
  setup) setup;;
  core) require_python; mkdir -p "$runs"; core;;
  smoke|exploratory|confirmation) require_python; mkdir -p "$MPLCONFIGDIR"; benchmark "$action";;
  full)
    setup
    mkdir -p "$MPLCONFIGDIR"
    core
    for profile in smoke exploratory confirmation; do benchmark "$profile"; done
    printf 'Planar workflow passed: %s\n' "$runs"
    ;;
esac
