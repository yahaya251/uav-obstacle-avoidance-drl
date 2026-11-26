#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
venv="${UAV_ISAAC_VENV:-$root/.venv-isaac}"
case "$venv" in /*) ;; *) venv="$PWD/$venv";; esac
mode="${1:-help}"
isaac_lab_version='3.0.0'
isaac_sim_version='6.2.0.0'

usage() {
  printf '%s\n' 'Usage: bash isaac_lab/setup.sh {install|check}'
  printf '%s\n' 'Uses a dedicated Python 3.12 environment on an NVIDIA RTX Linux x86-64 host.'
  printf 'Python environment: %s (override with UAV_ISAAC_VENV)\n' "$venv"
}
die() { printf '%s\n' "$*" >&2; exit 1; }

[[ $# -le 1 ]] || { usage >&2; exit 2; }
case "$mode" in
  help|-h|--help) usage; exit 0 ;;
  install|check) ;;
  *) usage >&2; exit 2 ;;
esac
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] ||
  die 'This Isaac Sim runtime requires an NVIDIA RTX Linux x86-64 host. The macOS or Lima workspace can use the existing planar tools.'
command -v nvidia-smi >/dev/null || die 'nvidia-smi is missing. Install the supported NVIDIA driver on the host.'
nvidia-smi -L || die 'The NVIDIA driver did not enumerate a GPU.'

python="$venv/bin/python"
if [[ "$mode" == install ]]; then
  command -v uv >/dev/null || die 'Install uv using its official instructions, then retry.'
  if [[ ! -e "$venv" ]]; then
    uv venv --python 3.12 "$venv"
  fi
  [[ -f "$venv/pyvenv.cfg" && -x "$python" ]] || die "Expected a Python virtual environment: $venv"
  "$python" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 12))' || die "Expected Python 3.12: $python"
  # Use the upstream v3.0.0 Isaac Sim dependency overrides. They resolve Isaac Sim 6.2 and Isaac Lab's shared packages.
  uv pip install --python "$python" \
    "isaaclab[isaacsim,sb3]==$isaac_lab_version" "isaacsim[all,extscache]==$isaac_sim_version" \
    --overrides 'https://raw.githubusercontent.com/isaac-sim/IsaacLab/v3.0.0/tools/wheel_builder/uv-overrides.txt' \
    --extra-index-url 'https://pypi.nvidia.com' --torch-backend cu130 --index-strategy unsafe-best-match
  uv pip install --python "$python" --no-deps -e "$root"
fi

[[ -x "$python" ]] || die 'Run bash isaac_lab/setup.sh install first.'
"$python" - "$isaac_lab_version" "$isaac_sim_version" <<'PY'
from importlib.metadata import version
import sys

import torch

if sys.version_info[:2] != (3, 12):
    raise SystemExit("Expected Python 3.12.")
for package, expected in zip(("isaaclab", "isaacsim"), sys.argv[1:], strict=True):
    installed = version(package)
    if installed != expected:
        raise SystemExit(f"Expected {package} {expected}, found {installed}.")
    print(f"{package}: {installed}")
if not torch.cuda.is_available():
    raise SystemExit("PyTorch cannot access CUDA on this host.")
print(f"PyTorch: {torch.__version__} | CUDA {torch.version.cuda} | {torch.cuda.get_device_name(0)}")
print(f"Stable-Baselines3: {version('stable-baselines3')}")
print("Dependency and CUDA check passed. Start the smoke task to validate Isaac Sim and the scene.")
PY
printf 'Run: %q %q smoke\n' "$python" "$root/isaac_lab/run.py"
