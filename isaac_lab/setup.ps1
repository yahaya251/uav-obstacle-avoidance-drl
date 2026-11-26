param(
    [ValidateSet('install', 'check', 'run', 'help')]
    [string]$Action = 'help'
)

$ErrorActionPreference = 'Stop'
$runArguments = $args
$repo = Split-Path -Parent $PSScriptRoot
$repoParent = Split-Path -Parent $repo
$runtimeHome = $repo
if ((Split-Path -Leaf $repoParent) -eq 'github') {
    $workspace = Split-Path -Parent $repoParent
    if ($workspace -and (Test-Path -LiteralPath (Join-Path $workspace 'START_HERE.md') -PathType Leaf)) {
        $runtimeHome = $workspace
    }
}
$runtime = Join-Path $runtimeHome '.windows'
$venv = Join-Path $runtime 'isaac'
if ($env:UAV_ISAAC_VENV) {
    $venv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($env:UAV_ISAAC_VENV)
}
$python = Join-Path $venv 'Scripts\python.exe'
$labVersion = '3.0.0'
$simVersion = '6.2.0.0'

if ($Action -eq 'help') {
    Write-Output @"
Usage: .\isaac_lab\setup.ps1 install | check | run <smoke|demo|train|evaluate|benchmark> [arguments]
Native Windows x64, Python 3.12, Isaac Lab $labVersion and Isaac Sim $simVersion.
Runtime: $venv (override with UAV_ISAAC_VENV)
Set UAV_UV to uv.exe if it is outside $runtime\uv and PATH.
"@
    return
}

if ([System.Environment]::OSVersion.Platform -ne 'Win32NT' -or
    [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture -ne 'X64') {
    throw 'Use this script on native Windows x64.'
}
if ($Action -ne 'run' -and $runArguments.Count) {
    throw 'Only the run action accepts additional arguments.'
}

function Invoke-Checked {
    param([string]$Executable, [string[]]$Arguments)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$(Split-Path -Leaf $Executable) exited with code $LASTEXITCODE."
    }
}

if ($Action -eq 'run') {
    if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw 'Run .\isaac_lab\setup.ps1 install first.'
    }
    $savedPortableRoot = $env:UAV_ISAAC_PORTABLE_ROOT
    $savedWarpCache = $env:WARP_CACHE_PATH
    try {
        if (-not $env:UAV_ISAAC_PORTABLE_ROOT) {
            $env:UAV_ISAAC_PORTABLE_ROOT = Join-Path $runtime 'kit'
        }
        if (-not $env:WARP_CACHE_PATH) {
            $env:WARP_CACHE_PATH = Join-Path $runtime 'warp'
        }
        & $python (Join-Path $PSScriptRoot 'run.py') @runArguments
        exit $LASTEXITCODE
    } finally {
        $env:UAV_ISAAC_PORTABLE_ROOT = $savedPortableRoot
        $env:WARP_CACHE_PATH = $savedWarpCache
    }
}

$gpu = Get-Command nvidia-smi.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $gpu) { throw 'Install a supported NVIDIA RTX driver so nvidia-smi.exe is available.' }
Invoke-Checked $gpu.Source @('--query-gpu=name,driver_version,memory.total', '--format=csv')

if ($Action -eq 'install') {
    $uv = $env:UAV_UV
    if (-not $uv) {
        $uv = Join-Path $runtime 'uv\uv.exe'
        if (-not (Test-Path -LiteralPath $uv -PathType Leaf)) {
            $uv = (Get-Command uv.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1).Source
        }
    }
    if (-not $uv) {
        throw 'Install uv from https://docs.astral.sh/uv/getting-started/installation/ or set UAV_UV to uv.exe.'
    }

    $settings = @{
        UV_PYTHON_INSTALL_DIR = (Join-Path $runtime 'python')
        UV_PYTHON_BIN_DIR = (Join-Path $runtime 'bin')
        UV_CACHE_DIR = (Join-Path $runtime 'cache')
        UV_NO_MODIFY_PATH = '1'
        UV_HTTP_TIMEOUT = '180'
    }
    $previous = @{}
    try {
        foreach ($name in $settings.Keys) {
            $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
            if (-not $previous[$name]) {
                [Environment]::SetEnvironmentVariable($name, $settings[$name], 'Process')
            }
        }
        if (-not (Test-Path -LiteralPath $venv)) {
            New-Item -ItemType Directory -Path (Split-Path -Parent $venv) -Force | Out-Null
            Invoke-Checked $uv @('venv', '--python', '3.12', $venv)
        }
        if (-not (Test-Path -LiteralPath (Join-Path $venv 'pyvenv.cfg') -PathType Leaf) -or
            -not (Test-Path -LiteralPath $python -PathType Leaf)) {
            throw "Expected a Windows Python virtual environment at $venv. Select a fresh UAV_ISAAC_VENV."
        }
        Invoke-Checked $python @('-c', 'import sys; sys.exit(sys.version_info[:2] != (3, 12))')
        Invoke-Checked $uv @(
            'pip', 'install', '--python', $python, "isaaclab[isaacsim,sb3]==$labVersion",
            '--overrides', "https://raw.githubusercontent.com/isaac-sim/IsaacLab/v$labVersion/tools/wheel_builder/uv-overrides.txt",
            '--extra-index-url', 'https://pypi.nvidia.com', '--torch-backend', 'cu130',
            '--index-strategy', 'unsafe-best-match'
        )
        Invoke-Checked $uv @('pip', 'install', '--python', $python, '--no-deps', '-e', $repo)
    } finally {
        foreach ($name in $previous.Keys) {
            [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process')
        }
    }
}

if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    throw 'Run .\isaac_lab\setup.ps1 install first.'
}
$check = @'
from importlib.metadata import version
import sys

import torch
import uav_nav

if sys.version_info[:2] != (3, 12):
    raise SystemExit("Expected Python 3.12.")
for package, expected in zip(("isaaclab", "isaacsim"), sys.argv[1:], strict=True):
    installed = version(package)
    if installed != expected:
        raise SystemExit(f"Expected {package} {expected}, found {installed}.")
    print(f"{package}: {installed}")
if not torch.cuda.is_available() or torch.version.cuda != "13.0":
    raise SystemExit("Expected a working PyTorch CUDA 13.0 environment.")
tensor = torch.arange(4, device="cuda:0")
if tensor.square().sum().item() != 14:
    raise SystemExit("CUDA computation check failed.")
print(f"PyTorch {torch.__version__} | CUDA {torch.version.cuda} | {torch.cuda.get_device_name(0)}")
print(f"Stable-Baselines3: {version('stable-baselines3')}")
'@
$check | & $python - $labVersion $simVersion
if ($LASTEXITCODE -ne 0) { throw 'Isaac dependency or CUDA check failed.' }
Write-Output 'CUDA check passed. Run .\isaac_lab\setup.ps1 run smoke --num-envs 1 to check the scene.'
