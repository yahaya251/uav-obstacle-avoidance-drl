[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('help', 'setup', 'core', 'smoke', 'exploratory', 'confirmation', 'full')]
    [string]$Action = 'help'
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false

if ($Action -eq 'help') {
    Write-Output 'Usage: .\setup\planar.ps1 {setup|core|smoke|exploratory|confirmation|full}'
    Write-Output 'Overrides: UAV_UV, UAV_PYTHON, UAV_PLANAR_VENV, UAV_PLANAR_RUNS.'
    return
}

$root = Split-Path -Parent $PSScriptRoot
$repoParent = Split-Path -Parent $root
$workspace = [IO.Path]::GetFullPath((Join-Path $root '..\..'))
$runtimeHome = $root
if ((Split-Path -Leaf $repoParent) -eq 'github' -and
    (Test-Path -LiteralPath (Join-Path $workspace 'START_HERE.md') -PathType Leaf)) {
    $runtimeHome = $workspace
}
$runtime = Join-Path $runtimeHome '.windows'

$venv = Join-Path $root '.venv-planar'
if ($env:UAV_PLANAR_VENV) {
    $venv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($env:UAV_PLANAR_VENV)
}
$runs = Join-Path $root 'runs'
if ($env:UAV_PLANAR_RUNS) {
    $runs = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($env:UAV_PLANAR_RUNS)
}
$python = Join-Path $venv 'Scripts\python.exe'
$pythonVersionCheck = "import sys; sys.exit(sys.version_info[:2] not in ((3, 11), (3, 12)))"

function Invoke-CheckedNative($Program, [string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$(Split-Path -Leaf $Program) failed with exit code ${LASTEXITCODE}."
    }
}

function Resolve-Uv {
    if ($env:UAV_UV) {
        return (Get-Command -Name $env:UAV_UV -CommandType Application | Select-Object -First 1).Source
    }
    $portableUv = Join-Path $runtime 'uv\uv.exe'
    if (Test-Path -LiteralPath $portableUv -PathType Leaf) {
        return $portableUv
    }
    $command = Get-Command -Name uv -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $command) {
        throw 'Install uv, or set UAV_UV to the full path of uv.exe.'
    }
    return $command.Source
}

function Assert-PlanarPython {
    if (-not (Test-Path -LiteralPath $python -PathType Leaf) -or
        -not (Test-Path -LiteralPath (Join-Path $venv 'pyvenv.cfg') -PathType Leaf)) {
        throw 'Run .\setup\planar.ps1 setup to create a Windows Python environment.'
    }
    & $python -c $pythonVersionCheck
    if ($LASTEXITCODE -ne 0) {
        throw 'The planar environment must use Python 3.11 or 3.12. Select a fresh UAV_PLANAR_VENV.'
    }
}

function Install-Planar {
    $uv = Resolve-Uv
    if (-not (Test-Path -LiteralPath $venv)) {
        $pythonRequest = '3.12'
        if ($env:UAV_PYTHON) {
            $pythonRequest = $env:UAV_PYTHON
        }
        Invoke-CheckedNative $uv @('venv', '--python', $pythonRequest, $venv)
    }
    Assert-PlanarPython
    Invoke-CheckedNative $uv @('pip', 'install', '--python', $python, '--torch-backend', 'cpu',
        'torch==2.6.0+cpu', '--editable', '.[benchmark,plots]')
    Invoke-CheckedNative $uv @('pip', 'check', '--python', $python)
}

function New-RunPath($Name) {
    $stamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmss'Z'")
    $suffix = [Guid]::NewGuid().ToString('N').Substring(0, 8)
    return (Join-Path $runs "$Name-$stamp-$suffix")
}

function Invoke-Core {
    Invoke-CheckedNative $python @('-m', 'uav_nav.checks', '--output', (New-RunPath 'core'))
}

function Invoke-Benchmark($Profile) {
    $path = New-RunPath $Profile
    Invoke-CheckedNative $python @('-m', 'uav_nav.benchmark', 'run', '--profile', $Profile, '--output', $path)
    Invoke-CheckedNative $python @('-m', 'uav_nav.benchmark', 'verify', $path)
    $audit = Get-Content -LiteralPath (Join-Path $path 'independent_verification.json') -Raw | ConvertFrom-Json
    if ($audit.passed -ne $true) {
        throw "Benchmark verification failed: $path"
    }
    Invoke-CheckedNative $python @('-m', 'uav_nav.benchmark', 'plot', $path)
    Write-Output "Verified benchmark: $path"
}

$settings = @{
    UV_CACHE_DIR = (Join-Path $runtime 'cache')
    UV_PYTHON_INSTALL_DIR = (Join-Path $runtime 'python')
    UV_PYTHON_BIN_DIR = (Join-Path $runtime 'bin')
    MPLCONFIGDIR = (Join-Path $runs '.matplotlib')
}
$temporarySettings = @()
Push-Location -LiteralPath $root
try {
    foreach ($name in $settings.Keys) {
        if (-not [Environment]::GetEnvironmentVariable($name, 'Process')) {
            [Environment]::SetEnvironmentVariable($name, $settings[$name], 'Process')
            $temporarySettings += $name
        }
    }

    switch ($Action) {
        'setup' { Install-Planar }
        'core' {
            Assert-PlanarPython
            Invoke-Core
        }
        'full' {
            Install-Planar
            Invoke-Core
            foreach ($profile in @('smoke', 'exploratory', 'confirmation')) {
                Invoke-Benchmark $profile
            }
        }
        default {
            Assert-PlanarPython
            Invoke-Benchmark $Action
        }
    }
} finally {
    foreach ($name in $temporarySettings) {
        [Environment]::SetEnvironmentVariable($name, $null, 'Process')
    }
    Pop-Location
}
