#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateSet('Full', 'Quick', 'Check', 'Plan', 'Planar', 'Demo', 'Train', 'Evaluate', 'Benchmark')]
    [string]$Mode = 'Full',
    [string]$Checkpoint,
    [switch]$ShowIsaacDemo,
    [switch]$NonInteractive,
    [switch]$NoInstall,
    [switch]$PlanOnly
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$planarScript = Join-Path $repo 'setup\planar.ps1'
$isaacScript = Join-Path $repo 'isaac_lab\setup.ps1'
$pwsh = Join-Path $PSHOME 'pwsh.exe'
$runs = Join-Path $repo 'runs\windows'
$planarFull = @('planar-setup', 'planar-core', 'planar-smoke', 'planar-exploratory', 'planar-confirmation')
$isaacSmoke = @('isaac-smoke-1', 'isaac-smoke-4')
$isaacEvaluation = @('isaac-validation', 'isaac-test')
$isaacBenchmarks = foreach ($count in 1, 8, 16, 32, 64) { "isaac-benchmark-$count" }
$workflows = @{
    Check = @('planar-check', 'isaac-check')
    Full = @('isaac-prepare') + $planarFull + $isaacSmoke + @('isaac-train') + $isaacEvaluation + $isaacBenchmarks
    Quick = @('isaac-prepare', 'planar-setup', 'planar-core', 'planar-smoke') + $isaacSmoke
    Planar = $planarFull
    Demo = @('isaac-prepare', 'isaac-demo')
    Train = @('isaac-prepare', 'isaac-train')
    Evaluate = @('isaac-prepare') + $isaacEvaluation
    Benchmark = @('isaac-prepare') + $isaacBenchmarks
}

if ($ShowIsaacDemo -and $Mode -notin @('Full', 'Plan')) {
    throw '-ShowIsaacDemo applies to the Full workflow. Choose Demo for just the GUI.'
}
if ($Checkpoint -and $Mode -ne 'Evaluate') { throw '-Checkpoint applies to Evaluate.' }
$selectedMode = if ($Mode -eq 'Plan') { 'Full' } else { $Mode }
$steps = @($workflows[$selectedMode])
if ($ShowIsaacDemo) {
    $beforeTraining = [array]::IndexOf($steps, 'isaac-train')
    $steps = $steps[0..($beforeTraining - 1)] + @('isaac-demo') + $steps[$beforeTraining..($steps.Count - 1)]
}
if ($NoInstall) {
    $steps = @($steps | ForEach-Object {
        switch ($_) {
            'planar-setup' { 'planar-check' }
            'isaac-prepare' { 'isaac-check' }
            default { $_ }
        }
    })
}
if ($Mode -eq 'Plan' -or $PlanOnly) {
    [ordered]@{ mode = $selectedMode; steps = $steps; output_base = $runs } | ConvertTo-Json
    return
}

$runID = '{0}-{1}-{2}' -f (Get-Date -Format 'yyyyMMdd-HHmmss'), $Mode.ToLowerInvariant(), ([guid]::NewGuid().ToString('N').Substring(0, 8))
$output = Join-Path $runs $runID
$logDir = Join-Path $output 'logs'
$planarDir = Join-Path $output 'planar'
$isaacDir = Join-Path $output 'isaac'
$summaryPath = Join-Path $output 'run-summary.json'
$summary = [ordered]@{
    mode = $Mode
    status = 'running'
    started_at = [DateTimeOffset]::Now.ToString('o')
    planned_steps = $steps
    steps = [System.Collections.Generic.List[object]]::new()
}

function Save-Summary {
    $pending = "$summaryPath.tmp"
    $summary | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $pending -Encoding utf8
    [IO.File]::Move($pending, $summaryPath, $true)
}

function Invoke-Logged([string]$Name, [string]$Program, [string[]]$Arguments) {
    $log = Join-Path $logDir "$Name.log"
    & $Program @Arguments 2>&1 | ForEach-Object { $_.ToString() } | Tee-Object -FilePath $log | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$Name exited with code $LASTEXITCODE. See $log" }
}

function Invoke-ProjectScript([string]$Name, [string]$Script, [string[]]$Arguments) {
    Invoke-Logged $Name $pwsh (@('-NoLogo', '-NoProfile', '-File', $Script) + $Arguments)
}

function Read-Result([string]$Path) {
    return (Get-Content -LiteralPath $Path -Raw -Encoding utf8 | ConvertFrom-Json)
}

function Invoke-Planar([string]$Profile) {
    if ($Profile -eq 'check') {
        $planarEnv = if ($env:UAV_PLANAR_VENV) { $env:UAV_PLANAR_VENV } else { Join-Path $repo '.venv-planar' }
        $planarEnv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($planarEnv)
        $python = Join-Path $planarEnv 'Scripts\python.exe'
        if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
            throw 'Planar Python is missing. Run the Planar or Full workflow to install dependencies.'
        }
        Invoke-Logged 'planar-check' $python @('-c', 'import platform, gymnasium, numpy, torch, stable_baselines3, matplotlib, uav_nav; print(platform.python_version(), torch.__version__)')
        return (Join-Path $logDir 'planar-check.log')
    }
    Invoke-ProjectScript "planar-$Profile" $planarScript @($Profile)
    if ($Profile -eq 'setup') { return (Join-Path $logDir 'planar-setup.log') }
    $resultName = if ($Profile -eq 'core') { 'test_report.json' } else { 'independent_verification.json' }
    $reportDir = Get-ChildItem -LiteralPath $planarDir -Directory -Filter "$Profile-*" | Select-Object -First 1
    if (-not $reportDir) { throw "No output directory was produced for planar $Profile." }
    $reportFile = Join-Path $reportDir.FullName $resultName
    if ((Read-Result $reportFile).passed -ne $true) { throw "Planar verification failed. See $reportFile" }
    return $reportFile
}

function Invoke-IsaacRun([string]$Name, [string]$Action, [string[]]$Arguments) {
    $resultDir = Join-Path $isaacDir $Name
    Invoke-ProjectScript "isaac-$Name" $isaacScript (@('run', $Action) + $Arguments + @('--output', $resultDir))
    $manifest = Read-Result (Join-Path $resultDir 'manifest.json')
    if ($manifest.status -ne 'completed' -or $manifest.mode -ne $Action) {
        throw "Isaac $Name did not complete. See $resultDir"
    }
    $results = Join-Path $resultDir 'results.json'
    return @{ path = $results; data = (Read-Result $results); manifest = $manifest }
}

function Get-EvaluationCheckpoint {
    if ($Checkpoint) { return $Checkpoint }
    foreach ($prior in (Get-ChildItem -LiteralPath $runs -Directory | Sort-Object Name -Descending)) {
        $training = Join-Path $prior.FullName 'isaac\train'
        $model = Join-Path $training 'ppo.zip'
        $record = Join-Path $training 'manifest.json'
        if ((Test-Path -LiteralPath $model -PathType Leaf) -and (Test-Path -LiteralPath $record -PathType Leaf) -and
            (Read-Result $record).status -eq 'completed') { return $model }
    }
    throw 'No completed PPO checkpoint was found. Run Train or pass -Checkpoint with the path to ppo.zip.'
}

function Invoke-Step([string]$Step) {
    switch -Regex ($Step) {
        '^planar-(.+)$' { return (Invoke-Planar $Matches[1]) }
        '^isaac-prepare$' {
            try { Invoke-ProjectScript 'isaac-check' $isaacScript @('check') }
            catch {
                Write-Host 'Isaac check failed. Installing or repairing the project runtime.'
                Invoke-ProjectScript 'isaac-install' $isaacScript @('install')
                return (Join-Path $logDir 'isaac-install.log')
            }
            return (Join-Path $logDir 'isaac-check.log')
        }
        '^isaac-check$' {
            Invoke-ProjectScript 'isaac-check' $isaacScript @('check')
            return (Join-Path $logDir 'isaac-check.log')
        }
        '^isaac-smoke-(\d+)$' {
            $count = [int]$Matches[1]
            $result = Invoke-IsaacRun "smoke-$count" 'smoke' @('--num-envs', $count)
            $episodes = @($result.data.episodes)
            $invalid = @($episodes | Where-Object { $_.success -ne $true -or $_.reset_to_start -ne $true -or $_.collision -or $_.timeout })
            if ($result.data.passed -ne $true -or $result.manifest.num_envs -ne $count -or $episodes.Count -ne $count -or $invalid.Count) {
                throw "Isaac smoke test did not verify all $count resets. See $($result.path)"
            }
            return $result.path
        }
        '^isaac-demo$' {
            $arguments = @('--num-envs', 5, '--viz', 'kit')
            if ($Mode -eq 'Demo') { $arguments += '--loop' }
            return (Invoke-IsaacRun 'demo' 'demo' $arguments).path
        }
        '^isaac-train$' {
            $result = Invoke-IsaacRun 'train' 'train' @(
                '--num-envs', 64, '--seed', 7, '--timesteps', 303104, '--rollout-steps', 128, '--policy-device', 'cpu'
            )
            $script:modelPath = Join-Path $isaacDir 'train\ppo.zip'
            if ($result.data.timesteps -ne 303104 -or -not (Test-Path -LiteralPath $script:modelPath -PathType Leaf)) {
                throw "PPO did not produce the expected checkpoint. See $($result.path)"
            }
            return $script:modelPath
        }
        '^isaac-(validation|test)$' {
            $suite = $Matches[1]
            $count = if ($suite -eq 'validation') { 20 } else { 60 }
            $result = Invoke-IsaacRun $suite 'evaluate' @('--suite', $suite, '--num-envs', $count, '--checkpoint', $script:modelPath)
            $sha = (Get-FileHash -LiteralPath $script:modelPath -Algorithm SHA256).Hash
            if ($result.manifest.checkpoint_sha256 -ne $sha -or @($result.data.episodes).Count -ne ($count * 3)) {
                throw "Evaluation did not verify the selected checkpoint and paired layouts. See $($result.path)"
            }
            foreach ($controller in 'APF', 'GoalSeeking', 'PPO') {
                $values = $result.data.summary.$controller
                if ($values.episodes -ne $count -or ($values.success + $values.collision + $values.timeout) -ne $count) {
                    throw "Evaluation has an incomplete result for $controller. See $($result.path)"
                }
            }
            return $result.path
        }
        '^isaac-benchmark-(\d+)$' {
            $count = [int]$Matches[1]
            $result = Invoke-IsaacRun "benchmark-$count" 'benchmark' @('--num-envs', $count, '--steps', 256, '--warmup', 32)
            if ($result.data.num_envs -ne $count -or $result.data.vector_steps -ne 256 -or $result.data.transitions_per_second -le 0) {
                throw "Benchmark did not produce valid throughput. See $($result.path)"
            }
            return $result.path
        }
        default { throw "Unknown workflow step $Step" }
    }
}

foreach ($scriptPath in $planarScript, $isaacScript) {
    if (-not (Test-Path -LiteralPath $scriptPath -PathType Leaf)) { throw "Missing project script $scriptPath" }
}
if ($Checkpoint) { $Checkpoint = (Resolve-Path -LiteralPath $Checkpoint).Path }
$previousEula = $env:OMNI_KIT_ACCEPT_EULA
$previousPlanarRuns = $env:UAV_PLANAR_RUNS
$currentStep = 'prepare'
$script:modelPath = $null
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
Save-Summary
Push-Location -LiteralPath $repo
try {
    if ($Mode -eq 'Evaluate') { $script:modelPath = Get-EvaluationCheckpoint }
    if ($Mode -notin @('Check', 'Planar') -and $env:OMNI_KIT_ACCEPT_EULA -ne 'yes') {
        Write-Host 'NVIDIA Omniverse license'
        Write-Host 'https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/NVIDIA_Omniverse_License_Agreement.html'
        if ($NonInteractive -or (Read-Host 'Type YES after reading and accepting the license for this run') -ne 'YES') {
            throw 'NVIDIA license confirmation is required to launch Isaac Sim.'
        }
        $env:OMNI_KIT_ACCEPT_EULA = 'yes'
    }
    $env:UAV_PLANAR_RUNS = $planarDir
    for ($index = 0; $index -lt $steps.Count; $index++) {
        $currentStep = $steps[$index]
        Write-Host "[$($index + 1)/$($steps.Count)] $currentStep"
        $evidence = Invoke-Step $currentStep
        $summary.steps.Add([ordered]@{ step = $currentStep; status = 'completed'; evidence = $evidence })
        Save-Summary
    }
    $summary.status = 'completed'
} catch {
    $summary.status = 'failed'
    $summary.failed_step = $currentStep
    $summary.error = $_.Exception.Message
    [Console]::Error.WriteLine($_.Exception.Message)
} finally {
    $summary.finished_at = [DateTimeOffset]::Now.ToString('o')
    Save-Summary
    $env:OMNI_KIT_ACCEPT_EULA = $previousEula
    $env:UAV_PLANAR_RUNS = $previousPlanarRuns
    Pop-Location
    Write-Host "Run results $output"
    Write-Host "Run summary $summaryPath"
}
if ($summary.status -ne 'completed') { exit 1 }
