# UAV obstacle avoidance project and run guide

This project evaluates local multirotor navigation in unknown static obstacle layouts. It uses a shared observation and motion contract for controller comparisons, and a separate passive simulation to validate the ROS 2, Gazebo, and PX4 data chain. Start each command from the root of the repository checkout unless a step specifies another location.

| Workflow | Purpose | Platform |
| --- | --- | --- |
| Planar benchmark | Train and compare proximal policy optimization (PPO), artificial potential field (APF), and direct goal seeking on matched layouts. Record successes, collisions, timeouts, and trajectories. | Windows, macOS, or Ubuntu |
| Isaac Lab | Reproduce the fixed-altitude planar task in a 3D scene. Verify resets, evaluate PPO against the baselines, and measure parallel throughput. | Windows or Ubuntu x86-64 with a supported NVIDIA RTX GPU |
| Passive software-in-the-loop (SITL) | Verify Gazebo LiDAR, ROS 2 messages, PX4 odometry and status, and recording. PX4 stays disarmed and the monitor records its APF velocity recommendations. | Ubuntu 24.04 |

The Isaac Lab vehicle translates horizontally at 1.5 m. Its 40-value observation combines 36 sectors from analytic LiDAR, the relative goal, and horizontal velocity. The Gazebo workflow receives simulated LiDAR and telemetry through the live bridges. Keep the two sets of results associated with their respective workflows. [System design](design.md)

## Repository layout

| Path | Contents |
| --- | --- |
| `src/uav_nav/` | Observation and action contracts, the planar environment, policies, scenarios, the benchmark, and its verifier |
| `tests/` | Navigation and passive monitor contract tests |
| `setup/windows/` | Windows workflow runner and command launcher |
| `setup/ubuntu-desktop/` | English desktop launchers, simulator lifecycle, recording, and the monitoring dashboard shortcut |
| `setup/` | Planar setup, pinned Ubuntu dependencies, headless capture, and the Lima guest definition |
| `isaac_lab/` | Isaac environment, Windows and Linux setup, scene runner, and tensor tests |
| `simulation/` | Gazebo assets, ROS 2 monitor, dashboard, and independent capture verification |
| `examples/` | A reference benchmark record |
| `runs/`, `.ubuntu-runtime/runs/` | Locally generated experiment results and managed Ubuntu captures |

## 1. Windows setup and full run

Use PowerShell 7 and install `uv` or set `UAV_UV` to the full path to `uv.exe`. The full workflow requires a supported RTX GPU and NVIDIA driver. The planar benchmark uses a dedicated CPU environment under `.venv-planar`. Isaac Lab 3.0.0 uses Python 3.12 and Isaac Sim 6.2.0.0 with CUDA 13.0 in a separate environment. The setup scripts can provision those environments on the first run. [Environment setup](environment-setup.md#native-windows) · [Isaac Lab requirements](../isaac_lab/README.md#prerequisites)

1. Preview the workflow. This shows the stage names and output base before installing dependencies or launching a simulator.

   ```powershell
   .\setup\windows\Run-Windows.cmd -Mode Plan -NonInteractive
   ```

2. Start the full run. The launcher prompts for NVIDIA Omniverse license acceptance when it needs to launch Isaac Sim. First-time installation and the confirmation profile can take longer than a quick check. [NVIDIA Omniverse license](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/NVIDIA_Omniverse_License_Agreement.html)

   ```powershell
   .\setup\windows\Run-Windows.cmd -Mode Full
   ```

3. Wait for all 16 stages to complete. The console prints the results directory and the summary file location.

   | Stages | What runs |
   | --- | --- |
   | 1–2 | Check or install Isaac and set up the pinned planar environment. |
   | 3–6 | Run the core checks, then smoke, exploratory, and confirmation planar benchmarks. Each benchmark includes independent verification. |
   | 7–8 | Start Isaac Sim and verify arrival and automatic reset in 1 and 4 parallel environments. |
   | 9 | Train PPO in 64 environments with seed 7 for 303,104 environment steps. |
   | 10–11 | Evaluate that PPO checkpoint, APF, and direct goal seeking on 20 validation and 60 held-out test layouts. |
   | 12–16 | Measure headless throughput at 1, 8, 16, 32, and 64 environments with the same warm-up and measurement lengths. |

4. Open `runs/windows/<run>/run-summary.json` and confirm `status` is `completed`. Follow each stage's `evidence` path. Read `logs/<step>.log`, `failed_step`, and `error` if a stage fails.

Use `-Mode Quick` for the planar core and smoke checks plus the two Isaac smoke checks. Use `-Mode Planar` for the CPU-only checks and all three planar profiles. The launcher also accepts `-Mode Check`, `-Mode Train`, `-Mode Evaluate`, and `-Mode Benchmark`. Standalone evaluation uses the newest completed managed training run when available. Use `-Checkpoint` to select a specific `ppo.zip`. On macOS or Ubuntu, `bash setup/planar.sh full` runs the planar checks and all three profiles. [Planar runner setup](environment-setup.md#planar-runner)

```powershell
.\setup\windows\Run-Windows.cmd -Mode Check
.\setup\windows\Run-Windows.cmd -Mode Evaluate -Checkpoint 'runs/windows/<run>/isaac/train/ppo.zip'
```

Replace `<run>` with the directory printed by the training or full run.

## 2. Watch the Isaac Lab vehicle move

Start the looping demonstration from a Windows desktop session. The viewport includes an open route, a single obstacle, staggered obstacles, a corridor, and a U-shaped trap. After the initial recorded rollout, the APF controller repeats the navigation in the same window. Each scene resets its vehicle when it reaches the goal, collides, or times out.

```powershell
.\setup\windows\Run-Windows.cmd -Mode Demo
```

The equivalent Isaac command after setup is below.

```powershell
.\isaac_lab\setup.ps1 run demo --num-envs 5 --viz kit --loop
```

Use the Kit toolbar to pause and resume, and close the Kit window to finish. Use `--keep-open` when you want to inspect the final pose after one rollout. `-ShowIsaacDemo` can add a finite demonstration before training in the full Windows workflow. On a supported Linux host, use the Isaac Python environment and the same `demo` arguments. [Isaac Lab launcher](../isaac_lab/README.md)

## 3. Set up the Ubuntu desktop simulation

Use Ubuntu 24.04 with a graphical session and a working display. The project pins ROS 2 Jazzy, Gazebo Harmonic, PX4 and `px4_msgs` v1.17.0, and Micro XRCE-DDS Agent v2.4.3. Use the Ubuntu system Python for the ROS tools. Run setup from a fresh Bash shell. [Ubuntu prerequisites and manual setup](../simulation/README.md#1-install-the-pinned-ubuntu-stack)

```bash
bash setup/bootstrap-ubuntu.sh install
bash setup/bootstrap-ubuntu.sh check
bash setup/ubuntu-desktop/install-desktop.sh --workspace
```

Use the desktop session's **Allow Launching** action if it marks the new shortcuts as untrusted. Deploy again after an update to refresh the managed shortcuts. On Apple Silicon, see the Lima instructions to provision a guest for the Ubuntu workflow. [Apple Silicon guest setup](environment-setup.md#create-an-ubuntu-guest-on-apple-silicon)

| Desktop entry | Action |
| --- | --- |
| **00 Verify Capture and Keep Gazebo Open** | Collect a 45-second sample after readiness, record the live checks in parallel, and independently verify the completed capture. After a pass, start the continuous simulator and open Gazebo. |
| **01 Start UAV Simulator and Gazebo** | Start the continuous simulator and its 3D viewer directly. |
| **02 Check UAV Status and ROS** | Record a health and ROS graph check while the simulator is running. |
| **03 Verify 360° LiDAR** | Record an audit of a complete scan during the active run. |
| **04 Open Gazebo Viewer** | Attach another viewer to the active simulation. |
| **05 Open UAV Dashboard** | Open the English monitoring dashboard. |
| **07 Stop UAV Simulator** | Stop the active run and flush its summary. |
| **08 Capture 45 Seconds with Viewer** | Show Gazebo during capture and verify the result. Use **00** if starting the renderer competes with sensor reception. |
| **09 Open UAV Results** | Open the managed runs directory. |

Launch **00** for the complete Ubuntu workflow. Read `UAV_CAPTURE_RUN_ID` after the audit. The capture and the subsequent continuous simulation have separate run directories. Open **05** after the simulator prints `Ready`. Use **02** and **03** to check the current data while leaving Gazebo open. Stop the continuous simulator with **07** when you finish. [English dashboard](http://127.0.0.1:8086/)

The shell entry point is also available on the Ubuntu desktop. Run `check`, `scan`, or `stop` from a second terminal while `full-gui` runs in the first.

```bash
bash setup/ubuntu-desktop/start.sh full-gui
bash setup/ubuntu-desktop/start.sh check --record
bash setup/ubuntu-desktop/start.sh scan --record
bash setup/ubuntu-desktop/start.sh stop
```

For a terminal-only guest, `bash setup/capture-sitl.sh` runs the timed passive simulation and its independent audit. [Headless capture](../simulation/README.md#3-start-and-verify)

## 4. Verify results and diagnose failures

| Output | What to check |
| --- | --- |
| `runs/windows/<run>/run-summary.json` | Completed stages and evidence locations. Use the stage log to investigate a failure. |
| `runs/windows/<run>/planar/` | Core `test_report.json` and per-profile `summary.json`, `independent_verification.json`, raw episodes, trajectories, and plots. The smoke, exploratory, and confirmation profiles retain 15, 300, and 300 test episodes. |
| `runs/windows/<run>/isaac/` | Per-stage `manifest.json`, `layouts.json`, and `results.json`, plus the training `ppo.zip`. Smoke results show `passed: true` and matching reset checks. Evaluation results retain paired episodes and the checkpoint hash. |
| `.ubuntu-runtime/runs/live-<timestamp>/` | Managed Ubuntu logs, `validation/`, `monitor/status.json`, `monitor/monitor.csv`, and `monitor/monitor.jsonl`. A completed timed run includes the monitor summary and `independent_verification.json`. |

On Windows, run `nvidia-smi` and `.\isaac_lab\setup.ps1 check` to diagnose a CUDA startup issue. Read the failed stage log for a dependency, launch, or verification error. Verify the two smoke cases before retrying training. For a new standalone Ubuntu checkout, the managed desktop paths are relative to that checkout. Use **09** to locate them from the graphical session.

On Ubuntu, use `gazebo-server.log` for simulator startup, `gz-server` rendering, and scene issues. Check `px4.log` and `agent.log` for SITL and DDS attachment, `scan-bridge.log` for LiDAR delivery, and `monitor/status.json` for readiness and sample age. Read `gazebo-gui.log` for viewer startup when the capture passes and Gazebo takes longer to appear. The audit checks 720 rays, full 36-sector coverage, 40 observation values, recent PX4 state, and the passive graph. [Ubuntu diagnostics](../simulation/README.md#3-start-and-verify)
