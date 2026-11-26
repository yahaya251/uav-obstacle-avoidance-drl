# UAV Obstacle Avoidance in Unknown Static Layouts

This research prototype compares proximal policy optimization (PPO), an artificial potential field (APF), and direct goal seeking on identical planar obstacle layouts. A separate software-in-the-loop (SITL) stack checks simulated LiDAR and flight-controller telemetry. The SITL monitor logs APF velocity recommendations while PX4 stays disarmed; it does not send flight commands or evaluate closed-loop avoidance.

Use the repository guide to run the full Windows workflow, set up the Ubuntu desktop, or open the looping Isaac Lab demonstration. [Project and run guide](docs/project-and-run-guide.md) · [Release history](CHANGELOG.md)

| What to run | Where | Start here |
| --- | --- | --- |
| Core checks and planar benchmark | Windows, macOS, or Ubuntu, Python 3.11 or 3.12 | [Planar quick start](#planar-quick-start) |
| Passive SITL | Ubuntu 24.04, either native or a Lima guest on Apple Silicon | [Ubuntu installation and verification](simulation/README.md) |
| Parallel Isaac Lab navigation research | Windows 11 or Ubuntu x86-64 with a supported NVIDIA RTX GPU | [Isaac Lab setup, evaluation and throughput](isaac_lab/README.md) |
| Fresh Windows workstation setup | Native PowerShell and separate planar and CUDA runtimes | [Windows environment setup](docs/environment-setup.md#native-windows) |
| Complete Windows validation | PowerShell 7 with the planar and RTX runtimes | [`setup/windows/Run-Windows.cmd`](setup/windows/Run-Windows.cmd) |
| Ubuntu graphical SITL | Ubuntu 24.04 desktop with Gazebo and ROS 2 | [Ubuntu desktop workflow](docs/project-and-run-guide.md#3-set-up-the-ubuntu-desktop-simulation) |
| Fresh Apple Silicon Mac setup | macOS for the benchmark; Lima for the Ubuntu simulator | [Environment setup](docs/environment-setup.md) |

## Planar quick start

On Windows, install [uv](https://docs.astral.sh/uv/getting-started/installation/) and run the PowerShell 7 entry point from the repository root.

```powershell
.\setup\planar.ps1 setup
.\setup\planar.ps1 core
.\setup\planar.ps1 smoke
```

On macOS and Ubuntu, use the shell entry point below.

With Python and Git installed, run `bash setup/planar.sh full` for setup, core checks and all three profiles. For a short first pass, use `bash setup/planar.sh setup` and `bash setup/planar.sh smoke`. The equivalent manual steps follow.

Install Git and Python 3.11 or 3.12. On macOS, [Homebrew's Python 3.12 formula](https://formulae.brew.sh/formula/python@3.12) is one option; on Ubuntu 24.04 install `git`, `python3`, `python3-pip`, and `python3-venv`. These commands run from a repository checkout. Use a fresh output directory for each run.

```bash
git clone https://github.com/yahaya251/uav-obstacle-avoidance-drl.git
cd uav-obstacle-avoidance-drl
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m uav_nav.checks --output runs/core-checks
```

The core checks run the navigation and passive-monitor contract tests and four deterministic planar scenarios. They require no training dependency. On Linux x86-64, install the PyTorch CPU wheel before installing the benchmark extras; on macOS and Linux ARM64, use the appropriate wheel from the package index.

```bash
# Linux x86-64 only:
python -m pip install torch==2.6.0 --index-url https://download.pytorch.org/whl/cpu
```

```bash
python -m pip install -e '.[benchmark,plots]'
python -m uav_nav.benchmark run --profile smoke --output runs/smoke
python -m uav_nav.benchmark verify runs/smoke
python -m uav_nav.benchmark plot runs/smoke
```

The benchmark pins NumPy 1.26.4, Gymnasium 1.0.0, Stable-Baselines3 2.5.0, and PyTorch 2.6.0. Plotting uses Matplotlib `>=3.8,<4`. See [environment setup](docs/environment-setup.md) for the shell shortcuts and simulator versions.

| Profile | Training steps per PPO seed | PPO seeds | Held-out layouts | Test episodes |
| --- | ---: | ---: | ---: | ---: |
| `smoke` | 1,024 | 1 | 5 | 15 |
| `exploratory` | 32,768 | 3 | 60 | 300 |
| `confirmation` | 300,032 | 3 | 60 | 300 |

```bash
python -m uav_nav.benchmark run --profile exploratory --output runs/exploratory
python -m uav_nav.benchmark verify runs/exploratory
python -m uav_nav.benchmark plot runs/exploratory

python -m uav_nav.benchmark run --profile confirmation --output runs/confirmation
python -m uav_nav.benchmark verify runs/confirmation
python -m uav_nav.benchmark plot runs/confirmation
```

Each profile selects PPO checkpoints on validation layouts before evaluating the test set. The profiles differ in training budget and may use different test layouts, so compare controllers within a profile. The archived confirmation benchmark recorded 48 of 60 successes for APF, 24 for direct goal seeking, and 45, 47, and 42 for the three PPO seeds. All five controller configurations timed out on the 12 U-shaped layouts. See the [system design](docs/design.md) for the test definitions and limits of these results.

## Repository map

| Path | Purpose |
| --- | --- |
| `src/uav_nav/core.py` | Shared observation builder, APF, and action limiting |
| `src/uav_nav/kinematics.py` | Seeded planar worlds, simulated sensor, and kinematic environment |
| `src/uav_nav/benchmark/` | PPO training, paired evaluation, independent verification, and plots |
| `tests/` | Navigation and passive-monitor contract tests |
| `simulation/` | Ubuntu SITL launcher, sensor bridge, passive monitor, and dashboard |
| `setup/` | Planar runner, Ubuntu bootstrap, one-command SITL capture, pinned versions, and Lima configuration |
| `isaac_lab/` | Optional Isaac Sim scene, vectorized sensing and motion, PPO, paired evaluation, and throughput checks |
| `examples/reference_metrics.json` | Archived benchmark configuration and aggregate results |

| Run output | What it records |
| --- | --- |
| `run_manifest.json`, `frozen_splits.json` | Runtime and dependency versions, algorithm settings, source hashes, and exact layouts |
| `ppo_seed_*/validation_scores.json`, `ppo_seed_*/best_validation.zip` | Validation scores and selected checkpoint for each PPO seed |
| `summary.json`, `test_episode_results.csv` | Aggregate results and one row per held-out episode |
| `test_trajectories.csv.gz` | Stepwise positions, executed actions, and constraint reasons |
| `independent_verification.json` and plot files | Independent audit and visualization of the benchmark records |
