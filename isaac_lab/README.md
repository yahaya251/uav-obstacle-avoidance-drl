# Isaac Lab planar multirotor simulation

This track recreates the project's static-layout navigation experiment in Isaac Lab. It instantiates a floor, boundary walls, cylindrical obstacles, a goal marker, and an x500-inspired multirotor in an Isaac Sim USD scene. Isaac Lab manages reset, stepping, parallel scenes and the reinforcement learning interface. The flight model stays at a fixed altitude and applies horizontal kinematics so the comparison uses the same motion contract as the existing benchmark.

The build follows the tutorial's sequence of scene setup, robot geometry, motion and sensing. The task uses an analytic horizontal scan of the same circular geometry used by the scene. This preserves the benchmark's range convention and keeps the scan batched on the task device. [NVIDIA's first robot tutorial](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/building-your-first-robot-in-isaac-sim/index.html)

## What the simulator measures

| Component | Implementation |
| --- | --- |
| Layouts | Seeded open routes, single obstacles, staggered obstacles, corridors and U-shaped traps |
| Observation | 720 rays up to 6 m, pooled into 36 world ENU sectors, plus relative goal and horizontal velocity for 40 values |
| Policy action | Two horizontal velocity components with the shared speed and acceleration limits and proximity stop |
| Episode | Fixed-altitude horizontal motion, swept-disc collision, goal arrival, or timeout |
| Comparisons | Artificial potential field (APF), direct goal seeking and PPO on matched evaluation layouts |
| Performance | A separately timed headless simulation loop. Compare environment steps per second across different batch sizes on the same RTX host. |

The scene uses a simplified visual multirotor. The scan is computed analytically from the layout, and the motion and collision metrics use the planar model. Rotor thrust, six-degree-of-freedom response, RTX LiDAR returns, and PX4 communication require separate validation. The existing [Gazebo and PX4 path](../simulation/README.md) remains the communication test for the project.

## Prerequisites

Use native Windows 11 or Ubuntu 22.04/24.04 x86-64 with an RTX GPU and a supported NVIDIA driver. NVIDIA lists an RTX 4080-class GPU with 16 GB of VRAM and 32 GB of system RAM for the workstation baseline. Allow at least 50 GB of storage for the installation and extensions. The project targets Isaac Lab **3.0.0**, Isaac Sim **6.2.0.0**, Python **3.12**, and the official CUDA 13.0 PyTorch stack. Keep this environment separate from the pinned CPU benchmark and Ubuntu ROS 2 installations. [Isaac Lab installation](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html) · [Isaac Sim 6.2 system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html)

The tensor-level tests also run without Isaac Sim. An RTX workstation can run the headless commands over SSH.

## Install and run on Windows

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) or set `UAV_UV` to its executable, then open PowerShell 7 in the repository root. The launcher puts its managed Python, package cache, isolated Isaac environment, portable Kit state, and Warp kernel cache in `.windows`. The WIA3002 folder layout uses its top-level `.windows` directory. NVIDIA's Python installation guide covers enabling Windows long paths if extraction fails on a long filename. [Isaac Sim Python installation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_python.html)

```powershell
.\isaac_lab\setup.ps1 install
.\isaac_lab\setup.ps1 check
```

Read the [NVIDIA Omniverse license](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/NVIDIA_Omniverse_License_Agreement.html) before launching Isaac Sim. If you accept the terms, you can set `$env:OMNI_KIT_ACCEPT_EULA = 'yes'` in the current PowerShell session for headless runs. The first launch can spend more than ten minutes loading or downloading extensions. [NVIDIA installation notes](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html#installing-the-isaacsim-extra)

```powershell
.\isaac_lab\setup.ps1 run smoke --num-envs 1
.\isaac_lab\setup.ps1 run smoke --num-envs 4
.\isaac_lab\setup.ps1 run demo --num-envs 2 --viz kit
.\isaac_lab\setup.ps1 run train --num-envs 32 --seed 7 --timesteps 32768 --rollout-steps 128 --output runs/isaac/ppo-seed-7
.\isaac_lab\setup.ps1 run evaluate --suite validation --num-envs 20 --checkpoint runs/isaac/ppo-seed-7/ppo.zip
.\isaac_lab\setup.ps1 run evaluate --suite test --num-envs 60 --checkpoint runs/isaac/ppo-seed-7/ppo.zip
foreach ($count in 1, 8, 16, 32, 64) {
    .\isaac_lab\setup.ps1 run benchmark --num-envs $count --steps 256 --warmup 32
}
```

Each smoke run writes `manifest.json`, `layouts.json`, and `results.json` under `runs/isaac`. Check for `"status": "completed"` and `"passed": true` before using the training or throughput commands. For a fresh run, use an unused path after `--output`. Set `UAV_ISAAC_VENV` to select another environment or `UAV_ISAAC_PORTABLE_ROOT` to change where Kit stores logs and cache.

## Install and run on Ubuntu

First check the GPU and driver with `nvidia-smi` on the Ubuntu host and install `uv`. Copy the checkout containing this folder to that host and run the commands from the repository root. The installer creates an isolated `.venv-isaac` and uses NVIDIA's dependency overrides for Isaac Sim 6.2. Review NVIDIA's license when prompted at first launch. [Install uv](https://docs.astral.sh/uv/getting-started/installation/) · [NVIDIA installation notes](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/installation.html#installing-the-isaacsim-extra)

```bash
nvidia-smi
bash isaac_lab/setup.sh install
bash isaac_lab/setup.sh check
isaac_python='.venv-isaac/bin/python'
"$isaac_python" isaac_lab/run.py smoke --num-envs 1
"$isaac_python" isaac_lab/run.py smoke --num-envs 4
```

The smoke test checks finite 40-value observations, open-route motion, arrival and automatic reset. It should produce a completed manifest and `"passed": true` in `results.json`. Two smoke sizes exercise both a single scene and vector stepping.

### Inspect the scene

Use `--viz kit` from an RTX host with an interactive desktop. Headless runs use the default `none`. Two scenes show an open route and a seeded obstacle layout side by side.

```bash
"$isaac_python" isaac_lab/run.py demo --num-envs 2 --viz kit
```

### Train and evaluate on fixed layouts

Use the same checkout and choose fresh output paths for each command. The example uses 32 training scenes and 60 test layouts. Choose fewer scenes to fit VRAM. Set total timesteps to a multiple of `num-envs × rollout-steps` to retain an exact training budget.

```bash
"$isaac_python" isaac_lab/run.py train --num-envs 32 --seed 7 \
  --timesteps 32768 --rollout-steps 128 --output runs/isaac/ppo-seed-7

"$isaac_python" isaac_lab/run.py evaluate --suite validation --num-envs 20 \
  --checkpoint runs/isaac/ppo-seed-7/ppo.zip

"$isaac_python" isaac_lab/run.py evaluate --suite test --num-envs 60 \
  --checkpoint runs/isaac/ppo-seed-7/ppo.zip
```

Each evaluation pairs the controllers on the same layouts and retains one completed episode per layout and controller. Omit `--checkpoint` for the APF and direct-goal baselines. The trainer saves one final fixed-budget checkpoint per run. Keep the test set for final reporting and use only the validation split for training choices. For a formal study, run multiple training seeds and retain every episode.

### Measure rollout throughput

Use the same host, configuration and step counts across runs. Compare `transitions_per_second`, which counts every environment transition. The benchmark uses a direct-goal policy and excludes simulator startup, initial reset, warm-up and output writing.

```bash
for count in 1 8 16 32; do
  "$isaac_python" isaac_lab/run.py benchmark --num-envs "$count" --steps 256 --warmup 32
done
```

Each run saves `manifest.json` with the runtime, device, and status, `layouts.json` with its seeded layouts, and `results.json` with the mode-specific measurements under `runs/isaac/` by default. Training also writes `ppo.zip`. Keep these files together when sharing results.

## Validate the tensor implementation locally

Run this from the repository root with the existing benchmark environment or any compatible Python with PyTorch installed.

```bash
PYTHONPATH=src:isaac_lab python -m unittest discover -s isaac_lab/tests -v
```

For Windows, first run `setup/planar.ps1 setup` from PowerShell, then run the same tensor suite with the planar interpreter.

```powershell
$env:PYTHONPATH = 'src;isaac_lab'
.\.venv-planar\Scripts\python.exe -m unittest discover -s isaac_lab/tests -v
```

These tests compare raw rays and pooled observations with the existing CPU reference, exercise layout isolation and geometry edge cases, and compare the batch action limiter with the scalar limiter. OpenUSD separately verified scene authoring for all five layout families, including the positions and radii of obstacle cylinders and the collision schema on 52 static primitives. The RTX smoke task covers simulator startup, scene initialization and environment stepping in Isaac Sim.

Use the original benchmark Python environment with its benchmark extras to reproduce the CPU-only observation timing. Confirm that the numerical comparison passes before comparing timings.

```bash
PYTHONPATH=src:isaac_lab python isaac_lab/benchmark_geometry.py
```

On the project Mac with one PyTorch CPU thread, the 64-layout check measured 2.83 ms per batch in the reference implementation and 1.98 ms per batch in the Torch implementation. The 32-layout batch showed a similar improvement. For one layout the NumPy reference was faster. These are observation timings on the same fixed layouts. Use the RTX throughput command for the end-to-end Isaac Lab result.
