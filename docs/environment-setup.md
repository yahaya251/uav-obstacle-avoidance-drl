# Development Environment

The planar benchmark runs natively on Windows, macOS, and Ubuntu. The passive simulator runs on Ubuntu 24.04, either directly or in a Lima guest on macOS. Keep the benchmark environment separate from the system Python used by ROS 2.

| Component | Project baseline |
| --- | --- |
| Planar benchmark | Python 3.11 or 3.12; NumPy 1.26.4, Gymnasium 1.0.0, Stable-Baselines3 2.5.0, PyTorch 2.6.0 |
| Isaac Lab simulation | Windows 11 or Ubuntu x86-64; Python 3.12, Isaac Lab 3.0.0, Isaac Sim 6.2.0.0 and CUDA 13.0 PyTorch |
| Plotting | Matplotlib `>=3.8,<4` |
| Ubuntu simulator | Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo Harmonic (Gazebo Sim 8), and `ros_gz_bridge` from Jazzy |
| PX4 and messages | [PX4 Autopilot v1.17.0](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) and [px4_msgs v1.17.0](https://github.com/PX4/px4_msgs/tree/v1.17.0) |
| DDS Agent | [Micro XRCE-DDS Agent v2.4.3](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3) |
| Optional ground station | [QGroundControl v5.1.5](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5) |

The main environment pins are in [`setup/versions.env`](../setup/versions.env). ROS and Gazebo packages come from their distribution repositories; installing on a later date can bring newer patch builds. The verified configuration used Gazebo Sim 8.15.0 and `ros_gz_bridge` 1.0.24. Use the [pinned PX4 v1.17 Ubuntu instructions](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu), [ROS 2 instructions](https://docs.px4.io/v1.17/en/ros2/user_guide), and [Gazebo instructions](https://docs.px4.io/v1.17/en/sim_gazebo_gz/) for reproduction. The current PX4 guides for [development environments](https://docs.px4.io/main/en/dev_setup/dev_env), [ROS 2](https://docs.px4.io/main/en/ros2/user_guide), and [Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/) cover additional combinations; they do not change this project's versions.

## Native Windows

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) or set `UAV_UV` to a portable `uv.exe`. In PowerShell 7, open the repository root and run the commands below.

```powershell
.\setup\planar.ps1 setup
.\setup\planar.ps1 core
.\setup\planar.ps1 smoke
```

The planar runner installs the pinned CPU benchmark into `.venv-planar` and saves verified results and plots under `runs`. You can use `exploratory` and `confirmation` independently, or `full` for setup and all profiles. Set `UAV_PLANAR_VENV` or `UAV_PLANAR_RUNS` to change those paths.

On an RTX workstation, use the [native Windows Isaac Lab setup](../isaac_lab/README.md#install-and-run-on-windows) for the parallel simulator. In a standalone clone, `.windows` holds the managed Python, package cache, CUDA environment, and portable Kit files. Inside the WIA3002 layout, the launchers use the shared `WIA3002/.windows` runtime. Windows GitHub Actions validates the planar workflow and the tensor math on CPU. Run the Isaac Lab smoke command locally to verify Kit and RTX.

Use the Ubuntu setup below for ROS 2, Gazebo, PX4, and the passive telemetry monitor.

## Fresh macOS host

Install [Homebrew](https://brew.sh/) and use its native shell on the Mac. Git and Python are enough for the planar benchmark; install Lima only if you need SITL.

```bash
brew install git python@3.12
git clone https://github.com/yahaya251/uav-obstacle-avoidance-drl.git
cd uav-obstacle-avoidance-drl
python3.12 --version
```

Use the [manual benchmark commands](../README.md#planar-quick-start) or the [planar runner](#planar-runner). A native macOS run does not need Ubuntu, ROS 2, Gazebo, or PX4.

## Planar runner

Run the following from the repository root. The runner looks for Python 3.12, then 3.11. To choose another compatible interpreter, set `UAV_PYTHON` to its path before setup. The runner uses a dedicated benchmark environment rather than the Ubuntu ROS system environment.

```bash
bash setup/planar.sh setup
bash setup/planar.sh core
bash setup/planar.sh smoke
```

Use `bash setup/planar.sh exploratory` and `bash setup/planar.sh confirmation` for individual profiles. `bash setup/planar.sh full` sets up the benchmark environment, then runs the core checks and all three profiles. Training is CPU-based; the confirmation profile takes longer than smoke.

## Create an Ubuntu guest on Apple Silicon

Install [Lima](https://lima-vm.io/docs/installation/) and create a guest from the checked-in configuration. It allocates six virtual CPUs, 8 GiB of memory, and 60 GiB of disk space. It does not mount the Mac checkout; clone separately inside the guest.

```bash
brew install lima
limactl start --name=uav-nav setup/lima.yaml
limactl shell uav-nav
```

Inside the new Ubuntu guest:

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/yahaya251/uav-obstacle-avoidance-drl.git
cd uav-obstacle-avoidance-drl
bash setup/bootstrap-ubuntu.sh install
bash setup/bootstrap-ubuntu.sh check
```

Then use the [SITL start and verification commands](../simulation/README.md#3-start-and-verify) from the guest. Use the Xvfb command unless the guest has a working X11 session. The monitor's TCP port 8086 is loopback-only. [Lima's default port forwarding](https://lima-vm.io/docs/config/port/) can make it available on the Mac at the same local address if the host port is free; it does not verify MAVLink delivery. If Git credentials are unavailable inside the guest, transfer a reviewed source archive into the guest with `limactl copy` instead of copying host credentials.

After stopping the simulator, exit the guest shell and run `limactl stop uav-nav` on the Mac.

## Native Ubuntu

Start on Ubuntu 24.04 LTS and clone the repository into your own checkout. Run `bash setup/bootstrap-ubuntu.sh install`, then `bash setup/bootstrap-ubuntu.sh check`. The [simulation guide](../simulation/README.md) provides the manual dependency commands, a launch and check sequence, and log locations. Training and plotting still use a separate Python virtual environment; open a fresh shell for ROS commands.

## QGroundControl

QGroundControl is optional; neither the benchmark nor the passive SITL acceptance checks depend on it. Install the signed build for your platform from the [official download instructions](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) or the [v5.1.5 release assets](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5).

On macOS, the v5.1.5 build passed signature and notarization checks, launched, and opened UDP 14550. This checks the application and local listener only. MAVLink connectivity between the host and guest was not verified. The simulator's confirmed evidence is from the ROS graph, LiDAR scan and monitor logs, with PX4 disarmed. There is no measured closed-loop PX4 navigation or real-flight outcome in this release.
