# Passive PX4 SITL Simulation

The simulator connects a Gazebo x500 model and horizontal LiDAR to a read-only ROS 2 monitor. Gazebo provides `/uav/scan`; PX4 provides `/fmu/out/vehicle_odometry` and `/fmu/out/vehicle_status_v1` over the Micro XRCE-DDS Agent. The monitor checks these inputs, builds the same 40-value observation used by the planar benchmark, and logs an artificial potential field (APF) velocity recommendation. It has no flight-command publishers; PX4 remains disarmed. Use the [planar benchmark](../README.md#planar-quick-start) for the controller comparison.

## 1. Install the pinned Ubuntu stack

Use Ubuntu 24.04 LTS with ROS 2 Jazzy, Gazebo Harmonic (Gazebo Sim 8), PX4 Autopilot and `px4_msgs` v1.17.0, and Micro XRCE-DDS Agent v2.4.3. On a Mac, use the [Lima guest steps](../docs/environment-setup.md#create-an-ubuntu-guest-on-apple-silicon); this stack runs inside the guest, not in native macOS. Allow room for builds and use the Ubuntu system Python for ROS 2.

See the current PX4 [development environment](https://docs.px4.io/main/en/dev_setup/dev_env), [ROS 2 integration](https://docs.px4.io/main/en/ros2/user_guide), and [Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/) guides for platform requirements. Reproduce this project with the [v1.17 Ubuntu guide](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu), [v1.17 ROS 2 guide](https://docs.px4.io/v1.17/en/ros2/user_guide), and [v1.17 Gazebo guide](https://docs.px4.io/v1.17/en/sim_gazebo_gz/). Newer combinations described by PX4 are outside this repository's verification.

From the repository root in a fresh Bash shell, use the installer or follow the manual steps below. The installer runs Ubuntu package installation and builds under the default user prefix. Its `check` mode validates the existing setup without installing anything.

```bash
bash setup/bootstrap-ubuntu.sh install
bash setup/bootstrap-ubuntu.sh check
```

For another installation prefix, export `UAV_SIM_DEPS` to the same directory in the install, launch, and verification shells. An existing incompatible dependency checkout should not be mixed with the pinned stack; use a separate prefix or repair the existing one.

## 2. Manual installation

### ROS 2 and Gazebo

Configure locales, enable the official ROS package source for Noble, then install ROS 2 Jazzy, its Gazebo bridge, and build and display dependencies. The ROS package source follows the official [Jazzy Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) ([documentation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)).

```bash
sudo apt update
sudo apt install -y curl ca-certificates locales software-properties-common
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
ROS_APT_TAG="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
ros_apt_deb="$(mktemp --suffix=.deb)"
curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_TAG}/ros2-apt-source_${ROS_APT_TAG}.noble_all.deb" -o "$ros_apt_deb"
sudo dpkg -i "$ros_apt_deb"
rm -f "$ros_apt_deb"
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-dev-tools \
  python3-pip python3-venv python3-numpy cmake build-essential git bc \
  cppzmq-dev libunwind-dev xvfb xauth libgl1-mesa-dri
```

### PX4 and the DDS Agent

Clone [PX4 Autopilot v1.17.0](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) with matching submodules, then run the pinned checkout's Ubuntu setup with the physical firmware toolchain disabled. The build target is the simulator binary. Compile and install [Micro XRCE-DDS Agent v2.4.3](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3) in the same dependency prefix. See [PX4 v1.17 DDS integration](https://docs.px4.io/v1.17/en/middleware/uxrce_dds) for background.

```bash
export UAV_SIM_DEPS="${UAV_SIM_DEPS:-$HOME/.local/share/uav-nav-sim}"
mkdir -p "$UAV_SIM_DEPS"
git clone --depth 1 --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git "$UAV_SIM_DEPS/PX4-Autopilot"
git -C "$UAV_SIM_DEPS/PX4-Autopilot" submodule update --init --recursive --depth 1
bash "$UAV_SIM_DEPS/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx
make -C "$UAV_SIM_DEPS/PX4-Autopilot" px4_sitl_default -j 4

git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$UAV_SIM_DEPS/Micro-XRCE-DDS-Agent"
cmake -S "$UAV_SIM_DEPS/Micro-XRCE-DDS-Agent" -B "$UAV_SIM_DEPS/agent-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$UAV_SIM_DEPS/xrce"
cmake --build "$UAV_SIM_DEPS/agent-build" -j 2
cmake --install "$UAV_SIM_DEPS/agent-build"
```

If the guest runs out of memory during a build, reduce the number of parallel jobs to two or one.

### Matching ROS message overlay

Keep [the message definitions](https://github.com/PX4/px4_msgs/tree/v1.17.0) at the same release as PX4. Build with the Jazzy shell environment; do not activate the benchmark virtual environment.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$UAV_SIM_DEPS/ros_ws/src"
git clone --depth 1 --branch v1.17.0 https://github.com/PX4/px4_msgs.git "$UAV_SIM_DEPS/ros_ws/src/px4_msgs"
(cd "$UAV_SIM_DEPS/ros_ws" && PYTHONNOUSERSITE=1 colcon build --symlink-install)
bash setup/bootstrap-ubuntu.sh check
```

The launcher sources Jazzy and the message overlay automatically. For an existing compatible setup, `PX4_ROOT`, `PX4_MSGS_SETUP`, `XRCE_AGENT`, `XRCE_LIB_DIR`, and `ROS_SETUP` can override individual paths. Set `UAV_XRCE_SOURCE` for an existing Agent checkout when its binary does not report a version. The ROS commands use DDS domain 0 and local discovery. Run one simulation at a time; it uses DDS on UDP 8888 and serves the dashboard on loopback TCP 8086.

## 3. Start and verify

To start the headless passive simulator, collect 45 seconds after readiness, and run the independent capture audit in one command:

```bash
bash setup/capture-sitl.sh
```

The command records ROS and LiDAR checks while the run is active. Look for `Verified passive capture:` and `independent_verification.json` in the printed output directory. Recheck a completed capture with `/usr/bin/python3 simulation/verify_capture.py path/to/run`. For an interactive demonstration use the commands below.

Run from the repository root. Use the first command on an Ubuntu desktop with X11, or the second on headless Ubuntu or Lima. The headless command provides a virtual X11 display and enables software rendering for the LiDAR.

```bash
bash simulation/run.sh start
# Or, for Ubuntu without an X11 session:
xvfb-run -a env UAV_SOFTWARE_GL=1 bash simulation/run.sh start
```

The launcher generates and validates the scene, starts Gazebo, attaches PX4 to the x500, starts the DDS Agent and LiDAR bridge, then starts the monitor. After it prints `Ready`, run the following in a second Ubuntu terminal at the repository root, using the same dependency prefix and any custom Gazebo partition:

```bash
bash simulation/run.sh check
bash simulation/run.sh scan
```

- `check` validates fresh topics, 720 rays, 36 covered sectors, a 40-value observation, a recent disarmed PX4 status and the passive monitor's ROS graph.
- `scan` checks angles and range validity from one complete Gazebo scan.

Open the [local dashboard](http://127.0.0.1:8086/?lang=en). On an Ubuntu desktop with X11, `bash simulation/run.sh gui` attaches a Gazebo window. Use the browser dashboard when running under Xvfb.

Press Ctrl+C in the terminal that started a continuous run to close its processes and flush the monitor summary. Its interrupt exit status is 130. For a timed capture, start a separate run with `--duration`. The timer starts after readiness; start the checks while that run is active.

```bash
xvfb-run -a env UAV_SOFTWARE_GL=1 bash simulation/run.sh start --duration 45
```

The default output is a new `runs/live-<UTC-timestamp>/` directory. Use `--output <new-directory>` to choose another location.

| Output | Purpose |
| --- | --- |
| `assets/`, `sdf-validation.log`, `gazebo-server.log` | Generated scene and sensor geometry; scene-validation and renderer diagnostics |
| `agent.log`, `px4.log`, `scan-bridge.log`, `monitor-node.log` | Component start, attachment and data-flow diagnostics |
| `monitor/status.json`, `monitor/monitor.csv`, `monitor/monitor.jsonl` | Current status and sample history |
| `monitor/run_summary.json`, `monitor/monitor_summary.json` | Counters and summaries written during orderly exit |
| `validation/`, `independent_verification.json` | Saved ROS graph and scan checks and the independent capture audit |

For startup failures, check `gazebo-server.log` for display and renderer errors, `scan-bridge.log` for bridge errors, and `agent.log` and `px4.log` for DDS and model attachment. If the dashboard loads but checks fail, read its readiness reasons and confirm that the PX4 firmware and message overlay match. If port forwarding fails in Lima, check the guest dashboard first, then the corresponding host port.

[QGroundControl](../docs/environment-setup.md#qgroundcontrol) is optional and is not part of the monitor acceptance checks. Closed-loop navigation, moving obstacles, and physical flights have not been validated.
