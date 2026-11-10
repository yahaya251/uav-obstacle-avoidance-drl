# Passive PX4 SITL integration / PX4 SITL 被动仿真

The obstacle scene connects an x500 model, a Gazebo LiDAR, PX4 telemetry, ROS 2 and a browser dashboard. The ROS node subscribes to telemetry and computes a dry-run artificial potential field (APF) recommendation. The vehicle stays disarmed. Use the separate 2D benchmark for PPO training and APF-versus-PPO evaluation.

障碍场景将 x500 模型、Gazebo 激光雷达、PX4 遥测、ROS 2 与浏览器面板连在一起。ROS 节点订阅遥测并计算人工势场（APF）建议速度，飞行器保持未解锁。PPO 训练与 APF、PPO 对照评估使用独立的二维基准。

## 1. Install on Ubuntu / 在 Ubuntu 内安装

Target Ubuntu 24.04 (Noble) with ROS 2 Jazzy, Gazebo Harmonic, PX4 and `px4_msgs` v1.17.0, and Micro XRCE-DDS Agent v2.4.3. Run these commands in a fresh Ubuntu shell or a dedicated VM. Allow disk space for the C++ build and keep all ROS runtime commands in the system Python environment. [ROS Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [PX4 Gazebo](https://docs.px4.io/v1.17/en/sim_gazebo_gz/), [PX4 uXRCE-DDS](https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html)

目标平台为 Ubuntu 24.04（Noble）、ROS 2 Jazzy、Gazebo Harmonic、PX4 与 `px4_msgs` v1.17.0，以及 Micro XRCE-DDS Agent v2.4.3。在新的 Ubuntu 终端或专用虚拟机里执行。为 C++ 构建预留磁盘空间，ROS 运行命令使用系统 Python 环境。[ROS Jazzy Ubuntu 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)、[PX4 Gazebo](https://docs.px4.io/v1.17/en/sim_gazebo_gz/)、[PX4 uXRCE-DDS](https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html)

### ROS 2 and Gazebo / 安装 ROS 2 与 Gazebo

Enable Ubuntu Universe, register the official ROS apt repository, then install the Jazzy tools and bridge. The release lookup selects the current apt-source package for Noble. Install Xvfb and Mesa for a virtual display and software rendering.

启用 Ubuntu Universe，注册官方 ROS apt 仓库，然后安装 Jazzy 工具和消息桥。发布信息查询用于选择当前 Noble 版 apt-source 软件包。安装 Xvfb 和 Mesa 以支持虚拟显示和软件渲染。

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

### PX4, DDS Agent and ROS message overlay / 构建 PX4、DDS Agent 和 ROS 消息工作空间

PX4’s official setup script installs its build dependencies and Gazebo integration. The PX4 and message tags must match. The three builds use a user-owned prefix and can be run separately. A small VM can lower `-j 4` to `-j 2` or `-j 1` if the compiler is terminated for lack of memory. [PX4 Ubuntu build environment](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu.html), [PX4 firmware v1.17.0](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0), [PX4 messages v1.17.0](https://github.com/PX4/px4_msgs/tree/v1.17.0), [DDS Agent v2.4.3](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3)

PX4 官方脚本会安装构建依赖和 Gazebo 集成组件。PX4 与消息包的标签必须匹配。三项构建都使用当前用户的目录，可以分开执行。如果虚拟机因内存不足终止编译器，可将 `-j 4` 改为 `-j 2` 或 `-j 1`。[PX4 Ubuntu 构建环境](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu.html)、[PX4 固件 v1.17.0](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0)、[PX4 消息 v1.17.0](https://github.com/PX4/px4_msgs/tree/v1.17.0)、[DDS Agent v2.4.3](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.3)

```bash
export UAV_SIM_DEPS="${UAV_SIM_DEPS:-$HOME/.local/share/uav-nav-sim}"
mkdir -p "$UAV_SIM_DEPS"
git clone --depth 1 --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git "$UAV_SIM_DEPS/PX4-Autopilot"
git -C "$UAV_SIM_DEPS/PX4-Autopilot" submodule update --init --recursive --depth 1
bash "$UAV_SIM_DEPS/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx
make -C "$UAV_SIM_DEPS/PX4-Autopilot" px4_sitl_default -j 4
```

```bash
git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$UAV_SIM_DEPS/Micro-XRCE-DDS-Agent"
cmake -S "$UAV_SIM_DEPS/Micro-XRCE-DDS-Agent" -B "$UAV_SIM_DEPS/agent-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$UAV_SIM_DEPS/xrce"
cmake --build "$UAV_SIM_DEPS/agent-build" -j 2
cmake --install "$UAV_SIM_DEPS/agent-build"
```

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$UAV_SIM_DEPS/ros_ws/src"
git clone --depth 1 --branch v1.17.0 https://github.com/PX4/px4_msgs.git "$UAV_SIM_DEPS/ros_ws/src/px4_msgs"
(cd "$UAV_SIM_DEPS/ros_ws" && PYTHONNOUSERSITE=1 colcon build --symlink-install)
```

For existing installations, set `UAV_SIM_DEPS` consistently in each launch and validation terminal. You can also set `PX4_ROOT`, `PX4_MSGS_SETUP`, `XRCE_AGENT`, `XRCE_LIB_DIR`, and `ROS_SETUP` to their absolute paths. The DDS Agent and monitor use UDP 8888 and loopback HTTP 8086 respectively. Run a single instance on the VM. The ROS commands use the stock PX4 DDS domain 0. Set the same `GZ_PARTITION` in each terminal if you override its default.

已有安装可在启动和验证终端中设置同一个 `UAV_SIM_DEPS`。也可以分别用绝对路径设置 `PX4_ROOT`、`PX4_MSGS_SETUP`、`XRCE_AGENT`、`XRCE_LIB_DIR` 和 `ROS_SETUP`。DDS Agent 使用 UDP 8888，监测面板使用本机回环地址的 HTTP 8086。在虚拟机中运行单实例。ROS 命令使用 PX4 默认的 DDS 域 0。若修改 `GZ_PARTITION`，请在各终端设置相同的值。

## 2. Run and verify / 启动和验证

From the repository root, use the first command in an Ubuntu graphical session with a working X11 display. On headless Ubuntu or a VM with software graphics, use the second command. Start from a new shell without an active Python virtual environment.

在仓库根目录执行。Ubuntu 图形会话已有可用 X11 显示时，使用第一条命令。无桌面 Ubuntu 或使用软件显卡的虚拟机使用第二条命令。请打开未激活 Python 虚拟环境的新终端。

```bash
bash simulation/run.sh start
```

```bash
xvfb-run -a env UAV_SOFTWARE_GL=1 bash simulation/run.sh start
```

The launcher generates and validates the SDF, starts Gazebo, attaches PX4 to the existing x500, exposes PX4 telemetry via DDS, bridges LiDAR from Gazebo to ROS 2, then starts the dashboard. Wait for `Ready` before checking. In a second Ubuntu terminal at the repository root, run the following commands. The first checks fresh 40-value observations, 36 covered sectors, 720 rays, a recent disarmed PX4 status and the monitor’s ROS graph. The second reads one complete scan and checks angles and ray validity.

启动器先生成并验证 SDF，再启动 Gazebo、让 PX4 连接场景中已有的 x500 并通过 DDS 输出遥测、建立从 Gazebo 到 ROS 2 的雷达消息桥，最后启动面板。看到 `Ready` 后，在另一个 Ubuntu 终端的仓库根目录执行以下命令。第一条检查最新的 40 维观测、36 个有效扇区、720 条射线、最近的 PX4 未解锁状态及监测节点的 ROS 图。第二条读取一帧完整扫描，检查扫描角度和射线有效性。

```bash
bash simulation/run.sh check
bash simulation/run.sh scan
```

Open the [English dashboard](http://127.0.0.1:8086/?lang=en) or [Chinese dashboard](http://127.0.0.1:8086/?lang=zh). To attach a Gazebo window on a graphical Ubuntu desktop, run `bash simulation/run.sh gui` in a terminal with X11 access. With Xvfb only, use the browser dashboard.

打开[英文面板](http://127.0.0.1:8086/?lang=en)或[中文面板](http://127.0.0.1:8086/?lang=zh)。如需在 Ubuntu 图形桌面连接 Gazebo 窗口，在具有 X11 访问权限的终端中执行 `bash simulation/run.sh gui`。仅使用 Xvfb 时，通过浏览器查看面板。

Press Ctrl+C in the original terminal to stop the launched services and flush the monitor summary. For a fixed collection time, append `--duration 30` to the start command to collect 30 seconds after readiness. The launcher creates a fresh `runs/live-<UTC timestamp>/`; `--output <new-directory>` chooses a different output location.

在原启动终端按 Ctrl+C 停止本次启动的服务并写入监测摘要。需要固定采样时长时，在启动命令末尾追加 `--duration 30`，就绪后采集 30 秒。启动器会新建 `runs/live-<UTC 时间戳>/`。如需另选输出位置，可使用 `--output <新目录>`。

| Output / 输出 | Purpose / 用途 |
| --- | --- |
| `assets/` | Generated scene, sensor model and reference geometry / 生成的场景、传感器模型及参考几何 |
| `sdf-validation.log`, `gazebo-server.log` | SDF parsing, model and renderer errors / SDF 解析、模型和渲染器诊断 |
| `agent.log`, `px4.log`, `scan-bridge.log`, `monitor-node.log` | Component startup and diagnostics / 组件启动与诊断 |
| `monitor/status.json`, `monitor/monitor.csv`, `monitor/monitor.jsonl` | Latest snapshot and sample history / 最新快照和采样历史 |
| `monitor/run_summary.json`, `monitor/monitor_summary.json` | Run counters and summaries written on exit / 退出时写入的运行计数与摘要 |

If startup times out, inspect `gazebo-server.log` for display/renderer errors, `scan-bridge.log` for LiDAR bridge issues, and `agent.log` and `px4.log` for DDS and model attachment. If the dashboard loads but `check` fails, inspect the rejection reasons on the dashboard. For persistent odometry or status failures, confirm the firmware and message tags, then rerun with a matching overlay.

若启动超时，查看 `gazebo-server.log` 中的显示或渲染器错误、`scan-bridge.log` 中的雷达桥接问题，以及 `agent.log` 和 `px4.log` 中的 DDS 与模型连接信息。若面板能打开但 `check` 失败，查看面板显示的拒绝原因。若里程计或状态持续缺失，核对固件与消息包标签，再用匹配的工作空间重试。

## 3. Optional macOS VM / 可选 macOS 虚拟机

On macOS with Lima installed, create a dedicated Ubuntu instance and enter it. Clone this repository into the guest, follow sections 1 and 2 inside Ubuntu, then open the loopback dashboard in a browser on the Mac. Lima’s default networking forwards guest loopback TCP ports to the host. If the host port is occupied, resolve the conflict or open the dashboard from the guest instead. [Lima user-mode networking](https://lima-vm.io/docs/config/network/user/)

在已安装 Lima 的 macOS 上，可创建并进入独立的 Ubuntu 实例。将本仓库克隆到虚拟机，在 Ubuntu 内完成第 1、2 节，然后在 Mac 浏览器打开回环地址的监测面板。Lima 默认网络会转发虚拟机的回环 TCP 端口到宿主机。如宿主机端口被占用，可处理冲突或从虚拟机内打开面板。[Lima 用户态网络](https://lima-vm.io/docs/config/network/user/)

```bash
limactl start --name=uav-nav --cpus=4 --memory=8 --disk=45 template:ubuntu-24.04
limactl shell uav-nav
```

Use the Xvfb launch command in Lima. To shut down the guest after stopping the simulation, exit the guest shell and run `limactl stop uav-nav` in macOS. The graphical window option requires a separate Ubuntu desktop session with X11.

在 Lima 内使用 Xvfb 启动命令。停止仿真后，退出虚拟机终端，在 macOS 中执行 `limactl stop uav-nav` 关闭虚拟机。图形窗口选项需要具有 X11 的 Ubuntu 桌面会话。
