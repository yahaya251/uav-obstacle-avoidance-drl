# Deep Reinforcement Learning for Autonomous UAV Obstacle Avoidance in Unknown Environments

**未知环境中的无人机深度强化学习避障**

[中文](#中文) · [English](#english) · [System design / 系统设计](docs/design.md) · [Simulation / 仿真接入](simulation/README.md)

## 中文

项目有两条独立路径。二维运动学基准使用模拟水平 LiDAR、相对目标和水平速度训练 PPO，并在相同的保留静态布局上与人工势场（APF）及直趋目标控制器比较。Ubuntu 仿真接入读取 ROS 2、Gazebo 和 PX4 SITL 的数据，在本地仪表板显示传感器状态、观测及 APF 推荐速度。

使用 Python 3.11 或 3.12。下方命令以 3.11 为例，在仓库根目录运行。基础测试只需 Python 标准库，训练和绘图需额外依赖。每次运行请选择新的输出目录。

1. 克隆并安装项目，运行导航和监测契约测试及四个确定性二维场景。
2. 安装训练和绘图依赖。用 `smoke` 跑通训练、验证集检查点选择、保留测试集评价、独立核验和绘图。
3. 使用 `exploratory` 或 `confirmation` 运行正式基准。查看汇总时同时关注成功、碰撞、超时和停止次数，并单独解读成功路径效率与决策延迟。
4. 如需在 Ubuntu 24.04 启动被动仿真监测，按 [仿真指南](simulation/README.md) 安装和检查。仿真接入保持 PX4 解锁前状态，只读数据并生成推荐值。PPO 闭环、运动障碍和实体飞行仍待验证。

## English

The project has two execution paths. The planar kinematic benchmark trains PPO from simulated horizontal LiDAR, relative goal position, and horizontal velocity. It compares PPO with artificial potential field (APF) and goal-seeking controllers on identical held-out static layouts. The Ubuntu integration reads ROS 2, Gazebo, and PX4 SITL data and exposes sensor health, observations, and APF velocity recommendations on a local dashboard.

Use Python 3.11 or 3.12. The commands below use Python 3.11 and run from the repository root. The core checks need only the standard library. Training and plotting use optional dependencies. Give every run a fresh output directory.

1. Clone and install the project. Run the navigation and monitor contract tests and the four deterministic planar scenarios.
2. Install the benchmark and plot extras. Run `smoke` to train a policy, select a checkpoint on validation layouts, evaluate held-out layouts, audit the output, and generate plots.
3. Run `exploratory` or `confirmation` for the full benchmark. Interpret successes, collisions, timeouts, and stops together. Report successful-path efficiency and decision latency separately.
4. Follow the [simulation guide](simulation/README.md) to start and check passive monitoring on Ubuntu 24.04. This integration requires a disarmed PX4 instance and produces read-only recommendations. Closed-loop PPO in SITL, moving obstacles, and physical flights remain to be validated.

## Run / 运行

```bash
git clone https://github.com/yahaya251/uav-obstacle-avoidance-drl.git
cd uav-obstacle-avoidance-drl
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m uav_nav.checks --output runs/core-checks

python -m pip install -e '.[benchmark,plots]'
python -m uav_nav.benchmark run --profile smoke --output runs/smoke
python -m uav_nav.benchmark verify runs/smoke
python -m uav_nav.benchmark plot runs/smoke
```

On Linux x86-64, install the CPU build first to avoid downloading CUDA libraries for a CPU benchmark. / 在 Linux x86-64 上，可先安装 CPU 版 PyTorch，避免为 CPU 实验下载 CUDA 库。

```bash
python -m pip install torch==2.6.0 --index-url https://download.pytorch.org/whl/cpu
```

| Profile / 配置 | Steps per PPO seed / 每种子训练步数 | PPO seeds / 种子数 | Held-out layouts / 保留布局数 |
| --- | ---: | ---: | ---: |
| `smoke` | 1,024 | 1 | 5 |
| `exploratory` | 32,768 | 3 | 60 |
| `confirmation` | 300,032 | 3 | 60 |

```bash
python -m uav_nav.benchmark run --profile exploratory --output runs/exploratory
python -m uav_nav.benchmark verify runs/exploratory
python -m uav_nav.benchmark run --profile confirmation --output runs/confirmation
python -m uav_nav.benchmark verify runs/confirmation
python -m uav_nav.benchmark plot runs/confirmation
```

## Files / 文件

| Path / 路径 | Purpose / 用途 |
| --- | --- |
| `src/uav_nav/core.py` | Observation builder, APF and action limits / 观测构建、人工势场及动作约束 |
| `src/uav_nav/kinematics.py` | Seeded planar worlds, simulated sensor and environment / 带种子的二维场景、模拟传感器及环境 |
| `src/uav_nav/benchmark/` | PPO training, paired evaluation, verification and plots / PPO 训练、配对评价、核验与绘图 |
| `tests/` | Navigation and passive-monitor contract tests / 导航与被动监测契约测试 |
| `simulation/` | Ubuntu setup, headless launch, graph checks and dashboard / Ubuntu 配置、无界面启动、节点图检查及仪表板 |
| `examples/reference_metrics.json` | Archived configuration and aggregate results / 历史实验配置及汇总结果 |
| `docs/design.md` | Architecture, metrics, archived results and interpretation / 架构、指标、历史结果及解读 |

| Run output / 运行产物 | How to read it / 如何使用 |
| --- | --- |
| `run_manifest.json`, `frozen_splits.json` | Versions, settings and exact layouts / 软件版本、运行配置及固定布局 |
| `ppo_seed_*/validation_scores.json`, `ppo_seed_*/best_validation.zip` | Checkpoint scores and the model selected on validation layouts / 验证集评分与选中的模型 |
| `summary.json`, `test_episode_results.csv` | Outcome and timing summaries, plus one row per held-out episode / 结果、耗时汇总及各测试回合的明细 |
| `test_trajectories.csv.gz` | Stepwise positions, guarded actions and stop reasons / 每步位置、限幅后动作及停止原因 |
| `independent_verification.json`, plot files | Audit results from `verify` and charts from `plot` / `verify` 的核验结果和 `plot` 的图表 |

The archived confirmation benchmark recorded APF at 48/60 successes and the three PPO seeds at 45/60, 47/60, and 42/60. Every controller timed out in all 12 confirmation U-trap layouts. See the [design notes](docs/design.md) for metric definitions and the conditions behind these results.

历史确认实验中，APF 成功 48/60，三个 PPO 种子分别成功 45/60、47/60 和 42/60。所有控制器在该次实验的 12 个 U 形陷阱布局中均超时。指标定义和实验条件见[系统设计](docs/design.md)。
