# System Design and Experiment Interpretation

### Execution paths

1. **Offline benchmark.** A 2D kinematic simulator generates 720 horizontal range beams from the layout and advances in 0.1-second steps. The observation builder compresses the scan and appends the relative goal and measured horizontal velocity. PPO, the artificial potential field (APF) and the goal-seeking controller use the same observation and action constraints. The environment applies the guarded action, checks the swept path segment for collision, and records the outcome.
2. **Simulation monitor.** The ROS 2 bridge carries the Gazebo horizontal LiDAR scan, and DDS carries PX4 SITL odometry and vehicle status. The monitor validates the scan and pose, converts coordinate frames, and logs APF velocity recommendations, sensor state and run metrics. A local dashboard displays the monitor output. This is a passive integration path. PX4 manages the flight state independently.

### Observation and action contract

| Indices | Input |
| --- | --- |
| 0–35 | Nearest range in each of 36 ENU sectors of 10° each, starting east and increasing counterclockwise, scaled by 6 m. |
| 36–37 | Goal displacement east and north, scaled by 12 m and clipped. |
| 38–39 | Measured east and north velocity, scaled by 1 m/s and clipped. |

The monitor expects 360° horizontal scan coverage, aligned local origins, and PX4 local NED pose and velocity. Swapping the north and east components and reversing the vertical component converts NED to ENU. Defaults allow a scan or pose reception age of 0.50 s, a scan–pose reception skew of 0.25 s, and absolute roll and pitch up to 15°. Stale or incomplete observations produce a zero-velocity recommendation with a reason.

Actions contain east and north velocity. The normal limits are 1 m/s for speed and 1 m/s² for change in velocity. A range of 0.55 m or less immediately produces a zero-velocity recommendation. The threshold combines the 0.20 m vehicle radius and the 0.35 m margin. The kinematic model applies this stop instantly. Physical stopping distance depends on vehicle dynamics, actuation latency, and sensor error.

### Evaluation protocol and results

The offline controller receives the observation vector. The environment uses ground-truth geometry to generate scans and detect collisions. Reward increases with progress toward the goal and penalizes elapsed steps, velocity changes, and low clearance. Goal completion adds 20 and collision subtracts 20. A run succeeds within the 0.35 m goal radius and times out at 240 steps, equivalent to 24 seconds of simulated time.

The benchmark freezes train, validation and test layouts. PPO uses a two-layer MLP with 64 units per layer on the CPU, and checkpoint selection uses mean return on the fixed validation layouts. All controllers are evaluated on the same test layouts. Each protocol has 60 held-out layouts, five controller configurations including three independent PPO training seeds, and 300 test episodes. See the [archived metrics](../examples/reference_metrics.json) for the recorded settings, outcome counts by layout family, and decision timing.

| Configuration | Exploratory successes | Confirmation successes | Confirmation timeouts |
| --- | ---: | ---: | ---: |
| APF | 48/60 | 48/60 | 12/60 |
| Goal-seeking | 24/60 | 24/60 | 36/60 |
| PPO, training seed 0 | 7/60 | 45/60 | 15/60 |
| PPO, training seed 1 | 0/60 | 47/60 | 13/60 |
| PPO, training seed 2 | 0/60 | 42/60 | 18/60 |

The two archived runs recorded zero collisions across 600 test episodes. On the confirmation layouts, PPO averaged 74.4% success across seeds, with a range of 70.0% to 78.3%. APF achieved 80.0%. All configurations timed out on the 12 confirmation U-trap layouts. The shared proximity stop can end motion early and lead to a timeout. The two runs used different held-out layouts and training budgets, so the comparison is descriptive.

Path efficiency divides the initial straight-line distance minus the goal radius by the distance traveled, for successful episodes only. The timing metric is the median across episodes of each episode's 95th percentile controller decision time. These metrics describe 2D kinematic behavior on static layouts. Training convergence, moving obstacles, closed-loop Gazebo/PX4 performance, and physical flights require separate validation.

## Algorithm references

- PPO algorithm. [Schulman et al. (2017)](https://arxiv.org/abs/1707.06347)
- APF baseline concept. [Khatib (1986)](https://khatib.stanford.edu/publications/pdfs/Khatib_1986_IJRR.pdf)
- PPO library and API. [Stable-Baselines3 2.5.0](https://stable-baselines3.readthedocs.io/en/v2.5.0/modules/ppo.html)
