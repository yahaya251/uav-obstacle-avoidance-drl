# Release history

## 2026-11-26 project release

- Publish the Windows one-click runner with environment checks, three planar profiles, Isaac smoke checks, PPO training, held-out evaluation, and a CUDA throughput sweep.
- Add continuous APF playback across the five Isaac Lab layout families. The Kit window supports pause, resume, and independent episode resets.
- Publish the Ubuntu desktop installer, the verified capture followed by a persistent Gazebo viewer, and one English dashboard. Run live ROS and LiDAR checks in parallel during the timed capture.
- Record the repository structure, commands, evidence files, and recovery procedure in the English project and run guide.

The preceding `main` is commit `dbbd7d6467935b719e83507708762a98d37746d9`. The `rollback/pre-final-2026-11-26` branch pins this snapshot. For a temporary check, check out that branch in a separate working directory. To roll back `main` while preserving history, revert the release commit through a pull request. [Previous release](https://github.com/yahaya251/uav-obstacle-avoidance-drl/commit/dbbd7d6467935b719e83507708762a98d37746d9)

## Previous Windows and Isaac Lab release

- Add native Windows installers and CI, including independent planar and CUDA environments.
- Run fixed-altitude multirotor navigation in Isaac Lab 3.0.0 and Isaac Sim 6.2.0.0 with GPU-batched observations and APF actions.
