"""Isaac Lab 3.0 configuration for the fixed-altitude navigation task."""

import gymnasium as gym
import numpy as np
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import (
    CuboidCfg,
    MassCfg,
    PreviewSurfaceCfg,
    SimulationCfg,
    UsdPhysicsCollisionCfg,
    UsdPhysicsRigidBodyCfg,
)
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg


CONTROL_PERIOD_S = 0.1
EPISODE_STEPS = 240
ALTITUDE_M = 1.5
HALF_WORLD_M = 6.0


@configclass
class UAVSceneCfg(InteractiveSceneCfg):
    """Declare the common UAV body. Heterogeneous layouts are authored per arena."""

    vehicle = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/UAV",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, ALTITUDE_M)),
        spawn=CuboidCfg(
            size=(0.18, 0.12, 0.08),
            rigid_props=UsdPhysicsRigidBodyCfg(rigid_body_enabled=True, kinematic_enabled=True),
            collision_props=UsdPhysicsCollisionCfg(collision_enabled=False),
            mass_props=MassCfg(mass=1.0),
            visual_material=PreviewSurfaceCfg(diffuse_color=(0.10, 0.34, 0.67)),
        ),
    )


@configclass
class UAVEnvCfg(DirectRLEnvCfg):
    decimation = 1
    episode_length_s = CONTROL_PERIOD_S * EPISODE_STEPS
    action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
    observation_space = gym.spaces.Box(
        low=np.array([0.0] * 36 + [-1.0] * 4, dtype=np.float32),
        high=np.ones(40, dtype=np.float32),
    )
    state_space = 0
    compute_final_obs = True
    track_episode_outcomes: bool = True
    ui_window_class_type = None
    sim = SimulationCfg(dt=CONTROL_PERIOD_S, render_interval=decimation, physics=PhysxCfg())
    scene = UAVSceneCfg(num_envs=1, env_spacing=16.0, replicate_physics=False)
    layouts: tuple[dict, ...] = ()
