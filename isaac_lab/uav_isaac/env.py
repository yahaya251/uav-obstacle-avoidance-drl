"""DirectRLEnv adapter for the shared batched planar navigation contract."""

import math
from collections.abc import Sequence

import torch
from isaaclab.envs import DirectRLEnv
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from .env_cfg import ALTITUDE_M, CONTROL_PERIOD_S, HALF_WORLD_M, UAVEnvCfg
from .geometry import CONFIG, RAY_COUNT, PlanarGeometry, limit_velocity_batch


OBSTACLE_HEIGHT_M = 2.5
WALL_THICKNESS_M = 0.12


def _place(geometry, position, color, *, collision=False):
    geometry.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    geometry.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(geometry.GetPrim()).CreateCollisionEnabledAttr(True)


def _box(stage, path, size, position, color, *, collision=False, yaw_degrees=0.0):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    _place(box, position, color, collision=collision)
    if yaw_degrees:
        box.AddRotateZOp().Set(yaw_degrees)
    box.AddScaleOp().Set(Gf.Vec3f(*size))


def _cylinder(stage, path, radius, height, position, color, *, collision=False):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(float(radius))
    cylinder.CreateHeightAttr(float(height))
    _place(cylinder, position, color, collision=collision)


def _author_arena(stage, root, layout):
    """Show exactly the cylindrical layout and interior bounds used by the policy task."""
    width = 2.0 * HALF_WORLD_M
    height = OBSTACLE_HEIGHT_M
    edge = HALF_WORLD_M + WALL_THICKNESS_M / 2
    wall_color = (0.26, 0.31, 0.39)
    _box(
        stage, f"{root}/Floor", (width, width, 0.10),
        (0, 0, -0.05), (0.11, 0.14, 0.19), collision=True,
    )
    for label, sign in (("Negative", -1), ("Positive", 1)):
        _box(
            stage, f"{root}/EastWest{label}", (WALL_THICKNESS_M, width, height),
            (sign * edge, 0, height / 2), wall_color, collision=True,
        )
        _box(
            stage, f"{root}/NorthSouth{label}", (width, WALL_THICKNESS_M, height),
            (0, sign * edge, height / 2), wall_color, collision=True,
        )
    for index, (east, north, radius) in enumerate(layout["obstacles"]):
        _cylinder(
            stage, f"{root}/Obstacle_{index}", radius, height,
            (east, north, height / 2), (0.88, 0.40, 0.17), collision=True,
        )
    goal_east, goal_north = layout["goal"]
    _cylinder(
        stage, f"{root}/Goal", CONFIG.goal_radius, 0.02,
        (goal_east, goal_north, 0.015), (0.22, 0.80, 0.49),
    )

    # The lightweight visual model fits within the navigation disc. The arms
    # and rotors have no actuator or flight-control configuration.
    for index, yaw in enumerate((45.0, -45.0)):
        _box(
            stage, f"{root}/UAV/Arm_{index}", (0.25, 0.022, 0.023),
            (0, 0, 0), (0.19, 0.44, 0.76), yaw_degrees=yaw,
        )
    rotor_centers = ((0.114, 0.114), (0.114, -0.114), (-0.114, 0.114), (-0.114, -0.114))
    for index, (east, north) in enumerate(rotor_centers):
        _cylinder(
            stage, f"{root}/UAV/Rotor_{index}", 0.037, 0.02,
            (east, north, 0.027), (0.13, 0.38, 0.68),
        )


class UAVEnv(DirectRLEnv):
    """Fixed-altitude ENU navigation with the existing 40D policy contract.

    Isaac Lab owns the scene, time stepping, vector API, and autoreset. A
    deterministic swept-disc model owns horizontal motion and obstacle
    contact. The kinematic USD body follows that state for visual inspection.
    """

    cfg: UAVEnvCfg

    def __init__(self, cfg: UAVEnvCfg, render_mode: str | None = None, **kwargs):
        if len(cfg.layouts) != cfg.scene.num_envs:
            raise ValueError("Provide one frozen layout per Isaac Lab scene instance.")
        if not math.isclose(cfg.sim.dt * cfg.decimation, CONTROL_PERIOD_S):
            raise ValueError("The planar benchmark runs at 0.1 seconds per control decision.")
        super().__init__(cfg, render_mode=render_mode, **kwargs)
        if self.sim.has_gui:
            east, north, up = self._position_origin[0].cpu().tolist()
            self.sim.set_camera_view(
                eye=(east + 8.0, north - 10.0, up + 17.0),
                target=(east, north, up),
            )

    def _setup_scene(self):
        self.geometry = PlanarGeometry(
            self.cfg.layouts, device=self.device, half_world=HALF_WORLD_M,
        )
        self.vehicle = self.scene.rigid_objects["vehicle"]
        for env_path, layout in zip(self.scene.env_prim_paths, self.cfg.layouts, strict=True):
            _author_arena(self.scene.stage, env_path, layout)
        light = UsdLux.DomeLight.Define(self.scene.stage, "/World/DomeLight")
        light.CreateIntensityAttr(800.0)

        self.position = self.geometry.start.clone()
        self.goal = self.geometry.goal
        self.velocity = torch.zeros_like(self.position)
        if self.cfg.track_episode_outcomes:
            self._path_length = self.position.new_zeros(self.num_envs)
            self._initial_clearance = self.geometry.clearance(self.position)
            self._min_clearance = self._initial_clearance.clone()
            self._clearance_stops = torch.zeros(self.num_envs, dtype=torch.int64, device=self.device)
            self._episode_return = self.position.new_zeros(self.num_envs)
        self._position_origin = self.scene.env_origins.to(dtype=self.position.dtype).clone()
        self._world_pose = self.position.new_zeros((self.num_envs, 7))
        self._world_pose[:, 2] = self._position_origin[:, 2] + ALTITUDE_M
        self._world_pose[:, 6] = 1.0  # Isaac Lab 3.0 xyzw identity orientation.
        self._refresh_sensing()
        self._initial_policy_obs = self._policy_obs
        self._initial_min_range = self._min_range
        self._initial_center_range = self._center_range

    def _refresh_sensing(self):
        raw_ranges = self.geometry.ranges(self.position)
        self._policy_obs, self._min_range = self.geometry.observe(
            self.position, self.velocity, ranges=raw_ranges,
        )
        self._center_range = raw_ranges[:, :: RAY_COUNT // CONFIG.rays].amin(dim=1)
        self._observations_current = True

    def _pre_physics_step(self, actions: torch.Tensor):
        if actions.shape != self.velocity.shape:
            raise ValueError("Expected one two-component action per environment.")
        actions = actions.to(dtype=self.position.dtype)
        previous_position, previous_velocity = self.position, self.velocity
        self.velocity, near_stop = limit_velocity_batch(
            actions, previous_velocity, self._min_range, self.step_dt,
        )
        self.position = previous_position + self.velocity * self.step_dt
        self._collision = self.geometry.collision(previous_position, self.position)
        self._goal_distance = torch.linalg.vector_norm(self.goal - self.position, dim=-1)
        previous_distance = torch.linalg.vector_norm(self.goal - previous_position, dim=-1)
        self._progress = previous_distance - self._goal_distance
        self._velocity_change_sq = (self.velocity - previous_velocity).square().sum(dim=-1)
        if self.cfg.track_episode_outcomes:
            self._path_length += torch.linalg.vector_norm(self.position - previous_position, dim=-1)
            self._min_clearance = torch.minimum(self._min_clearance, self.geometry.clearance(self.position))
            self._clearance_stops += near_stop
        self._observations_current = False
        self.extras.pop("final_obs", None)

    def _write_vehicle_pose(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            self._world_pose[:, :2] = self._position_origin[:, :2] + self.position
            self.vehicle.write_root_pose_to_sim_index(root_pose=self._world_pose)
            return

        self._world_pose[env_ids, :2] = self._position_origin[env_ids, :2] + self.position[env_ids]
        self.vehicle.write_root_pose_to_sim_index(
            root_pose=self._world_pose[env_ids], env_ids=env_ids,
        )

    def _apply_action(self):
        self._write_vehicle_pose()

    def _get_observations(self):
        if not self._observations_current:
            self._refresh_sensing()
        return {"policy": self._policy_obs}

    def _get_dones(self):
        self._success = ~self._collision & (self._goal_distance <= CONFIG.goal_radius)
        terminated = self._collision | self._success
        self._timeout = ~terminated & (self.episode_length_buf >= self.max_episode_length)
        return terminated, self._timeout

    def _get_rewards(self):
        if not self._observations_current:
            self._refresh_sensing()
        clearance = self._center_range - CONFIG.vehicle_radius
        reward = 2.0 * self._progress - 0.01 - 0.02 * self._velocity_change_sq
        reward -= 0.1 * (0.6 - clearance).clamp_min(0.0)
        reward += 20.0 * self._success - 20.0 * self._collision
        if not self.cfg.track_episode_outcomes:
            return reward
        self._episode_return += reward

        # The base environment returns reset observations when an episode ends.
        # Preserve terminal metrics before it changes any row in this batch.
        self.extras["episode_outcome"] = {
            "success": self._success,
            "collision": self._collision,
            "timeout": self._timeout,
            "steps": self.episode_length_buf.clone(),
            "path_length_m": self._path_length.clone(),
            "goal_distance_m": self._goal_distance,
            "min_clearance_m": self._min_clearance.clone(),
            "guard_stop_steps": self._clearance_stops.clone(),
            "episode_return": self._episode_return.clone(),
        }
        return reward

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        indices = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self.position[indices] = self.geometry.start[indices]
        self.velocity[indices] = 0.0
        if self.cfg.track_episode_outcomes:
            self._path_length[indices] = 0.0
            self._min_clearance[indices] = self._initial_clearance[indices]
            self._clearance_stops[indices] = 0
            self._episode_return[indices] = 0.0
        # Layouts are fixed. Replace only the reset rows with the precomputed
        # start scans and keep the old observation tensor for PPO bootstrapping.
        self._policy_obs = self._policy_obs.index_copy(0, indices, self._initial_policy_obs[indices])
        self._min_range = self._min_range.index_copy(0, indices, self._initial_min_range[indices])
        self._center_range = self._center_range.index_copy(0, indices, self._initial_center_range[indices])
        self._write_vehicle_pose(indices)
