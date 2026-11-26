"""Batched horizontal sensing and swept-disc collision for fixed ENU layouts."""

import math

import torch

from uav_nav.core import Config


CONFIG = Config()
RAY_COUNT = 720


class PlanarGeometry:
    """Hold world geometry on the same device as the environment state."""

    def __init__(self, layouts, *, device="cpu", dtype=torch.float32, half_world=6.0):
        if not layouts:
            raise ValueError("At least one layout is required.")
        self.half_world = half_world
        self.start = torch.tensor([layout["start"] for layout in layouts], device=device, dtype=dtype)
        self.goal = torch.tensor([layout["goal"] for layout in layouts], device=device, dtype=dtype)
        if self.start.shape != (len(layouts), 2) or self.goal.shape != self.start.shape:
            raise ValueError("Starts and goals must be ENU pairs.")
        counts = [len(layout["obstacles"]) for layout in layouts]
        width = max(1, *counts)
        padded = [list(layout["obstacles"]) + [(0.0, 0.0, 0.0)] * (width - count)
                  for layout, count in zip(layouts, counts)]
        self.circles = torch.tensor(padded, device=device, dtype=dtype)
        if self.circles.shape != (len(layouts), width, 3):
            raise ValueError("Obstacles must be (east, north, radius) triples.")
        self.obstacle_mask = torch.arange(width, device=device)[None, :] < torch.tensor(
            counts, device=device
        )[:, None]
        if not self._valid_layouts(half_world):
            raise ValueError("Layouts require finite coordinates, in-bounds endpoints and positive radii.")

        angles = [-math.pi + i * (math.tau / RAY_COUNT) for i in range(RAY_COUNT)]
        self.directions = torch.tensor(
            [(math.cos(angle), math.sin(angle)) for angle in angles], device=device, dtype=dtype
        )
        sector_angles = [i * math.tau / CONFIG.rays for i in range(CONFIG.rays)]
        self.sector_directions = torch.tensor(
            [(math.cos(angle), math.sin(angle)) for angle in sector_angles], device=device, dtype=dtype
        )
        self.sectors = torch.tensor(
            [int(math.floor(((angle + math.pi / CONFIG.rays) % math.tau)
                            * CONFIG.rays / math.tau)) % CONFIG.rays for angle in angles],
            device=device,
        ).expand(len(layouts), -1)
        self.inverse_directions = torch.where(
            self.directions.abs() > 1e-12, self.directions.reciprocal(), torch.inf
        )
        self.parallel_to_walls = self.inverse_directions.isinf()
        self.wall_coordinates = torch.where(self.directions > 0, half_world, -half_world)
        self.radius_sq = self.circles[..., 2].square()
        self.collision_radius_sq = (self.circles[..., 2] + CONFIG.vehicle_radius).square()

    def _valid_layouts(self, half_world):
        return (
            math.isfinite(half_world) and half_world > CONFIG.vehicle_radius
            and bool(torch.isfinite(self.start).all() & torch.isfinite(self.goal).all()
                     & torch.isfinite(self.circles).all())
            and bool((self.start.abs() < half_world).all() & (self.goal.abs() < half_world).all())
            and bool(((self.circles[..., 2] > 0) | ~self.obstacle_mask).all())
        )

    def ranges(self, position):
        """Return capped ray distances at angles -pi + i * 2pi / 720."""
        wall_distances = (self.wall_coordinates - position[:, None, :]) * self.inverse_directions
        wall_distances = wall_distances.masked_fill(
            self.parallel_to_walls | (wall_distances < 0), torch.inf
        )
        wall_ranges = wall_distances.amin(dim=-1)
        offset = position[:, None, :] - self.circles[..., :2]
        projection = offset @ self.directions.T
        radial_term = offset.square().sum(dim=-1) - self.radius_sq
        discriminant = projection.square() - radial_term[..., None]
        near = -projection - discriminant.clamp_min(0).sqrt()
        hits = near.masked_fill((discriminant < 0) | (near < 0), torch.inf)
        hits = hits.masked_fill(radial_term[..., None] <= 0, 0.0)
        hits = hits.masked_fill(~self.obstacle_mask[..., None], torch.inf)
        return torch.minimum(wall_ranges, hits.amin(dim=1)).clamp_max(CONFIG.sensor_range)

    def observe(self, position, velocity, *, ranges=None):
        """Return the shared 40-value observation and the nearest raw range."""
        if ranges is None:
            ranges = self.ranges(position)
        sectors = position.new_full((position.shape[0], CONFIG.rays), CONFIG.sensor_range)
        sectors.scatter_reduce_(1, self.sectors, ranges, reduce="amin")
        relative = ((self.goal - position) / CONFIG.goal_scale).clamp(-1.0, 1.0)
        obs = torch.cat((sectors / CONFIG.sensor_range, relative, velocity / CONFIG.max_speed), dim=1)
        return obs, sectors.amin(dim=1)

    def collision(self, old, new):
        """Test the swept vehicle radius against cylinders and room boundaries."""
        segment = (new - old)[:, None, :]
        to_circle = self.circles[..., :2] - old[:, None, :]
        denominator = segment.square().sum(dim=-1)
        fraction = ((to_circle * segment).sum(dim=-1) / denominator.clamp_min(
            torch.finfo(new.dtype).tiny
        )).clamp(0.0, 1.0)
        nearest = to_circle - fraction[..., None] * segment
        hit = (nearest.square().sum(dim=-1) <= self.collision_radius_sq) & self.obstacle_mask
        wall = new.abs().amax(dim=-1) >= self.half_world - CONFIG.vehicle_radius
        return hit.any(dim=-1) | wall

    def clearance(self, position):
        """Return signed horizontal clearance from the vehicle perimeter."""
        center_distances = torch.linalg.vector_norm(position[:, None, :] - self.circles[..., :2], dim=-1)
        obstacle = (center_distances - self.circles[..., 2]).masked_fill(~self.obstacle_mask, torch.inf)
        wall = self.half_world - position.abs().amax(dim=-1)
        return torch.minimum(obstacle.amin(dim=-1), wall) - CONFIG.vehicle_radius

    def apf_action(self, policy_obs):
        """Evaluate the shared potential-field baseline on the simulator device."""
        goal = policy_obs[:, CONFIG.rays : CONFIG.rays + 2] * CONFIG.goal_scale
        distance = torch.linalg.vector_norm(goal, dim=1, keepdim=True)
        clearance = (policy_obs[:, : CONFIG.rays] * CONFIG.sensor_range - CONFIG.vehicle_radius).clamp_min(0.05)
        repulsion = (0.06 * (clearance.reciprocal() - 1 / 1.5) / clearance.square()).clamp(0.0, 3.0)
        force = goal / distance.clamp_min(CONFIG.goal_radius) - repulsion @ self.sector_directions
        norm = torch.linalg.vector_norm(force, dim=1, keepdim=True).clamp_min(1.0)
        ready = (distance > CONFIG.goal_radius) & torch.isfinite(policy_obs).all(dim=1, keepdim=True)
        return torch.where(ready, CONFIG.max_speed * force / norm, 0.0)


def limit_velocity_batch(actions, previous, minimum_range, dt):
    """Apply the project's speed, acceleration and near-obstacle rules."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("A positive finite update period is required.")
    speed = CONFIG.max_speed
    valid = torch.isfinite(actions).all(dim=-1) & torch.isfinite(previous).all(dim=-1)
    valid_range = (minimum_range >= 0) & ~minimum_range.isnan()
    near_stop = valid_range & (minimum_range <= CONFIG.vehicle_radius + CONFIG.stop_margin)
    actions = torch.where(valid[:, None], actions, 0.0).clamp(-speed, speed)
    previous = torch.where(valid[:, None], previous, 0.0)
    target = actions / (torch.linalg.vector_norm(actions, dim=-1, keepdim=True) / speed).clamp_min(1.0)
    previous = previous / (torch.linalg.vector_norm(previous, dim=-1, keepdim=True) / speed).clamp_min(1.0)
    delta = target - previous
    max_delta = CONFIG.max_acceleration * dt
    scale = max_delta / torch.linalg.vector_norm(delta, dim=-1, keepdim=True).clamp_min(max_delta)
    velocity = torch.where((valid & valid_range & ~near_stop)[:, None], previous + delta * scale, 0.0)
    return velocity, near_stop
