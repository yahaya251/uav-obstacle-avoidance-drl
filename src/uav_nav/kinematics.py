"""Planar kinematic environment with deterministic seeded layouts.

Fixed-altitude horizontal ENU frame. Rays start at +East and advance CCW.
Observations: 36 ranges / 6m + target dx,dy / 12m + vx,vy / 1m/s.
Action: [East velocity, North velocity], normalized then limited to 1m/s norm.
No map/obstacle list is delivered to the controller. No noise or delay yet.
"""
from dataclasses import dataclass
import math
import random

@dataclass(frozen=True)
class Config:
    dt: float = 0.1
    speed: float = 1.0
    sensor_range: float = 6.0
    goal_scale: float = 12.0
    rays: int = 36
    vehicle_radius: float = 0.2
    goal_radius: float = 0.35
    half_world: float = 6.0
    max_steps: int = 300

def segment_distance(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.0
    return math.hypot(a[0]+t*dx-p[0], a[1]+t*dy-p[1])

class KinematicEnv:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.done = True

    def reset(self, *, seed=0, options=None):
        self.seed = seed
        rng = random.Random(seed)
        opts = options or {}
        self.position = tuple(opts.get("start", (-4.5, -0.7)))
        self.goal = tuple(opts.get("goal", (4.5, 0.7)))
        self.obstacles = list(opts.get("obstacles", [(0.0, rng.uniform(-0.5, 0.5), 0.65), (1.8, 2.2, 0.5), (-2.0, -2.3, 0.45)]))
        self.velocity = (0.0, 0.0)
        self.steps, self.path_length, self.done = 0, 0.0, False
        self.initial_distance = math.dist(self.position, self.goal)
        return self.observe(), {"seed": seed, "engineering_smoke": True}

    def ray_distance(self, angle):
        x, y = self.position
        ux, uy = math.cos(angle), math.sin(angle)
        c = self.cfg
        candidates = [c.sensor_range]
        for coord, direction in ((x, ux), (y, uy)):
            if abs(direction) > 1e-12:
                for wall in (-c.half_world, c.half_world):
                    t = (wall-coord)/direction
                    if t >= 0:
                        candidates.append(t)
        for cx, cy, radius in self.obstacles:
            ox, oy = x-cx, y-cy
            b = ox*ux+oy*uy
            discr = b*b-(ox*ox+oy*oy-radius*radius)
            if discr >= 0:
                t = -b-math.sqrt(discr)
                if t >= 0:
                    candidates.append(t)
                elif ox*ox+oy*oy <= radius*radius:
                    candidates.append(0.0)
        return min(candidates)

    def observe(self):
        c = self.cfg
        rays = [self.ray_distance(i*2*math.pi/c.rays)/c.sensor_range for i in range(c.rays)]
        rel = [max(-1.0, min(1.0, (self.goal[i]-self.position[i])/c.goal_scale)) for i in range(2)]
        return rays + rel + [v/c.speed for v in self.velocity]

    def step(self, action):
        if self.done:
            raise RuntimeError("Call reset after an episode terminates or truncates.")
        if len(action) != 2 or not all(math.isfinite(float(a)) for a in action):
            raise ValueError("Action must contain two finite numbers.")
        c = self.cfg
        u = [max(-1.0, min(1.0, float(a))) for a in action]
        norm = max(1.0, math.hypot(*u))
        v = tuple(c.speed*a/norm for a in u)
        old, old_v = self.position, self.velocity
        old_d = math.dist(old, self.goal)
        self.position = (old[0]+v[0]*c.dt, old[1]+v[1]*c.dt)
        self.velocity = v
        self.path_length += math.dist(old, self.position)
        self.steps += 1
        collision = any(segment_distance(old, self.position, (x,y)) <= r+c.vehicle_radius for x,y,r in self.obstacles)
        collision = collision or any(abs(q) >= c.half_world-c.vehicle_radius for q in self.position)
        distance = math.dist(self.position, self.goal)
        success = not collision and distance <= c.goal_radius
        terminated = bool(collision or success)
        truncated = not terminated and self.steps >= c.max_steps
        self.done = terminated or truncated
        clearance = min(self.ray_distance(i*2*math.pi/c.rays) for i in range(c.rays)) - c.vehicle_radius
        # Proposed initial reward, to be tuned with train/validation worlds only.
        reward = 2.0*(old_d-distance)-0.01-0.02*sum((v[i]-old_v[i])**2 for i in range(2))
        reward -= 0.1*max(0.0, 0.6-clearance)
        reward += 20.0 if success else -20.0 if collision else 0.0
        info = {"engineering_smoke": True, "success": success, "collision": collision,
                "timeout": truncated, "steps": self.steps, "path_length_m": self.path_length,
                "goal_distance_m": distance, "clearance_m": clearance}
        return self.observe(), reward, terminated, truncated, info
