"""CPU benchmarking of APF, goal seeking and PPO under a shared 40D contract."""

import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import shutil
import sys
import time

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from ..core import Config, ObservationResult, apf_velocity, limit_velocity
from ..kinematics import Config as ToyConfig, KinematicEnv

CFG = Config()
SCOPE = "CPU_2D_kinematic_diagnostic_not_Gazebo_PX4_flight"
ANGLES = np.asarray([-math.pi + i * (math.tau / 720) for i in range(720)])
DIRECTIONS = np.stack([np.cos(ANGLES), np.sin(ANGLES)], axis=1)
SECTORS = np.asarray([
    int(math.floor(((float(angle) + math.pi / 36) % math.tau) * 36 / math.tau)) % 36
    for angle in ANGLES
])
FAMILIES = ["open", "single_obstacle", "staggered", "corridor", "u_trap"]


def make_layout(seed, family):
    r = random.Random(seed)
    start = (-4.4, r.uniform(-0.5, 0.5))
    goal = (4.4, r.uniform(-0.5, 0.5))
    if family == "open":
        obstacles = []
    elif family == "single_obstacle":
        obstacles = [(r.uniform(-0.5, 0.5), r.uniform(-0.7, 0.7), r.uniform(0.45, 0.7))]
    elif family == "staggered":
        obstacles = [
            (-1.0, r.uniform(0.65, 1.25), r.uniform(0.4, 0.55)),
            (1.0, -r.uniform(0.65, 1.25), r.uniform(0.4, 0.55)),
            (2.5, r.uniform(0.8, 1.3), 0.35),
        ]
    elif family == "corridor":
        width = r.uniform(1.65, 1.95)
        obstacles = [
            (x, y, r.uniform(0.27, 0.34))
            for x in (-2, -1, 0, 1, 2) for y in (-width, width)
        ]
    elif family == "u_trap":
        shift = r.uniform(-0.12, 0.12)
        start = (-1.8 + shift, r.uniform(-0.12, 0.12))
        goal = (4.1, r.uniform(-0.2, 0.2))
        obstacles = (
            [(shift, y, 0.3) for y in (-1.2, -0.6, 0, 0.6, 1.2)]
            + [(x + shift, y, 0.3) for x in (-0.6, -1.2, -1.8, -2.4) for y in (-1.2, 1.2)]
        )
    else:
        raise ValueError(family)
    return {
        "id": f"{family}-{seed}", "layout_seed": seed, "family": family,
        "start": start, "goal": goal, "obstacles": obstacles,
    }


def freeze_splits(output, profile):
    def layouts(first_seed, count):
        return [
            make_layout(first_seed + i, FAMILIES[i % len(FAMILIES)])
            for i in range(count)
        ]

    payload = {
        "schema": "uav_nav.split.v1",
        "scope": SCOPE,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "train": layouts(0, 200),
        "validation": layouts(10000, 20),
        "test": layouts(profile.test_seed_start, profile.test_layouts),
    }
    assert_splits_disjoint(payload)
    (output / "frozen_splits.json").write_text(json.dumps(payload, indent=2) + "\n")
    return payload


def assert_splits_disjoint(payload):
    groups = [payload[name] for name in ("train", "validation", "test")]
    seed_sets = [{row["layout_seed"] for row in group} for group in groups]
    geometry_sets = [{
        hashlib.sha256(json.dumps(
            {key: row[key] for key in ("start", "goal", "obstacles")}, sort_keys=True
        ).encode()).hexdigest()
        for row in group
    } for group in groups]
    for sets in (seed_sets, geometry_sets):
        if any(sets[i] & sets[j] for i in range(3) for j in range(i)):
            raise ValueError("Training, validation and test splits must be disjoint.")


def scan_ranges(core):
    pos = np.asarray(core.position, dtype=float)
    result = np.full(720, CFG.sensor_range)
    for axis in (0, 1):
        d = DIRECTIONS[:, axis]
        for wall in (-core.cfg.half_world, core.cfg.half_world):
            t = np.divide(wall - pos[axis], d, out=np.full(720, np.inf), where=np.abs(d) > 1e-12)
            result = np.minimum(result, np.where(t >= 0, t, np.inf))
    if core.obstacles:
        circles = np.asarray(core.obstacles)
        offset = pos[None, :] - circles[:, :2]
        b = offset @ DIRECTIONS.T
        disc = b * b - (np.sum(offset * offset, axis=1) - circles[:, 2] ** 2)[:, None]
        near = -b - np.sqrt(np.maximum(0, disc))
        hit = np.where((disc >= 0) & (near >= 0), near, np.inf)
        inside = np.sum(offset * offset, axis=1) <= circles[:, 2] ** 2
        hit[inside, :] = 0.0
        result = np.minimum(result, hit.min(axis=0))
    return result


def observation(core):
    raw = scan_ranges(core)
    bins = np.full(36, CFG.sensor_range)
    np.minimum.at(bins, SECTORS, raw)
    relative = np.clip((np.asarray(core.goal) - core.position) / CFG.goal_scale, -1, 1)
    values = (
        tuple(bins / CFG.sensor_range) + tuple(relative)
        + tuple(np.asarray(core.velocity) / CFG.max_speed)
    )
    return ObservationResult(values, True, (), (True,) * 36, float(bins.min()), 0.0, 0.0, 0, 720)


def exact_clearance(core):
    boundary = core.cfg.half_world - max(abs(v) for v in core.position) - core.cfg.vehicle_radius
    return min([boundary] + [
        math.dist(core.position, (x, y)) - radius - core.cfg.vehicle_radius
        for x, y, radius in core.obstacles
    ])


class DiagnosticEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, layouts):
        super().__init__()
        self.layouts = layouts
        self.core = KinematicEnv(ToyConfig(max_steps=240))
        self.observation_space = gym.spaces.Box(
            low=np.array([0.0] * 36 + [-1.0] * 4, dtype=np.float32),
            high=np.ones(40, dtype=np.float32),
        )
        self.action_space = gym.spaces.Box(-1, 1, shape=(2,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.layout = (
            options["layout"] if options and "layout" in options
            else self.layouts[int(self.np_random.integers(len(self.layouts)))]
        )
        _, info = self.core.reset(seed=self.layout["layout_seed"], options=self.layout)
        self.previous = (0.0, 0.0)
        self.obs = observation(self.core)
        self.min_clearance = exact_clearance(self.core)
        self.guard_stops = 0
        return np.asarray(self.obs.values, dtype=np.float32), info

    def step(self, action):
        limited = limit_velocity(
            action, self.previous, self.core.cfg.dt,
            cfg=CFG, ready=self.obs.ready, min_range_m=self.obs.min_range_m,
        )
        _, reward, terminated, truncated, info = self.core.step(limited.velocity_enu)
        self.previous = limited.velocity_enu
        self.guard_stops += int(limited.reason == "minimum_range_stop")
        self.obs = observation(self.core)
        self.min_clearance = min(self.min_clearance, exact_clearance(self.core))
        info.update(
            min_clearance_m=self.min_clearance,
            guard_stops=self.guard_stops,
            action_reason=limited.reason,
        )
        return np.asarray(self.obs.values, dtype=np.float32), reward, terminated, truncated, info


def goal_action(obs):
    g = np.asarray(obs.values[36:38])
    n = np.linalg.norm(g)
    return tuple(g / max(1, n)) if n < 1e-12 else tuple(g / n)


def evaluate(policy, layouts, algorithm, training_seed=None, trajectory_writer=None):
    env = DiagnosticEnv(layouts)
    rows = []
    for layout in layouts:
        obs, _ = env.reset(options={"layout": layout}, seed=layout["layout_seed"])
        total = 0.0
        latencies = []
        step_total = []
        for step in range(env.core.cfg.max_steps):
            begin = time.perf_counter_ns()
            if algorithm == "APF":
                action = apf_velocity(env.obs)
            elif algorithm == "GoalSeeking":
                action = goal_action(env.obs)
            else:
                action, _ = policy.predict(obs, deterministic=True)
            latencies.append((time.perf_counter_ns() - begin) / 1e6)
            obs, reward, terminated, truncated, info = env.step(action)
            step_total.append((time.perf_counter_ns() - begin) / 1e6)
            total += reward
            if trajectory_writer:
                trajectory_writer.writerow([
                    algorithm, training_seed, layout["id"], step + 1,
                    *env.core.position, *env.previous, info["action_reason"],
                ])
            if terminated or truncated:
                break
        row = {
            "algorithm": algorithm,
            "training_seed": training_seed,
            "layout_id": layout["id"],
            "family": layout["family"],
            "success": bool(info["success"]),
            "collision": bool(info["collision"]),
            "timeout": bool(info["timeout"]),
            "steps": info["steps"],
            "simulated_duration_s": info["steps"] * 0.1,
            "return": float(total),
            "path_length_m": info["path_length_m"],
            "goal_distance_m": info["goal_distance_m"],
            "min_clearance_m": env.min_clearance,
            "guard_stop_steps": env.guard_stops,
            "success_lower_bound_efficiency": (
                (env.core.initial_distance - CFG.goal_radius) / info["path_length_m"]
                if info["success"] else None
            ),
            "decision_latency_median_ms": float(np.median(latencies)),
            "decision_latency_p95_ms": float(np.percentile(latencies, 95)),
            "local_step_median_ms": float(np.median(step_total)),
            "infrastructure_failure": False,
        }
        assert sum(row[x] for x in ("success", "collision", "timeout")) == 1
        rows.append(row)
    return rows

class ValidationCallback(BaseCallback):
    def __init__(self, layouts, directory, checkpoints):
        super().__init__()
        self.layouts = layouts
        self.directory = directory
        self.checkpoints = checkpoints
        self.best = -math.inf
        self.scores = []

    def _on_step(self):
        if self.num_timesteps in self.checkpoints:
            rows = evaluate(self.model, self.layouts, "PPO")
            score = float(np.mean([row["return"] for row in rows]))
            record = {
                "timesteps": self.num_timesteps,
                "validation_mean_return": score,
                "validation_successes": sum(row["success"] for row in rows),
                "episodes": len(rows),
            }
            self.scores.append(record)
            if score > self.best:
                self.best = score
                self.model.save(self.directory / "best_validation")
            (self.directory / "validation_scores.json").write_text(
                json.dumps(self.scores, indent=2) + "\n"
            )
            print(json.dumps({"event": "validation", **record}), flush=True)
        return True


def grouped(rows):
    output = []
    keys = sorted({(r["algorithm"], str(r["training_seed"])) for r in rows})
    for alg, seed in keys:
        rr = [r for r in rows if r["algorithm"] == alg and str(r["training_seed"]) == seed]
        success = [r for r in rr if r["success"]]
        output.append({
            "algorithm": alg,
            "training_seed": None if seed == "None" else int(seed),
            "episodes": len(rr),
            **{k: sum(r[k] for r in rr) for k in ("success", "collision", "timeout")},
            "success_rate": sum(r["success"] for r in rr) / len(rr),
            "mean_success_efficiency": (
                float(np.mean([r["success_lower_bound_efficiency"] for r in success]))
                if success else None
            ),
            "mean_path_all_episodes_m": float(np.mean([r["path_length_m"] for r in rr])),
            "median_episode_decision_p95_ms": float(np.median([r["decision_latency_p95_ms"] for r in rr])),
            "families": [
                {
                    f: sum(r["success"] for r in rr if r["family"] == f),
                    "episodes": sum(r["family"] == f for r in rr),
                }
                for f in FAMILIES
            ],
        })
    return output

def run_benchmark(output, profile_name, profile):
    """Execute the specified protocol in a newly allocated directory."""
    output = Path(output).resolve()
    output.mkdir(parents=True)
    torch.set_num_threads(1)

    splits = freeze_splits(output, profile)
    from .contracts import run_tests
    test_report = run_tests(splits, output)

    parameters = {
        "n_steps": 1024,
        "batch_size": 128,
        "n_epochs": 5,
        "learning_rate": 0.0003,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip_range": 0.2,
    }
    package = Path(__file__).resolve().parents[1]
    source_paths = [
        "core.py", "kinematics.py", "benchmark/experiment.py",
        "benchmark/contracts.py", "benchmark/profiles.py",
    ]
    source_hashes = {
        f"src/uav_nav/{name}": hashlib.sha256((package / name).read_bytes()).hexdigest()
        for name in source_paths
    }
    metadata = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "scope": SCOPE,
        "profile": profile_name,
        "python": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "torch": torch.__version__,
        "numpy": np.__version__,
        "observation_dimensions": 40,
        "sensor_beams": len(ANGLES),
        "sensor_range_m": CFG.sensor_range,
        "sector_count": CFG.rays,
        "dt_s": 0.1,
        "max_speed_m_s": CFG.max_speed,
        "max_acceleration_m_s2": CFG.max_acceleration,
        "stop_center_range_m": CFG.vehicle_radius + CFG.stop_margin,
        "max_steps": 240,
        "training_seeds": profile.training_seeds,
        "timesteps_per_seed": profile.timesteps,
        "validation_checkpoints": profile.checkpoints,
        "test_seed_start": profile.test_seed_start,
        "split_sizes": {name: len(rows) for name, rows in splits.items() if isinstance(rows, list)},
        "checkpoint_selection": "highest_mean_validation_return_only",
        "split_sha256": hashlib.sha256((output / "frozen_splits.json").read_bytes()).hexdigest(),
        "ppo_hyperparameters": {**parameters, "net_arch": [64, 64], "device": "cpu", "torch_threads": 1},
        "source_sha256": source_hashes,
    }
    (output / "run_manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    for name in source_paths:
        target = output / "executed_sources/src/uav_nav" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(package / name, target)

    results = []
    with gzip.open(output / "test_trajectories.csv.gz", "wt", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "algorithm", "training_seed", "layout_id", "step",
            "east_m", "north_m", "executed_east_m_s", "executed_north_m_s",
            "action_reason",
        ])
        for algorithm in ("APF", "GoalSeeking"):
            results.extend(evaluate(None, splits["test"], algorithm, trajectory_writer=writer))

        for seed in profile.training_seeds:
            directory = output / f"ppo_seed_{seed}"
            directory.mkdir()
            env = Monitor(DiagnosticEnv(splits["train"]), str(directory / "training_monitor.csv"))
            try:
                model = PPO(
                    "MlpPolicy", env, seed=seed, device="cpu",
                    policy_kwargs={"net_arch": [64, 64]}, verbose=0, **parameters,
                )
                model.set_logger(configure(str(directory), ["csv"]))
                callback = ValidationCallback(splits["validation"], directory, profile.checkpoints)
                started = time.perf_counter()
                model.learn(total_timesteps=profile.timesteps, callback=callback)
                seconds = time.perf_counter() - started
                model.save(directory / "final_checkpoint")
            finally:
                env.close()
            selected = PPO.load(directory / "best_validation", device="cpu")
            results.extend(evaluate(selected, splits["test"], "PPO", seed, writer))
            timing = {
                "training_and_validation_wall_seconds": seconds,
                "training_environment_steps": profile.timesteps,
                "validation_episodes": len(splits["validation"]) * len(profile.checkpoints),
            }
            (directory / "training_time.json").write_text(json.dumps(timing, indent=2) + "\n")
            print(json.dumps({"event": "seed_complete", "seed": seed, "seconds": seconds}), flush=True)

    (output / "test_episode_results.json").write_text(json.dumps(results, indent=2) + "\n")
    with (output / "test_episode_results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)

    report = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "scope": SCOPE,
        "profile": profile_name,
        "episodes": len(results),
        "summaries": grouped(results),
        "tests": test_report,
        "latency_scope": "Local Python decision and kinematic step timings. Each reported p95 is computed within one episode before taking the controller median.",
        "evaluation_scope": "Fixed-layout 2D prototype results. Gazebo/PX4 closed-loop flight and hardware validation remain separate.",
    }
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"event": "complete", "episodes": len(results), "output": str(output)}), flush=True)
