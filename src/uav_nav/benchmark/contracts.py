"""Preflight checks for frozen layouts, sensor equivalence and shared limits."""

import json
import math

import numpy as np

from ..core import PoseSample, ScanSample, build_observation, limit_velocity
from .experiment import DiagnosticEnv, FAMILIES, assert_splits_disjoint, scan_ranges


def run_tests(splits, output):
    checks = {}
    env = DiagnosticEnv(splits["train"])
    obs, _ = env.reset(seed=4)
    checks["finite_40D_bounds"] = len(obs) == 40 and env.observation_space.contains(obs)

    raw = scan_ranges(env.core)
    scalar = np.array([env.core.ray_distance(i * math.tau / 720 - math.pi) for i in range(720)])
    checks["vectorized_720_ray_matches_scalar_geometry"] = bool(np.allclose(raw, scalar, atol=1e-9, rtol=0))
    reference = build_observation(
        ScanSample(tuple(raw), -math.pi, math.tau / 720, 0.0, 6.0, 0.0),
        PoseSample((*env.core.position, 1.5), (*env.core.velocity, 0.0), 0.0, 0.0),
        env.core.goal,
        now=0.0,
    )
    checks["sector_pool_matches_shared_observation"] = bool(
        np.allclose(reference.values, env.obs.values, atol=1e-9, rtol=0)
    )

    first = obs.copy()
    env.step([1, 1])
    again, _ = env.reset(seed=4)
    checks["deterministic_seed_reset"] = bool(np.array_equal(first, again))

    env.reset(options={"layout": splits["test"][0]})
    _, _, _, _, info = env.step([1, 1])
    checks["reused_norm_and_acceleration_guard"] = (
        math.hypot(*env.previous) <= 0.1000000001 and info["action_reason"] == "limited"
    )
    stop = limit_velocity((1, 0), (0.5, 0), 0.1, ready=True, min_range_m=0.5)
    checks["same_near_obstacle_zero_guard"] = stop.velocity_enu == (0.0, 0.0)

    assert_splits_disjoint(splits)
    checks["disjoint_seed_and_geometry_partitions"] = True
    checks["balanced_held_out_families"] = all(
        sum(row["family"] == family for row in splits["test"]) * len(FAMILIES) == len(splits["test"])
        for family in FAMILIES
    )
    checks["no_controller_map_input"] = len(env.obs.values) == 40 and not hasattr(env.obs, "obstacles")

    env.reset(options={"layout": splits["test"][0]})
    for _ in range(env.core.cfg.max_steps):
        _, _, terminated, truncated, info = env.step([0, 0])
        if terminated or truncated:
            break
    checks["zero_action_timeout_classification"] = bool(
        info["timeout"] and not info["success"] and not info["collision"]
    )
    env.close()

    report = {
        "passed": all(checks.values()),
        "count": len(checks),
        "checks": checks,
        "scope": "kinematic_benchmark_contracts",
    }
    (output / "test_report.json").write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise RuntimeError(f"Benchmark contracts failed: {checks}")
    return report
