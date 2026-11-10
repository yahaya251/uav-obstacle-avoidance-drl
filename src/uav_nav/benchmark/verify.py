"""Independently check recorded outcomes, trajectories and provenance."""

from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path

from stable_baselines3 import PPO

OUTCOMES = ("success", "collision", "timeout")


def episode_key(row):
    return row["algorithm"], row["training_seed"], row["layout_id"]


def check_run(directory):
    directory = Path(directory)
    manifest = json.loads((directory / "run_manifest.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    split_path = directory / "frozen_splits.json"
    split = json.loads(split_path.read_text())
    rows = json.loads((directory / "test_episode_results.json").read_text())
    layouts = {row["id"]: row for row in split["test"]}
    controllers = [
        ("APF", None), ("GoalSeeking", None),
        *(("PPO", seed) for seed in manifest["training_seeds"]),
    ]
    expected_keys = {
        (algorithm, seed, layout_id)
        for algorithm, seed in controllers
        for layout_id in layouts
    }
    episode_keys = {episode_key(row) for row in rows}
    snapshot = (directory / "executed_sources").resolve()
    source_hashes = manifest["source_sha256"]

    checks = {
        "split_hash": hashlib.sha256(split_path.read_bytes()).hexdigest() == manifest["split_sha256"],
        "executed_source_snapshot_hashes": bool(source_hashes) and all(
            (path := (snapshot / name).resolve()).is_relative_to(snapshot)
            and hashlib.sha256(path.read_bytes()).hexdigest() == expected
            for name, expected in source_hashes.items()
        ),
        "all_test_episodes_retained": len(rows) == len(expected_keys) == len(episode_keys),
        "complete_mutually_exclusive_outcomes": all(
            all(isinstance(row[key], bool) for key in OUTCOMES)
            and sum(row[key] for key in OUTCOMES) == 1 for row in rows
        ),
        "no_excluded_infrastructure_failures": all(row["infrastructure_failure"] is False for row in rows),
        "identical_test_layouts_per_controller": episode_keys == expected_keys,
        "finite_numeric_results": all(
            math.isfinite(value) for row in rows for value in row.values()
            if isinstance(value, (int, float))
        ),
        "success_efficiency_in_0_1": all(
            0 < row["success_lower_bound_efficiency"] <= 1 + 1e-8
            if row["success"] else row["success_lower_bound_efficiency"] is None
            for row in rows
        ),
    }

    counts = defaultdict(Counter)
    for row in rows:
        counts[episode_key(row)[:2]].update(
            ["episodes", *(key for key in OUTCOMES if row[key])]
        )
    summaries = summary["summaries"]
    summary_keys = {(row["algorithm"], row["training_seed"]) for row in summaries}
    checks["summary_counts_match_episode_records"] = (
        len(summary_keys) == len(summaries) == len(controllers)
        and summary_keys == set(controllers)
        and summary["episodes"] == len(rows)
        and all(
            row[key] == counts[(row["algorithm"], row["training_seed"])][key]
            for row in summaries for key in ("episodes", *OUTCOMES)
        )
    )

    selected = []
    correct_budgets = True
    valid_selection = manifest["checkpoint_selection"] == "highest_mean_validation_return_only"
    for seed in manifest["training_seeds"]:
        folder = directory / f"ppo_seed_{seed}"
        scores = json.loads((folder / "validation_scores.json").read_text())
        best = max(scores, key=lambda row: row["validation_mean_return"])
        model = PPO.load(folder / "best_validation", device="cpu")
        valid_selection &= (
            [row["timesteps"] for row in scores] == manifest["validation_checkpoints"]
            and all(row["episodes"] == len(split["validation"]) for row in scores)
            and model.num_timesteps == best["timesteps"]
            and model.observation_space.shape == (40,)
            and model.action_space.shape == (2,)
        )
        with (folder / "progress.csv").open() as stream:
            progress = list(csv.DictReader(stream))
        correct_budgets &= int(progress[-1]["time/total_timesteps"]) == manifest["timesteps_per_seed"]
        selected.append({
            "training_seed": seed,
            "selected_training_steps": best["timesteps"],
            "validation_mean_return": best["validation_mean_return"],
        })
    checks["selection_uses_validation_only"] = valid_selection
    checks["exact_per_seed_training_budget"] = correct_budgets

    states = {}
    distances = defaultdict(float)
    steps = Counter()
    stop_counts = Counter()
    motion_matches = True
    immediate_stop = True
    max_speed = max_acceleration = 0.0
    dt = manifest["dt_s"]
    with gzip.open(directory / "test_trajectories.csv.gz", "rt", newline="") as stream:
        for row in csv.DictReader(stream):
            row["training_seed"] = int(row["training_seed"]) if row["training_seed"] else None
            key = episode_key(row)
            pos = float(row["east_m"]), float(row["north_m"])
            vel = float(row["executed_east_m_s"]), float(row["executed_north_m_s"])
            old_pos, old_vel = states.get(key, (layouts[row["layout_id"]]["start"], (0.0, 0.0)))
            steps[key] += 1
            motion_matches &= int(row["step"]) == steps[key] and all(
                abs(new - old - dt * velocity) < 1e-8
                for new, old, velocity in zip(pos, old_pos, vel)
            )
            max_speed = max(max_speed, math.hypot(*vel))
            if row["action_reason"] == "minimum_range_stop":
                stop_counts[key] += 1
                immediate_stop &= vel == (0.0, 0.0)
            else:
                max_acceleration = max(max_acceleration, math.dist(vel, old_vel) / dt)
            distances[key] += math.dist(old_pos, pos)
            states[key] = pos, vel

    checks["executed_speed_within_limit"] = max_speed <= manifest["max_speed_m_s"] + 1e-8
    checks["regular_acceleration_within_limit"] = max_acceleration <= manifest["max_acceleration_m_s2"] + 1e-8
    checks["immediate_stop_bypass_is_explicit"] = immediate_stop and all(
        stop_counts[episode_key(row)] == row["guard_stop_steps"] for row in rows
    )
    checks["trajectory_steps_and_path_lengths_match_results"] = (
        motion_matches and set(steps) == expected_keys and all(
            steps[episode_key(row)] == row["steps"]
            and abs(distances[episode_key(row)] - row["path_length_m"]) < 1e-7 for row in rows
        )
    )
    checks["kinematic_scope"] = (
        summary["scope"] == manifest["scope"] == "CPU_2D_kinematic_diagnostic_not_Gazebo_PX4_flight"
    )

    failed = [key for key, passed in checks.items() if not passed]
    report = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "passed": not failed,
        "count": len(checks),
        "checks": checks,
        "selected_checkpoints": selected,
        "max_executed_speed_m_s": max_speed,
        "max_regular_acceleration_m_s2": max_acceleration,
        "scope": "local_kinematic_provenance_and_result_audit",
    }
    (directory / "independent_verification.json").write_text(json.dumps(report, indent=2) + "\n")
    if failed:
        raise ValueError("Verification failed for " + ", ".join(failed))
    return report
