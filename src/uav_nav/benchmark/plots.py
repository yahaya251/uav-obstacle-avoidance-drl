"""Create portable PNG and SVG figures from completed benchmark artifacts."""

import csv
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle


def label(row):
    seed = row["training_seed"]
    return row["algorithm"] if seed is None else f"{row['algorithm']} seed {seed}"


def save_figure(figure, directory, stem):
    outputs = [directory / f"{stem}.{extension}" for extension in ("png", "svg")]
    figure.savefig(outputs[0], dpi=170)
    figure.savefig(outputs[1])
    plt.close(figure)
    return outputs


def plot_outcomes(directory, rows):
    fig, ax = plt.subplots(figsize=(10, 5.2), constrained_layout=True)
    positions = list(range(len(rows)))
    bottom = [0] * len(rows)
    for key, color in (("success", "#286e4f"), ("collision", "#a92d38"), ("timeout", "#dca55c")):
        values = [row[key] for row in rows]
        ax.bar(positions, values, bottom=bottom, color=color, label=key.capitalize())
        bottom = [base + value for base, value in zip(bottom, values)]
    count = rows[0]["episodes"]
    for index, row in enumerate(rows):
        ax.text(index, count * 1.02, f"{row['success']} / {count}", ha="center")
    ax.set_xticks(positions, [label(row) for row in rows])
    ax.set_ylim(0, count * 1.16)
    ax.set_ylabel("Episodes")
    ax.set_title(f"Frozen {count}-layout test per controller", loc="left")
    ax.legend(ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.22), frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", alpha=0.15)
    ax.set_axisbelow(True)
    csv_path = directory / "test_outcome_counts.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("controller", "success", "collision", "timeout", "episodes"))
        writer.writerows(
            [label(row), *(row[key] for key in ("success", "collision", "timeout", "episodes"))]
            for row in rows
        )
    return [*save_figure(fig, directory, "test_outcome_counts"), csv_path]


def plot_learning(directory, seeds):
    fig, axes = plt.subplots(
        1, len(seeds), squeeze=False, figsize=(4.7 * len(seeds), 4.8), constrained_layout=True
    )
    curves = []
    for seed, ax in zip(seeds, axes[0]):
        folder = directory / f"ppo_seed_{seed}"
        with (folder / "progress.csv").open(newline="") as stream:
            progress = [
                (int(row["time/total_timesteps"]), float(row["rollout/ep_rew_mean"]))
                for row in csv.DictReader(stream) if row.get("rollout/ep_rew_mean")
            ]
        validation = json.loads((folder / "validation_scores.json").read_text())
        if progress:
            steps, rewards = zip(*progress)
            ax.plot(steps, rewards, marker=".", lw=1.6, color="#235c91", label="Training mean")
        ax.plot(
            [row["timesteps"] for row in validation],
            [row["validation_mean_return"] for row in validation],
            marker="o", ls="--", lw=1.2, color="#b15d20", label="Validation mean",
        )
        curves.extend((seed, step, reward, None) for step, reward in progress)
        curves.extend((seed, row["timesteps"], None, row["validation_mean_return"]) for row in validation)
        ax.set_title(f"PPO training seed {seed}", loc="left")
        ax.set_xlabel("Environment steps")
        ax.set_ylabel("Episode return")
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
        ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 0))
        ax.legend(fontsize=8, loc="best", frameon=False)
    fig.suptitle("Training and validation reward traces")
    csv_path = directory / "learning_curves.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("training_seed", "environment_steps", "training_reward_mean", "validation_reward_mean"))
        writer.writerows(curves)
    return [*save_figure(fig, directory, "training_validation_curves"), csv_path]


def plot_trajectories(directory, example_seed):
    split = json.loads((directory / "frozen_splits.json").read_text())
    examples = {}
    for row in split["test"]:
        examples.setdefault(row["family"], row)
    selected = {row["id"] for row in examples.values()}
    points = {}
    with gzip.open(directory / "test_trajectories.csv.gz", "rt", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["layout_id"] in selected and (
                row["algorithm"] != "PPO" or row["training_seed"] == str(example_seed)
            ):
                key = row["layout_id"], row["algorithm"]
                points.setdefault(key, []).append((float(row["east_m"]), float(row["north_m"])))

    fig, axes = plt.subplots(2, 3, figsize=(13.2, 8), constrained_layout=True)
    names = {
        "open": "Open", "single_obstacle": "Single obstacle", "staggered": "Staggered",
        "corridor": "Corridor", "u_trap": "U-trap",
    }
    for ax, layout in zip(axes.flat, examples.values()):
        for x, y, radius in layout["obstacles"]:
            ax.add_patch(Circle((x, y), radius, color="#aeb6be"))
        for algorithm, color in (("APF", "#286e4f"), ("GoalSeeking", "#b47d23"), ("PPO", "#315fac")):
            coords = [layout["start"], *points[(layout["id"], algorithm)]]
            name = f"PPO seed {example_seed}" if algorithm == "PPO" else algorithm
            ax.plot(*zip(*coords), color=color, lw=1.5, label=name)
        ax.scatter(*layout["start"], marker="o", s=22, c="black", zorder=5)
        ax.scatter(*layout["goal"], marker="*", s=80, c="#4f237c", zorder=5)
        ax.set_aspect("equal")
        ax.set_xlim(-6, 6)
        ax.set_ylim(-3.3, 3.3)
        ax.set_title(names[layout["family"]], loc="left")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(alpha=0.15)
    note = axes.flat[-1]
    note.axis("off")
    note.text(0.05, 0.85, "First frozen layout from each family\n\nPlanar kinematics", va="top")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    note.legend(handles, labels, loc="lower left", frameon=False)
    return save_figure(fig, directory, "frozen_example_trajectories")


def stage_plots(directory):
    directory = Path(directory)
    rows = json.loads((directory / "summary.json").read_text())["summaries"]
    seeds = json.loads((directory / "run_manifest.json").read_text())["training_seeds"]
    return [
        *plot_outcomes(directory, rows),
        *plot_learning(directory, seeds),
        *plot_trajectories(directory, seeds[0]),
    ]
