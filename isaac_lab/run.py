#!/usr/bin/env python3
"""Run the optional Isaac Lab static-layout simulator on an NVIDIA workstation."""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import platform
import re
import sys
from time import perf_counter, sleep


ROOT = Path(__file__).resolve().parents[1]
SCOPE = "isaac_lab_fixed_altitude_kinematic_simulation"
SPLIT_START = {"train": 0, "validation": 10_000, "test": 30_000, "diagnostic": 40_000}
DEFAULT_ENVS = {"demo": 1, "smoke": 1, "train": 32, "evaluate": 10, "benchmark": 16}
RUNTIME_VERSIONS = {"isaaclab": "3.0.0", "isaacsim": "6.2.0.0"}


def positive_integer(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("Expected a positive integer.")
    return value


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    descriptions = {
        "demo": "Run the project APF baseline with optional Kit visualization.",
        "smoke": "Check observation, movement, success and reset on an open route.",
        "train": "Train PPO on fixed training layouts with Isaac Lab and SB3.",
        "evaluate": "Compare APF, direct goal seeking, and optionally PPO on one held-out split.",
        "benchmark": "Measure headless vectorized rollout throughput after warm-up.",
    }
    for mode, description in descriptions.items():
        command = commands.add_parser(mode, help=description, description=description)
        command.add_argument("--num-envs", type=positive_integer, default=DEFAULT_ENVS[mode])
        command.add_argument("--seed", type=int, default=7, help="Runtime and, for training, PPO seed.")
        command.add_argument("--device", default="cuda:0", help="Simulator CUDA device, e.g. cuda:0.")
        command.add_argument("--output", type=Path, help="New output directory; default: runs/isaac/…")
        if mode == "demo":
            command.add_argument("--viz", choices=("none", "kit"), default="none", help="Use kit for a desktop window.")
            command.add_argument("--steps", type=positive_integer, default=240)
            command.add_argument("--keep-open", action="store_true",
                                 help="Keep the Kit viewport interactive after saving the demo results.")
            command.add_argument("--loop", action="store_true",
                                 help="Keep the APF demonstration running in Kit until the window closes.")
        elif mode == "train":
            command.add_argument("--timesteps", type=positive_integer, default=32768)
            command.add_argument("--rollout-steps", type=positive_integer, default=128)
            command.add_argument("--policy-device", default="cpu", help="SB3 device, e.g. cpu or cuda:0.")
        elif mode == "evaluate":
            command.add_argument("--suite", choices=("validation", "test"), default="test")
            command.add_argument("--checkpoint", type=Path, help="Optional PPO checkpoint from this project.")
        elif mode == "benchmark":
            command.add_argument("--steps", type=positive_integer, default=256)
            command.add_argument("--warmup", type=positive_integer, default=32)
    args = parser.parse_args()
    if not re.fullmatch(r"cuda:(0|[1-9]\d*)", args.device):
        parser.error("--device must be a CUDA device such as cuda:0.")
    if not 0 <= args.seed < 2**32:
        parser.error("--seed must be between 0 and 4294967295.")
    if args.num_envs > 200:
        parser.error("--num-envs must be at most 200, the size of the fixed training layout pool.")
    if args.mode == "demo" and args.viz != "kit" and (args.loop or args.keep_open):
        option = "--loop" if args.loop else "--keep-open"
        parser.error(f"{option} requires --viz kit.")
    if args.mode == "train":
        rollout_size = args.rollout_steps * args.num_envs
        if rollout_size < 2 or args.timesteps % rollout_size:
            parser.error("--timesteps must be a multiple of --rollout-steps × --num-envs; minimum rollout is 2.")
    if args.mode == "evaluate" and args.checkpoint and not args.checkpoint.is_file():
        parser.error(f"Checkpoint not found: {args.checkpoint}")
    return args


def choose_layouts(args):
    from uav_nav.benchmark.experiment import FAMILIES, make_layout

    split = args.suite if args.mode == "evaluate" else "train" if args.mode == "train" else "diagnostic"
    start = SPLIT_START[split]
    return split, [
        make_layout(start + index, "open" if args.mode == "smoke" else FAMILIES[index % len(FAMILIES)])
        for index in range(args.num_envs)
    ]


def write_json(path, payload):
    pending = path.with_suffix(".tmp")
    pending.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    pending.replace(path)


def direct_action(policy_obs):
    import torch

    goal = policy_obs[:, 36:38]
    return goal / torch.linalg.vector_norm(goal, dim=-1, keepdim=True).clamp_min(1e-12)


def observations_valid(policy_obs, num_envs):
    if policy_obs.shape != (num_envs, 40):
        raise RuntimeError("The policy must receive one 40-value observation per environment.")
    scan, goal_and_velocity = policy_obs[:, :36], policy_obs[:, 36:]
    return ((scan >= -1e-6) & (scan <= 1 + 1e-6)).all() & (goal_and_velocity.abs() <= 1 + 1e-6).all()


def rollout(env, layouts, action, *, seed, max_steps, name, verify_reset=False):
    """Retain the first finished episode in each simulator slot."""
    import torch

    obs, _ = env.reset(seed=seed)
    count = len(layouts)
    valid_observations = observations_valid(obs["policy"], count)
    finite_rewards = torch.ones_like(valid_observations)
    finished = [False] * count
    rows = []
    for _ in range(max_steps):
        with torch.no_grad():
            obs, reward, terminated, truncated, extras = env.step(action(obs["policy"]))
        valid_observations &= observations_valid(obs["policy"], count)
        if reward.shape != (count,):
            raise RuntimeError("Expected one reward per environment.")
        finite_rewards &= torch.isfinite(reward).all()
        done = (terminated | truncated).detach().cpu().tolist()
        new = [index for index, ended in enumerate(done) if ended and not finished[index]]
        if not new:
            continue
        metrics = extras["episode_outcome"]
        values = {key: tensor[new].detach().cpu().tolist() for key, tensor in metrics.items()}
        if verify_reset:
            values["reset_to_start"] = (
                (env.position[new] == env.geometry.start[new]).all(dim=1)
                & (env.velocity[new] == 0).all(dim=1)
            ).cpu().tolist()
        for offset, index in enumerate(new):
            outcome = {key: value[offset] for key, value in values.items()}
            if sum(bool(outcome[key]) for key in ("success", "collision", "timeout")) != 1:
                raise RuntimeError(f"Expected one terminal outcome for {layouts[index]['id']}.")
            rows.append({"controller": name, "layout_id": layouts[index]["id"],
                         "layout_seed": layouts[index]["layout_seed"], "family": layouts[index]["family"],
                         **outcome})
            finished[index] = True
        if all(finished):
            break
    if not bool(valid_observations):
        raise RuntimeError("Expected finite scan sectors in [0, 1] and goal and velocity values in [-1, 1].")
    if not bool(finite_rewards):
        raise RuntimeError("Expected finite rewards throughout the rollout.")
    return rows


def summarize(rows):
    names = sorted({row["controller"] for row in rows})
    summary = {}
    for name in names:
        selected = [row for row in rows if row["controller"] == name]
        outcomes = {key: sum(bool(row[key]) for row in selected) for key in ("success", "collision", "timeout")}
        summary[name] = {"episodes": len(selected), **outcomes, "success_rate": outcomes["success"] / len(selected)}
    return summary


def evaluate(env, layouts, args):
    controllers = {"APF": env.geometry.apf_action, "GoalSeeking": direct_action}
    if args.checkpoint:
        from stable_baselines3 import PPO

        policy = PPO.load(args.checkpoint, device="cpu")

        def predict(policy_obs):
            action, _ = policy.predict(policy_obs.detach().cpu().numpy(), deterministic=True)
            return policy_obs.new_tensor(action)

        controllers["PPO"] = predict
    rows = []
    for name, action in controllers.items():
        episodes = rollout(env, layouts, action, seed=args.seed, max_steps=env.max_episode_length + 1, name=name)
        if len(episodes) != len(layouts):
            raise RuntimeError(f"{name} finished only {len(episodes)} of {len(layouts)} held-out layouts.")
        rows.extend(sorted(episodes, key=lambda row: row["layout_seed"]))
    return {"episodes": rows, "summary": summarize(rows)}


def smoke(env, layouts, args):
    rows = rollout(env, layouts, direct_action, seed=args.seed, max_steps=env.max_episode_length + 1,
                   name="GoalSeeking", verify_reset=True)
    if len(rows) != len(layouts) or not all(row["success"] and row["path_length_m"] > 1.0 for row in rows):
        raise RuntimeError("Open-route smoke must move, reach the goal, and finish in every slot.")
    if not all(row["reset_to_start"] for row in rows):
        raise RuntimeError("Completed open routes did not automatically reset to their start.")
    return {"passed": True, "episodes": rows, "summary": summarize(rows)}


def train(env, args, output):
    from isaaclab_rl.sb3 import Sb3VecEnvWrapper
    from stable_baselines3 import PPO

    rollout_size = args.rollout_steps * args.num_envs
    batch_size = next((size for size in range(min(256, rollout_size), 1, -1) if rollout_size % size == 0), rollout_size)
    wrapped = Sb3VecEnvWrapper(env)
    model = PPO("MlpPolicy", wrapped, n_steps=args.rollout_steps, batch_size=batch_size, learning_rate=3e-4,
                device=args.policy_device, seed=args.seed, verbose=1)
    start = perf_counter()
    model.learn(total_timesteps=args.timesteps)
    elapsed = perf_counter() - start
    if model.num_timesteps != args.timesteps:
        raise RuntimeError(f"Unexpected number of measured transitions: {model.num_timesteps}.")
    model.save(output / "ppo.zip")
    return {"checkpoint": "ppo.zip", "timesteps": model.num_timesteps, "elapsed_seconds": elapsed,
            "rollout_steps": args.rollout_steps, "batch_size": batch_size, "policy_device": str(model.device)}


def benchmark(env, args):
    import torch

    with torch.no_grad():
        obs, _ = env.reset(seed=args.seed)
        for _ in range(args.warmup):
            obs, _, _, _, _ = env.step(direct_action(obs["policy"]))
        torch.cuda.synchronize(env.device)
        start = perf_counter()
        for _ in range(args.steps):
            obs, _, _, _, _ = env.step(direct_action(obs["policy"]))
        torch.cuda.synchronize(env.device)
        elapsed = perf_counter() - start
        if not bool(observations_valid(obs["policy"], args.num_envs)):
            raise RuntimeError("The benchmark ended with invalid observations.")
    return {"workload": "direct_goal_rollout_including_resets", "num_envs": args.num_envs,
            "warmup_vector_steps": args.warmup, "vector_steps": args.steps, "elapsed_seconds": elapsed,
            "vector_steps_per_second": args.steps / elapsed,
            "transitions_per_second": args.steps * args.num_envs / elapsed}


def frame_demo_layouts(env, half_world):
    origins = env.scene.env_origins
    bounds_min = origins.amin(dim=0)
    bounds_max = origins.amax(dim=0)
    east, north, up = ((bounds_min + bounds_max) / 2).tolist()
    span = float((bounds_max[:2] - bounds_min[:2]).max()) + 2 * half_world
    env.sim.set_camera_view(eye=(east + 0.3 * span, north - 0.6 * span, up + 1.1 * span),
                            target=(east, north, up))


def loop_demo(env, seed):
    """Run the APF controller at a watchable pace and let each slot autoreset."""
    import omni.timeline
    import torch

    timeline = omni.timeline.get_timeline_interface()
    with torch.no_grad():
        obs, _ = env.reset(seed=seed)
        next_step = perf_counter()
        while env.sim.is_running():
            now = perf_counter()
            if timeline.is_playing() and now >= next_step:
                obs, *_ = env.step(env.geometry.apf_action(obs["policy"]))
                next_step = now + env.step_dt
            else:
                env.sim.render()
                sleep(1 / 30)


def main():
    args = parse_arguments()
    system = platform.system()
    if (system not in {"Linux", "Windows"} or platform.machine().lower() not in {"x86_64", "amd64"}
            or sys.version_info[:2] != (3, 12)):
        raise SystemExit("Use the dedicated Python 3.12 Isaac environment on an NVIDIA RTX Windows or Linux x86-64 host.")
    setup = r".\isaac_lab\setup.ps1" if system == "Windows" else "bash isaac_lab/setup.sh"
    for package, expected in RUNTIME_VERSIONS.items():
        try:
            installed = version(package)
        except PackageNotFoundError as exc:
            raise SystemExit(f"Missing {package}. Run {setup} install.") from exc
        if installed != expected:
            raise SystemExit(f"Expected {package} {expected}, found {installed}. Run {setup} check.")
    try:
        from isaaclab.app import launch_simulation
        import torch
    except ModuleNotFoundError as exc:
        raise SystemExit(f"Isaac Lab dependency missing ({exc.name}). Run {setup} install.") from exc
    index = int(args.device.split(":")[1])
    if not torch.cuda.is_available() or index >= torch.cuda.device_count():
        raise SystemExit(f"CUDA device {args.device} is not available. Run {setup} check.")

    from uav_isaac.env_cfg import HALF_WORLD_M, UAVEnvCfg

    split, layouts = choose_layouts(args)
    cfg = UAVEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    cfg.seed = args.seed
    cfg.layouts = tuple(layouts)
    cfg.compute_final_obs = args.mode == "train"
    cfg.track_episode_outcomes = args.mode not in {"train", "benchmark"}
    now = datetime.now(timezone.utc)
    output = args.output or ROOT / "runs" / "isaac" / f"{args.mode}-{now:%Y%m%dT%H%M%S%fZ}"
    output.mkdir(parents=True, exist_ok=False)
    manifest = {
        "status": "running", "scope": SCOPE, "mode": args.mode, "started_at": now.isoformat(),
        "isaac_lab": RUNTIME_VERSIONS["isaaclab"], "isaac_sim": RUNTIME_VERSIONS["isaacsim"], "torch": torch.__version__,
        "physics": "isaacsim_physx", "device": args.device, "gpu": torch.cuda.get_device_name(index),
        "seed": args.seed, "split": split, "num_envs": args.num_envs,
        "platform": platform.platform(), "python": platform.python_version(),
        "track_episode_outcomes": cfg.track_episode_outcomes,
    }
    checkpoint = getattr(args, "checkpoint", None)
    if checkpoint is not None:
        with checkpoint.open("rb") as source:
            manifest["checkpoint_sha256"] = hashlib.file_digest(source, "sha256").hexdigest()
    write_json(output / "layouts.json", layouts)
    write_json(output / "manifest.json", manifest)
    launcher_args = {"device": args.device, "visualizer": getattr(args, "viz", "none"), "livestream": 0}
    keep_open = args.mode == "demo" and (args.keep_open or args.loop)
    with ExitStack() as runtime:
        try:
            script_args = sys.argv
            sys.argv = script_args[:1]
            try:
                portable_root = os.environ.get("UAV_ISAAC_PORTABLE_ROOT")
                if portable_root:
                    portable_dir = Path(portable_root).resolve()
                    portable_dir.mkdir(parents=True, exist_ok=True)
                    # KitLauncher splits kit_args on whitespace. SimulationApp also reads
                    # argv and preserves each element, including paths containing spaces.
                    sys.argv.extend(("--portable", "--portable-root", str(portable_dir)))
                runtime.enter_context(launch_simulation(cfg, launcher_args))
            finally:
                sys.argv = script_args

            from uav_isaac.env import UAVEnv

            env = UAVEnv(cfg=cfg)
            try:
                if args.mode == "train":
                    result = train(env, args, output)
                elif args.mode == "evaluate":
                    result = evaluate(env, layouts, args)
                elif args.mode == "benchmark":
                    result = benchmark(env, args)
                elif args.mode == "smoke":
                    result = smoke(env, layouts, args)
                else:
                    if args.viz == "kit":
                        if not env.sim.has_gui:
                            raise RuntimeError("Kit did not provide a visible simulation window.")
                        frame_demo_layouts(env, HALF_WORLD_M)
                    rows = rollout(env, layouts, env.geometry.apf_action, seed=args.seed, max_steps=args.steps,
                                   name="APF")
                    result = {"episodes": rows, "summary": summarize(rows), "finished_slots": len(rows),
                              "unfinished_slots": len(layouts) - len(rows), "step_limit": args.steps}
                # Save before the optional inspection loop and before Kit fast shutdown.
                write_json(output / "results.json", result)
                write_json(output / "manifest.json", {**manifest, "status": "completed"})
                print(f"Completed Isaac Lab {args.mode}. Results: {output.resolve()}", flush=True)
                if keep_open:
                    try:
                        if args.loop:
                            print("Looping the APF demonstration. Close the Kit window to finish.", flush=True)
                            loop_demo(env, args.seed)
                        else:
                            print("The Kit viewport stays open until you close its window.", flush=True)
                            while env.sim.is_running():
                                env.sim.render()
                                sleep(1 / 30)
                    except KeyboardInterrupt:
                        pass
            finally:
                env.close()
        except KeyboardInterrupt:
            write_json(output / "manifest.json", {**manifest, "status": "interrupted"})
            raise
        except (Exception, SystemExit) as exc:
            write_json(output / "manifest.json", {**manifest, "status": "failed", "error": str(exc)})
            raise
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
