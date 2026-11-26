"""Compare observation construction against the original NumPy benchmark."""

import argparse
import json
import platform
import statistics
import time

import torch

from uav_isaac.geometry import PlanarGeometry
from uav_nav.benchmark.experiment import FAMILIES, make_layout, observation
from uav_nav.kinematics import KinematicEnv


def measure(work, repeat, iterations, warmup):
    for _ in range(warmup):
        work()
    samples = []
    for _ in range(repeat):
        start = time.perf_counter()
        for _ in range(iterations):
            work()
        samples.append(1000.0 * (time.perf_counter() - start) / iterations)
    return statistics.median(samples)


def compare(count, repeat, iterations, warmup):
    layouts = [
        make_layout(30000 + i, FAMILIES[i % len(FAMILIES)]) for i in range(count)
    ]
    worlds = []
    for layout in layouts:
        env = KinematicEnv()
        env.reset(options=layout)
        worlds.append(env)
    geometry = PlanarGeometry(layouts)
    velocity = torch.zeros_like(geometry.start)

    def original():
        return [observation(env) for env in worlds]

    def batched():
        return geometry.observe(geometry.start, velocity)[0]

    expected = torch.tensor([item.values for item in original()], dtype=torch.float32)
    actual = batched()
    torch.testing.assert_close(actual, expected, atol=2e-4, rtol=0)
    reference_ms = measure(original, repeat, iterations, warmup)
    batched_ms = measure(batched, repeat, iterations, warmup)
    return {
        "layouts": count,
        "reference_ms_per_batch": reference_ms,
        "torch_ms_per_batch": batched_ms,
        "reference_over_torch": reference_ms / batched_ms,
        "max_absolute_observation_error": (actual - expected).abs().max().item(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", type=int, nargs="+", default=[1, 8, 32, 64])
    parser.add_argument("--repeat", type=int, default=7)
    parser.add_argument("--iterations", type=int, default=120)
    parser.add_argument("--warmup", type=int, default=20)
    args = parser.parse_args()
    if min(*args.counts, args.repeat, args.iterations) < 1 or args.warmup < 0:
        parser.error("counts, repeat and iterations must be positive, and warmup must be nonnegative")
    torch.set_num_threads(1)
    print(json.dumps({
        "scope": "cpu_observation_only",
        "platform": platform.platform(),
        "torch": torch.__version__,
        "layout_seed_start": 30000,
        "repeat": args.repeat,
        "iterations": args.iterations,
        "warmup": args.warmup,
        "results": [compare(n, args.repeat, args.iterations, args.warmup) for n in args.counts],
    }, indent=2))


if __name__ == "__main__":
    main()
