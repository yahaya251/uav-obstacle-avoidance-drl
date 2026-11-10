"""Command-line entry points for the planar benchmark."""

import argparse
import json
import os
from pathlib import Path

from .profiles import PROFILES


def main():
    parser = argparse.ArgumentParser(description="Benchmark APF and PPO in a seeded planar environment.")
    subcommands = parser.add_subparsers(dest="command", required=True)
    run = subcommands.add_parser("run", help="Train and evaluate a frozen profile.")
    run.add_argument("--profile", choices=PROFILES, default="exploratory")
    run.add_argument("--output", type=Path, required=True, help="A new output directory.")
    for command, help_text in (
        ("verify", "Audit a completed local run and write independent_verification.json."),
        ("plot", "Render plots from a completed run."),
    ):
        subcommands.add_parser(command, help=help_text).add_argument("directory", type=Path)
    args = parser.parse_args()

    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    if args.command == "run":
        if args.output.exists():
            parser.error(f"Output already exists: {args.output}. Choose a new directory.")
        from .experiment import run_benchmark
        run_benchmark(args.output, args.profile, PROFILES[args.profile])
    elif args.command == "verify":
        from .verify import check_run
        audit = check_run(args.directory)
        print(json.dumps({
            "directory": str(args.directory),
            "passed": audit["passed"],
            "checks": audit["count"],
            "selected_checkpoints": audit["selected_checkpoints"],
        }, indent=2))
    else:
        from .plots import stage_plots
        for path in stage_plots(args.directory):
            print(path)


if __name__ == "__main__":
    main()
