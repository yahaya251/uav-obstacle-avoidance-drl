"""Run navigation and monitor contracts plus deterministic 2D fixtures."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
import unittest

from .scenarios import run_scenarios


class RecordedResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.details = []

    def addSuccess(self, test):
        super().addSuccess(test)
        self.details.append({"test": test.id(), "passed": True})

    def addFailure(self, test, error):
        super().addFailure(test, error)
        self.details.append({"test": test.id(), "passed": False, "kind": "failure"})

    def addError(self, test, error):
        super().addError(test, error)
        self.details.append({"test": test.id(), "passed": False, "kind": "error"})


def main():
    root = Path.cwd()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/core-checks"))
    args = parser.parse_args()
    tests_path = root / "tests"
    if not (tests_path / "test_core.py").is_file() or not (root / "simulation/monitor.py").is_file():
        parser.error("Run this command from the repository root.")
    try:
        args.output.mkdir(parents=True)
    except FileExistsError:
        parser.error(f"Output already exists: {args.output}. Choose a new directory.")

    tests = unittest.defaultTestLoader.discover(str(tests_path), pattern="test_*.py")
    log = args.output / "test_output.txt"
    with log.open("w", encoding="utf-8") as stream:
        result = unittest.TextTestRunner(
            stream=stream, verbosity=2, resultclass=RecordedResult
        ).run(tests)

    sources = [*sorted((root / "src/uav_nav").rglob("*.py")), *sorted(tests_path.glob("*.py"))]
    sources.extend(
        path for name in ("monitor.py", "dashboard.html")
        if (path := root / "simulation" / name).is_file()
    )
    report = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "scope": "navigation_and_monitor_contracts",
        "tests_run": result.testsRun,
        "failures": len(result.failures),
        "errors": len(result.errors),
        "passed": result.wasSuccessful(),
        "tests": result.details,
        "source_sha256": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sources
        },
        "flight_command_sent": False,
        "ppo_training_performed": False,
    }
    (args.output / "test_report.json").write_text(json.dumps(report, indent=2) + "\n")
    if not result.wasSuccessful():
        print(log.read_text(encoding="utf-8"), file=sys.stderr)
        return 1

    print(f"{result.testsRun} contract tests passed. Reports: {args.output}")
    for row in run_scenarios(args.output / "scenarios")["scenarios"]:
        info = row["outcome"]
        print(
            f'{row["scenario"]}: success={info["success"]}, collision={info["collision"]}, '
            f'timeout={info["timeout"]}, steps={info["steps"]}'
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
