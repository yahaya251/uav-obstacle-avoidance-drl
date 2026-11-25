#!/usr/bin/env python3
"""Reconcile a completed passive SITL capture with its ROS checks and raw samples."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text())


def validation_records(directory, prefix):
    decoder = json.JSONDecoder()
    for path in sorted(directory.glob(f"{prefix}-*.txt")):
        output = path.read_text().lstrip()
        while output:
            record, end = decoder.raw_decode(output)
            yield record
            output = output[end:].lstrip()


def finite_vector(value, length):
    return isinstance(value, list) and len(value) == length and all(
        type(item) in (int, float) and math.isfinite(item) for item in value
    )


def verify(directory):
    monitor = directory / "monitor"
    summary = read_json(monitor / "run_summary.json")
    status = read_json(monitor / "status.json")
    telemetry = monitor / "monitor.jsonl"
    digest = hashlib.sha256()
    samples = []
    with telemetry.open("rb") as stream:
        for line in stream:
            digest.update(line)
            samples.append(json.loads(line))
    with (monitor / "monitor.csv").open(newline="") as stream:
        csv_samples = list(csv.DictReader(stream))

    ready = [sample for sample in samples if sample["ready"] is True]
    times = [sample["monotonic_s"] for sample in samples]
    ready_span = ready[-1]["monotonic_s"] - ready[0]["monotonic_s"] if ready else 0
    checks = list(validation_records(directory / "validation", "check"))
    health = [row for row in checks if "observation_ready" in row]
    graphs = [row for row in checks if "explicit_application_publishers" in row]
    scans = list(validation_records(directory / "validation", "scan"))
    scan = scans[-1] if scans else {}
    counts = summary["message_counts"]
    results = {
        "sample_counts_match": bool(samples) and len(samples) == summary["samples"]
        == status["sample_count"] and len(ready) == summary["ready_samples"]
        == status["ready_samples"] and len(samples) - len(ready) == summary["rejected_samples"],
        "samples_are_sequential": [row["sample"] for row in samples]
        == list(range(1, len(samples) + 1)) and all(math.isfinite(t) for t in times)
        and all(a < b for a, b in zip(times, times[1:])),
        "csv_matches_sample_stream": len(csv_samples) == len(samples) and all(
            csv_row[key] == str(raw_row[key] if raw_row[key] is not None else "")
            for csv_row, raw_row in zip(csv_samples, samples) for key in csv_row
        ),
        # The capture clock starts at readiness, and the monitor writes at roughly 10 Hz.
        "ready_interval_covers_capture": ready_span >= 44.8 and summary["elapsed_s"] >= 45,
        "ready_samples_have_full_observations": bool(ready) and all(
            finite_vector(row["observation"], 40) and row["coverage"] == [True] * 36
            for row in ready
        ),
        "scan_and_telemetry_present": all(counts.get(name, 0) > 0 for name in (
            "scan", "odometry", "vehicle_status")),
        "passive_mode": status["run_id"] == summary["run_id"]
        and status["vehicle"]["armed"] is False
        and summary["explicit_application_publishers_created"] == 0
        and status["flight_command_sent"] is False and summary["flight_command_sent"] is False
        and all(row["flight_command_sent"] is False for row in samples),
        "ros_status_passed": bool(health) and all(
            row["passed"] is True and row["observation_ready"] is True
            and row["vehicle"]["armed"] is False for row in health
        ),
        "ros_graph_passed": bool(graphs) and all(
            row["passed"] is True and row["explicit_application_publishers"] == [] for row in graphs
        ),
        "lidar_passed": bool(scans) and all(
            row["passed"] is True and row["samples"] == 720 and row["nan_count"] == 0
            and abs(row["span_degrees"] - 360) < 0.001 for row in scans
        ),
    }
    return {
        "passed": all(results.values()),
        "run_directory": directory.name,
        "monitor_run_id": summary["run_id"],
        "checks": results,
        "total_samples": len(samples),
        "ready_samples": len(ready),
        "ready_elapsed_seconds": round(ready_span, 3),
        "monitor_elapsed_seconds": round(summary["elapsed_s"], 3),
        "message_counts": counts,
        "lidar_rays": scan.get("samples"),
        "lidar_span_degrees": scan.get("span_degrees"),
        "observation_dimensions": len(ready[-1]["observation"]) if ready else 0,
        "covered_sectors": sum(ready[-1]["coverage"]) if ready else 0,
        "armed": status["vehicle"]["armed"],
        "flight_command_sent": summary["flight_command_sent"],
        "telemetry_sha256": digest.hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, help="optional output report path")
    args = parser.parse_args()
    try:
        report = verify(args.directory)
    except (OSError, ValueError, KeyError, TypeError) as error:
        report = {"passed": False, "run_directory": args.directory.name, "error": str(error)}
    contents = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(contents)
    print(contents, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
