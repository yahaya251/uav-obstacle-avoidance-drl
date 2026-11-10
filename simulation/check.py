#!/usr/bin/env python3
"""Check the passive simulation status returned by the loopback HTTP server."""

import json
import math
from urllib.request import urlopen

PORT = 8086
SCHEMA = "uav-nav.live.v1"


def fetch_status():
    with urlopen(f"http://127.0.0.1:{PORT}/api/status", timeout=1) as response:
        return json.load(response)


def status_errors(status):
    if not isinstance(status, dict) or status.get("schema_version") != SCHEMA:
        return ["Unexpected monitor status schema."]
    errors = []
    if status.get("ready") is not True:
        errors.append("Observation is not ready.")
    age = status.get("monitor_sample_age_s")
    if not isinstance(age, (float, int)) or not 0 <= age <= 1:
        errors.append("Monitor tick is missing or stale.")
    scan = status.get("scan") or {}
    if scan.get("ray_count") != 720:
        errors.append("Expected 720 LiDAR rays.")
    observation = status.get("observation") or {}
    values = observation.get("values", [])
    coverage = observation.get("coverage", [])
    if len(values) != 40 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
        errors.append("Expected 40 finite observation values.")
    if len(coverage) != 36 or not all(coverage):
        errors.append("Expected complete coverage of all 36 sectors.")
    vehicle = status.get("vehicle") or {}
    vehicle_age = vehicle.get("received_age_s")
    if (vehicle.get("armed") is not False or not isinstance(vehicle_age, (float, int))
            or not 0 <= vehicle_age <= 2):
        errors.append("A recent disarmed PX4 vehicle state is required.")
    if (status.get("flight_command_sent") is not False
            or status.get("mode") != "PASSIVE_MONITOR_NO_AUTOPILOT_PUBLICATIONS"):
        errors.append("Unexpected monitor mode.")
    return errors


if __name__ == "__main__":
    try:
        status = fetch_status()
        errors = status_errors(status)
    except (OSError, ValueError) as error:
        raise SystemExit(f"Monitor unavailable: {error}") from error

    print(json.dumps({
        "passed": not errors,
        "errors": errors,
        "observation_ready": status.get("ready"),
        "rates_hz": status.get("rates_hz"),
        "message_counts": status.get("message_counts"),
        "vehicle": status.get("vehicle"),
    }, indent=2))
    raise SystemExit(bool(errors))
