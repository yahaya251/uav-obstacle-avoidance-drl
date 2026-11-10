#!/usr/bin/env python3
"""Run and supervise the passive PX4, Gazebo, ROS 2 and dashboard processes."""

import argparse
from datetime import datetime, timezone
import fcntl
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

from build_assets import SCAN_TOPIC, VEHICLE, WORLD, build_assets
from check import PORT, fetch_status, status_errors

SIMULATION_DIR = Path(__file__).resolve().parent
RUNS_DIR = SIMULATION_DIR.parent / "runs"
DDS_PORT = 8888


class Stack:
    def __init__(self, output):
        self.output = output
        self.children = []

    def start(self, name, command, **kwargs):
        with (self.output / f"{name}.log").open("wb") as log:
            child = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, **kwargs,
            )
        self.children.append((name, child))

    def verify_children(self):
        for name, child in self.children:
            if child.poll() is not None:
                raise RuntimeError(f"{name} exited ({child.returncode}). See {self.output / (name + '.log')}.")

    def close(self):
        for _, child in reversed(self.children):
            for sig, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
                if child.poll() is not None:
                    break
                try:
                    os.killpg(child.pid, sig)
                    child.wait(timeout=timeout)
                except ProcessLookupError:
                    child.wait()
                except subprocess.TimeoutExpired:
                    continue
                break


def ensure_port_free(port, socket_type):
    with socket.socket(socket.AF_INET, socket_type) as probe:
        if socket_type == socket.SOCK_STREAM:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as error:
            raise RuntimeError(f"Port {port} is in use. Stop its existing service and retry.") from error


def wait_for_gazebo(stack):
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        stack.verify_children()
        try:
            service = subprocess.run(
                ["gz", "service", "-i", "-s", f"/world/{WORLD}/scene/info"],
                capture_output=True, text=True, timeout=2,
            )
            if service.returncode == 0 and "Service providers" in service.stdout:
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    raise RuntimeError("Gazebo scene service did not appear within 45 seconds.")


def wait_for_monitor(stack):
    deadline = time.monotonic() + 90
    errors = ["Monitor API is starting."]
    while time.monotonic() < deadline:
        stack.verify_children()
        try:
            errors = status_errors(fetch_status())
            if not errors:
                return
        except (OSError, ValueError) as error:
            errors = [str(error)]
        time.sleep(0.5)
    raise RuntimeError(f"Monitor did not become ready within 90 seconds. {' '.join(errors)}")


def path_setting(key, default):
    return Path(os.environ.get(key, default)).expanduser().resolve()


def start_stack(stack):
    deps = path_setting("UAV_SIM_DEPS", Path.home() / ".local/share/uav-nav-sim")
    px4 = path_setting("PX4_ROOT", deps / "PX4-Autopilot")
    px4_bin = px4 / "build/px4_sitl_default/bin/px4"
    gz_assets = px4 / "Tools/simulation/gz"
    agent = path_setting("XRCE_AGENT", deps / "xrce/bin/MicroXRCEAgent")
    xrce_lib = path_setting("XRCE_LIB_DIR", deps / "xrce/lib")
    for command in ("gz", "ros2"):
        if shutil.which(command) is None:
            raise RuntimeError(f"Missing {command}. Follow simulation/README.md.")
    for path in (px4_bin, agent):
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f"Missing executable {path}. Follow simulation/README.md.")
    if not (gz_assets / "models/x500/model.sdf").is_file():
        raise RuntimeError("PX4 Gazebo assets are missing. Initialize the PX4 git submodules.")
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("An X11 display is required for the Ogre LiDAR. Use Xvfb as documented.")

    assets = stack.output / "assets"
    build_assets(assets)
    world_file = assets / f"worlds/{WORLD}.sdf"
    env = os.environ.copy()
    resources = [str(assets / "models"), str(gz_assets / "models"), str(gz_assets / "worlds")]
    for key in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
        env[key] = os.pathsep.join(resources + ([env[key]] if env.get(key) else []))
    env["LC_MESSAGES"] = "C.UTF-8"
    validation_log = stack.output / "sdf-validation.log"
    with validation_log.open("wb") as log:
        result = subprocess.run(
            ["gz", "sdf", "-k", str(world_file)], stdout=log, stderr=subprocess.STDOUT,
            env=env, timeout=30,
        )
    if result.returncode:
        raise RuntimeError(f"Invalid world. See {validation_log}.")

    stack.start(
        "gazebo-server", ["gz", "sim", "-r", "-s", str(world_file), "--render-engine", "ogre", "-v", "3"],
        env=env,
    )
    wait_for_gazebo(stack)
    agent_env = env.copy()
    agent_env["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(xrce_lib)] + ([env["LD_LIBRARY_PATH"]] if env.get("LD_LIBRARY_PATH") else [])
    )
    stack.start("agent", [str(agent), "udp4", "-p", str(DDS_PORT)], env=agent_env)

    px4_env = {
        **env,
        "HEADLESS": "1",
        "PX4_GZ_STANDALONE": "1",
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": "gz_x500",
        "PX4_GZ_WORLD": WORLD,
        "PX4_GZ_MODEL_NAME": VEHICLE,
    }
    stack.start("px4", [str(px4_bin), "-d"], cwd=px4, env=px4_env)
    stack.start("scan-bridge", [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        f"{SCAN_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ], env=env)
    stack.start("monitor-node", [
        sys.executable, str(SIMULATION_DIR / "monitor.py"),
        "--output", str(stack.output / "monitor"), "--port", str(PORT),
    ], env=env)
    wait_for_monitor(stack)


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new directory for assets, status, and logs")
    parser.add_argument("--duration", type=float, default=0, help="seconds after readiness, 0 for continuous")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be a finite nonnegative number.")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = (args.output or RUNS_DIR / f"live-{stamp}").resolve()
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGTERM, interrupt)
    with (RUNS_DIR / ".simulation.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("A simulation is already running from this checkout.")
        stack = Stack(output)
        try:
            ensure_port_free(PORT, socket.SOCK_STREAM)
            ensure_port_free(DDS_PORT, socket.SOCK_DGRAM)
            output.mkdir(parents=True)
            print(f"Starting passive simulation. Logs: {output}", flush=True)
            start_stack(stack)
            print(f"Ready: http://127.0.0.1:{PORT}/ (Ctrl+C to stop)", flush=True)
            deadline = time.monotonic() + args.duration if args.duration else math.inf
            while time.monotonic() < deadline:
                stack.verify_children()
                time.sleep(0.25)
        except KeyboardInterrupt:
            return 130
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            print(f"Simulation failed: {error}", file=sys.stderr)
            return 1
        finally:
            # A repeated Ctrl+C should not interrupt teardown.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            stack.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
