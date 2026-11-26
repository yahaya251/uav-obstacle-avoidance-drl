#!/usr/bin/env python3
"""Track one managed simulator and stop only the exact launcher process."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import tempfile
import time

STATE = Path(os.environ.get("UAV_RUNTIME_STATE", Path.home() / "uav-nav-runtime")).expanduser().resolve()
RUNS = STATE / "runs"
RECORD = STATE / "active-run.json"
LOCK = STATE / "active-run.lock"
BOOT_ID = Path("/proc/sys/kernel/random/boot_id")


def start_token(process):
    return (process / "stat").read_text().rsplit(")", 1)[1].split()[19]


def run(output, launcher, arguments):
    if output.parent != RUNS:
        raise ValueError(f"Managed simulation output must be under {RUNS}.")
    STATE.mkdir(parents=True, exist_ok=True)
    with LOCK.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("A managed simulation is already running. Use check or stop.")
        pid = os.getpid()
        record = {
            "pid": pid,
            "lock_fd": lock.fileno(),
            "start_token": start_token(Path("/proc/self")),
            "boot_id": BOOT_ID.read_text().strip(),
            "launcher": str(launcher.with_name("launch.py")),
            "output": str(output),
        }
        with tempfile.NamedTemporaryFile(mode="w", dir=STATE, delete=False) as staging:
            json.dump(record, staging)
        os.replace(staging.name, RECORD)
        os.set_inheritable(lock.fileno(), True)
        os.execv("/bin/bash", ["bash", str(launcher), "start", "--output", str(output), *arguments])


def owned_launcher(wait=0):
    """Return a pidfd and output only when kernel identity and argv both match."""
    try:
        record = json.loads(RECORD.read_text())
        output = Path(record["output"])
        launcher = Path(record["launcher"])
        token = record["start_token"]
        lock_fd = record["lock_fd"]
        if (record["boot_id"] != BOOT_ID.read_text().strip() or output.parent != RUNS
                or not launcher.is_absolute() or launcher.name != "launch.py"
                or not isinstance(record["pid"], int) or record["pid"] <= 0
                or not isinstance(lock_fd, int) or lock_fd < 3):
            return None
        process = Path(f"/proc/{record['pid']}")
        fd = os.pidfd_open(record["pid"])
    except (OSError, ValueError, KeyError, TypeError):
        return None

    deadline = time.monotonic() + wait
    expected = ["/usr/bin/python3", str(launcher), "--output", str(output)]
    try:
        while process.stat().st_uid == os.getuid() and start_token(process) == token:
            arguments = (process / "cmdline").read_bytes().split(b"\0")
            if ([os.fsdecode(argument) for argument in arguments[:4]] == expected
                    and os.path.samefile(process / "fd" / str(lock_fd), LOCK)):
                return fd, output
            if time.monotonic() >= deadline:
                break
            time.sleep(.1)
    except (OSError, ValueError, IndexError):
        pass
    os.close(fd)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    start = commands.add_parser("run")
    start.add_argument("output", type=Path)
    start.add_argument("launcher", type=Path)
    start.add_argument("arguments", nargs=argparse.REMAINDER)
    for action in ("stop", "status", "active-run"):
        commands.add_parser(action)
    args = parser.parse_args()
    if args.action == "run":
        run(args.output, args.launcher, args.arguments)

    process = owned_launcher(wait=3 if args.action == "stop" else 0)
    if process is None:
        if args.action != "active-run":
            print("No verified managed simulation is running.")
        return 1 if args.action == "active-run" else 0
    fd, output = process
    try:
        if args.action == "active-run":
            print(output)
        elif args.action == "status":
            print(f"Managed launcher running: {output}")
        else:
            try:
                signal.pidfd_send_signal(fd, signal.SIGTERM)
            except ProcessLookupError:
                print("Managed simulation already exited.")
                return 0
            watcher = select.poll()
            watcher.register(fd, select.POLLIN)
            if not watcher.poll(45_000):
                print(f"Shutdown is still in progress. Check {output}.")
                return 1
            print(f"Managed simulation stopped. Logs: {output}")
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
