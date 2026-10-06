#!/usr/bin/env python3
"""Live interop check of PX4SITLVehicle against a running PX4 SITL (REQ-HIL-003).

Start PX4 SITL first (e.g. `HEADLESS=1 make px4_sitl gz_x500` in PX4-Autopilot), then:

    python tools/sitl_smoke.py [--connection udpin:0.0.0.0:14540] [--out artifacts/sitl/sitl_smoke.json]

Writes the result file the requirement registry reads. Exit 0 only if telemetry
arrived AND both arm and disarm got an acknowledgement from PX4. An arm that PX4
*denies* (pre-arm checks) is still an acknowledgement and is recorded as such.
"""
import argparse
import datetime
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.comms.clock import SystemClock  # noqa: E402
from px4_offboard.vehicle.interface import CommandRejected, CommandTimeout  # noqa: E402
from px4_offboard.vehicle.mavlink_vehicle import PX4SITLVehicle  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udpin:0.0.0.0:14540")
    parser.add_argument("--telemetry-timeout", type=float, default=30.0)
    parser.add_argument("--px4-dir", type=Path, default=Path.home() / "PX4-Autopilot", help="only used to record the firmware version")
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts" / "sitl" / "sitl_smoke.json")
    args = parser.parse_args()

    clock = SystemClock()
    vehicle = PX4SITLVehicle(args.connection, clock=clock, command_timeout_s=5.0)
    result = {"connection": args.connection, "telemetry": False, "arm_ack": None, "disarm_ack": None}
    vehicle.connect()
    deadline = time.monotonic() + args.telemetry_timeout
    while vehicle.state is None and time.monotonic() < deadline:
        vehicle.send_heartbeat()
        vehicle.update(0.2)
    if vehicle.state is not None:
        for _ in range(25):  # a few seconds of heartbeats: PX4's GCS-link preflight check, plus rate figure
            vehicle.send_heartbeat()
            vehicle.update(0.2)
        health = vehicle.link_health()
        result.update(
            telemetry=True,
            state=vehicle.state.__dict__,
            rx_rate_hz=round(health.rx_rate_hz, 1),
            parser={k: getattr(vehicle.link.parser_stats, k) for k in ("frames", "bad_crc", "unknown_msg", "unsupported")},
        )
        for name, action in (("arm_ack", vehicle.arm), ("disarm_ack", vehicle.disarm)):
            try:
                action()
                result[name] = "accepted"
            except CommandRejected as exc:
                result[name] = f"denied: {exc}"  # PX4 answered: acknowledged, just not accepted
            except CommandTimeout:
                result[name] = None
            vehicle.update(0.5)
    vehicle.disconnect()

    def git(repo, *cmd):
        try:
            return subprocess.run(["git", "-C", str(repo), *cmd], capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return "unknown"

    result["provenance"] = {
        "recorded_at_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "px4_version": git(args.px4_dir, "describe", "--tags", "--always"),
        "repo_commit": git(ROOT, "rev-parse", "--short", "HEAD"),
        "note": "Recorded run against a local PX4 SITL (gz_x500, headless). Re-run tools/sitl_smoke.py to refresh.",
    }
    result["passed"] = bool(result["telemetry"] and result["arm_ack"] and result["disarm_ack"])
    result["detail"] = (
        f"telemetry={'yes' if result['telemetry'] else 'NO'} rx_rate={result.get('rx_rate_hz')} Hz; "
        f"arm: {result['arm_ack']}; disarm: {result['disarm_ack']}"
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, default=str) + "\n")
    print(result["detail"])
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
