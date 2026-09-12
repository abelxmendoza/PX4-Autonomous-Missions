#!/usr/bin/env python3
"""Own and verify an isolated, headless two-PX4/Gazebo survey run.

Run from a shell with ROS 2 and px4_msgs sourced. All processes and artifacts
are local simulation; no hardware connections are opened.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-dir", default="~/PX4-Autopilot")
    parser.add_argument("--dropout", action="store_true", help="abort vehicle 2 just after takeoff")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--timeout", type=float, default=390.0)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    px4 = Path(args.px4_dir).expanduser().resolve()
    build = px4 / "build/px4_sitl_default"
    if not (build / "bin/px4").is_file():
        parser.error("build PX4 SITL before running this demo")
    from pymavlink import mavutil
    # The parent's environment remains untouched; isolate this simulation's
    # ROS discovery, Gazebo transport, DDS agent port and artifact files.
    run = root / "demo_artifacts/swarm" / (time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])
    run.mkdir(parents=True)
    (run / "runner.pid").write_text(str(os.getpid()))
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID="88", ROS_LOCALHOST_ONLY="0", ROS_LOG_DIR=str(run / "ros_logs"),
               GZ_PARTITION=f"swarm_{run.name}", GZ_IP="127.0.0.1", HEADLESS="1", PX4_GZ_STANDALONE="1",
               PX4_GZ_WORLD="obstacle_world", PX4_SYS_AUTOSTART="4001", PX4_SIM_MODEL="gz_x500",
               PX4_UXRCE_DDS_PORT="8889", PX4_PARAM_COM_RCL_EXCEPT="4",
               GZ_SIM_RESOURCE_PATH=str(px4 / "Tools/simulation/gz/models"),
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / "src/modules/simulation/gz_plugins"),
               GZ_SIM_SERVER_CONFIG_PATH=str(px4 / "src/modules/simulation/gz_bridge/server.config"),
               PYTHONPATH=str(root / "src/px4_offboard") + os.pathsep + env.get("PYTHONPATH", ""))
    # Instance-derived namespaces are required by the fixed demo identities.
    env.pop("PX4_UXRCE_DDS_NS", None)
    egl = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if Path(egl).exists():
        env["__EGL_VENDOR_LIBRARY_FILENAMES"] = egl
    children, outputs, links = [], [], []

    def start(name, command, extra=None, cwd=None):
        output = (run / f"{name}.log").open("w")
        outputs.append(output)
        process = subprocess.Popen(command, cwd=cwd or run, env=dict(env, **(extra or {})),
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
        return process

    def heartbeat():
        for link in links:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    print(f"Artifacts: {run}", flush=True)
    report = {"passed": False, "errors": ["run did not complete"]}
    try:
        gz = ["gz", "sim", "-v", "2", "-r", str(root / "worlds/obstacle_world.sdf")]
        if not args.gui:
            gz.insert(2, "-s")
        start("gazebo", gz)
        start("dds", ["MicroXRCEAgent", "udp4", "-p", "8889"])
        for instance, east in ((1, -3), (2, 7)):
            working = run / f"px4_{instance}"
            working.mkdir()
            start(f"px4_{instance}", [str(build / "bin/px4"), "-i", str(instance), "-w", str(working),
                                      "-d", str(build / "etc")],
                  {"PX4_GZ_MODEL_POSE": f"{east},0",
                   "PX4_PARAM_MPC_XY_VEL_MAX": "12",
                   "PX4_PARAM_MPC_XY_CRUISE": "10",
                   "PX4_PARAM_MPC_XY_VEL_ALL": "12",
                   "PX4_PARAM_MPC_ACC_HOR_MAX": "8"})
            links.append(mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + instance}",
                                                    source_system=255, source_component=190))
        # Start nodes directly from source so the runner can validate edits
        # before installation; the ROS launch exposes the same parameters.
        for instance in (1, 2):
            vehicle = f"px4_{instance}"
            start(f"controller_{instance}", [sys.executable, "-m", "px4_offboard.swarm_vehicle",
                  "--ros-args", "-r", f"__ns:=/{vehicle}", "-p", f"vehicle_id:={vehicle}",
                  "-p", f"target_system_id:={instance + 1}", "-p",
                  f"abort_after_ready_s:={0.2 if args.dropout and instance == 2 else 0.0}"])
            start(f"trail_{instance}", [sys.executable, "-m", "px4_offboard.flight_trail",
                  "--ros-args", "-r", f"__ns:=/{vehicle}", "-p", "input_mode:=swarm",
                  "-p", "world_name:=obstacle_world", "-p",
                  "trail_color:=" + ("[0.1, 0.95, 1.0, 1.0]" if instance == 1 else "[1.0, 0.3, 0.7, 1.0]"),
                  "-p", "route_color:=" + ("[0.2, 1.0, 0.45, 0.9]" if instance == 1 else "[1.0, 0.65, 0.15, 0.9]")])
        start("coordinator", [sys.executable, "-m", "px4_offboard.swarm_coordinator",
                              "--ros-args", "-p", f"log_dir:={run}"])
        deadline = time.monotonic() + args.timeout
        last_heartbeat = 0
        last_phase = None
        while time.monotonic() < deadline:
            if any(p.poll() is not None for _, p in children):
                report = {"passed": False, "errors": [f"process exited: {name}" for name, p in children if p.poll() is not None]}
                break
            now = time.monotonic()
            if now - last_heartbeat >= 0.5:
                heartbeat()
                last_heartbeat = now
            logs = list(run.glob("swarm_*.jsonl"))
            if logs:
                lines = logs[0].read_text().splitlines()
                try:
                    row = json.loads(lines[-1])
                except (ValueError, IndexError):
                    time.sleep(0.1)
                    continue
                phase = row.get("phase")
                if phase != last_phase and phase:
                    print(f"{phase}: {row.get('reason')}", flush=True)
                    last_phase = phase
                terminal = phase in {"COMPLETE", "ABORTED"}
                landed = len(row.get("vehicles", {})) == 2 and all(
                    v["state"] == "LANDED" and not v["armed"] and v["landed"]
                    for v in row.get("vehicles", {}).values())
                if terminal and landed:
                    from px4_offboard.swarm_verify import verify
                    report = verify(logs[0])
                    break
            time.sleep(0.1)
        else:
            report = {"passed": False, "errors": ["runner timeout"]}
    except KeyboardInterrupt:
        report = {"passed": False, "errors": ["interrupted"]}
    finally:
        # Own process groups only; never pkill unrelated PX4/Gazebo processes.
        for _, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in reversed(children):
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for link in links:
            link.close()
        for output in outputs:
            output.close()
        (run / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
