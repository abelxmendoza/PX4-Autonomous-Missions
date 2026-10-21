"""Headless camera snapshot of a world: checks the VISUAL meshes.

drop_test checks collision geometry; this renders what cameras (and, by the
same loader, gpu_lidar/depth sensors) see. Findings on this machine:
  * the Sensors system needs render_engine "ogre" here; ogre2 produced no
    frames headless (EGL "failed to create dri2 screen"), matching
    config/gz_server_ogre.config
  * gz-sim renders a camera only while something subscribes to its topic,
    so a `gz topic -e` subscriber is started (same idea as scripts/gz_cam_sub)
"""
from __future__ import annotations

import glob
import os
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

SYSTEMS = [("gz-sim-physics-system", "gz::sim::systems::Physics"),
           ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
           ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
           ("gz-sim-sensors-system", "gz::sim::systems::Sensors")]


def snapshot(world_sdf, models_dir, out_png, pose="-3 -6 6 0 0.45 1.15",
             size=(1280, 720), engine="ogre", timeout_s=60):
    """Render one frame of world_sdf from `pose` (x y z roll pitch yaw)."""
    tree = ET.parse(world_sdf)
    world = tree.getroot().find("world")
    if world.find("plugin") is not None:
        raise ValueError("world declares its own plugins; snapshot adds the minimal set itself")
    with tempfile.TemporaryDirectory() as tmp:
        frames = Path(tmp) / "frames"
        frames.mkdir()
        for filename, name in SYSTEMS:
            plugin = ET.SubElement(world, "plugin", filename=filename, name=name)
            if filename == "gz-sim-sensors-system":
                ET.SubElement(plugin, "render_engine").text = engine
        world.append(ET.fromstring(f"""
<model name="snapshot_cam"><static>true</static><pose>{pose}</pose><link name="l">
<sensor name="cam" type="camera"><topic>/snapshot_cam</topic><update_rate>2</update_rate>
<always_on>1</always_on><camera><horizontal_fov>1.4</horizontal_fov>
<image><width>{size[0]}</width><height>{size[1]}</height></image>
<clip><near>0.1</near><far>300</far></clip><save enabled="true"><path>{frames}</path></save>
</camera></sensor></link></model>"""))
        world_file = Path(tmp) / "snapshot_world.sdf"
        tree.write(world_file)
        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=f"{Path(models_dir).resolve()}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}",
                   GZ_PARTITION=f"sim_asset_snap_{os.getpid()}")
        log = open(Path(tmp) / "gz.log", "w")
        server = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(world_file)],
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sub = None
        try:
            time.sleep(4)
            sub = subprocess.Popen(["gz", "topic", "-e", "-t", "/snapshot_cam"], env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
            deadline = time.monotonic() + timeout_s
            # wait for the 3rd frame: the first can be taken before textures stream in
            while time.monotonic() < deadline and len(glob.glob(f"{frames}/*.png")) < 3:
                time.sleep(1)
        finally:
            for proc in (sub, server):
                if proc:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait()
        shots = sorted(glob.glob(f"{frames}/*.png"), key=os.path.getmtime)
        if not shots:
            raise RuntimeError(f"no frame rendered (engine={engine}); see {log.name}")
        Path(out_png).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(shots[-1], out_png)
    return str(out_png)
