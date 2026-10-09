#!/usr/bin/env python3
"""Live window of the search drone's downward camera, with ArUco detections drawn on it.

    GZ_IP=127.0.0.1 python tools/search_camera_view.py [--topic T] [--snapshot out.png] [--no-window]

Gazebo only renders a camera while a C++ gz-transport subscriber is attached (Python
subscriptions do not count; see camera_bridge.py and BUG-017), so this starts the repo's
gz_cam_sub on the same topic. Everything here is display only: it does not command the
drone and it does not read the ground-truth target file.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("GZ_IP", "127.0.0.1")
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image as GzImage  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TOPIC = ("/world/search_field/model/x500_search_cam_0/link/camera_down_link"
                 "/sensor/down_imager/image")


def _cam_sub_binary() -> str:
    for p in (ROOT / "build" / "cpp" / "gz_cam_sub", ROOT / "scripts" / "gz_cam_sub"):
        if p.is_file() and os.access(p, os.X_OK):
            return str(p)
    raise SystemExit("gz_cam_sub not built: run `make cpp-build` (or launch full_stack once)")


class Latest:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.count = 0

    def on_image(self, msg: GzImage) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        with self.lock:
            self.frame = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
            self.count += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--snapshot", type=Path, help="also save the annotated frame here every second")
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()

    waker = subprocess.Popen([_cam_sub_binary(), args.topic], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    latest = Latest()
    node = GzNode()
    if not node.subscribe(GzImage, args.topic, latest.on_image):
        waker.terminate()
        raise SystemExit(f"could not subscribe to {args.topic}")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7 (same split as vision_marker_detect.py)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
    else:
        params, detector = cv2.aruco.DetectorParameters_create(), None
    seen: set[int] = set()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    last_snap = 0.0
    print(f"viewing {args.topic}", flush=True)
    try:
        while not stop.is_set():
            with latest.lock:
                frame = None if latest.frame is None else latest.frame.copy()
                frames = latest.count
            if frame is None:
                time.sleep(0.1)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = (detector.detectMarkers(gray) if detector
                               else cv2.aruco.detectMarkers(gray, dictionary, parameters=params))
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                new = {int(i) for i in np.ravel(ids)} - seen
                if new:
                    print(f"detected marker id(s) {sorted(new)} (frame {frames})", flush=True)
                seen |= new
            cv2.putText(frame, f"down camera  frame {frames}  ids seen: {sorted(seen)}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if args.snapshot and time.time() - last_snap > 1.0:
                cv2.imwrite(str(args.snapshot), frame)
                last_snap = time.time()
            if not args.no_window:
                cv2.imshow("Search drone - downward camera", frame)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.05)
    finally:
        waker.terminate()
        cv2.destroyAllWindows()
    print(f"ids seen: {sorted(seen)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
