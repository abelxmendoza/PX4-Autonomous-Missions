"""Latest frame from a Gazebo camera, outside ROS (used by the search tools).

Gazebo only renders a camera while a C++ gz-transport subscriber is attached; Python
subscriptions do not count (camera_bridge.py, BUG-017). So this also starts the repo's
gz_cam_sub on the topic. All consumers must run with GZ_IP=127.0.0.1, like PX4's server.
"""

from __future__ import annotations

import os
import subprocess
import threading
from pathlib import Path

os.environ.setdefault("GZ_IP", "127.0.0.1")
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SEARCH_DOWN_CAMERA_TOPIC = ("/world/search_field/model/x500_search_cam_0/link/camera_down_link"
                            "/sensor/down_imager/image")


def _cam_sub_binary() -> str:
    for p in (ROOT / "build" / "cpp" / "gz_cam_sub", ROOT / "scripts" / "gz_cam_sub"):
        if p.is_file() and os.access(p, os.X_OK):
            return str(p)
    raise RuntimeError("gz_cam_sub is not built: run `make cpp-build`")


class CameraFeed:
    def __init__(self, topic: str = SEARCH_DOWN_CAMERA_TOPIC) -> None:
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.transport13 import Node as GzNode

        self.topic = topic
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._count = 0
        self._waker = subprocess.Popen([_cam_sub_binary(), topic],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._node = GzNode()
        if not self._node.subscribe(GzImage, topic, self._on_image):
            self.close()
            raise RuntimeError(f"could not subscribe to {topic}")

    def _on_image(self, msg) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        with self._lock:
            self._frame = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
            self._count += 1

    def latest(self) -> tuple[np.ndarray | None, int]:
        """(BGR frame or None, number of frames received so far)."""
        with self._lock:
            return (None if self._frame is None else self._frame.copy()), self._count

    def close(self) -> None:
        self._waker.terminate()
