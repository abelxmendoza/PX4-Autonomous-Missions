"""CameraFeed.close() must leave no gz-transport handler holding a Python callback.

A handler that outlives close() lets gz's C++ thread take the GIL during
interpreter shutdown; integrated_seeding_sitl.py then exited 139 or 134 after
writing a complete report.
"""

import os
import threading
import time

import pytest

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
image_pb2 =pytest.importorskip("gz.msgs10.image_pb2")
transport = pytest.importorskip("gz.transport13")

from px4_offboard import gz_camera_feed  # noqa: E402

TOPIC = "/camera_feed_test/image"


def test_close_releases_callback_while_frames_arrive(monkeypatch):
    monkeypatch.setenv("GZ_IP", "127.0.0.1")
    monkeypatch.setenv("GZ_PARTITION", f"camera_feed_test_{os.getpid()}")
    monkeypatch.setattr(gz_camera_feed, "_cam_sub_binary", lambda: "/bin/true")
    publisher = transport.Node().advertise(TOPIC, image_pb2.Image)
    frame = image_pb2.Image(width=4, height=2, step=12, data=bytes(24))
    stop = threading.Event()

    def publish():
        while not stop.is_set():
            publisher.publish(frame)
            time.sleep(0.001)

    feed = gz_camera_feed.CameraFeed(TOPIC)
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while feed.latest()[1] == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert feed.latest()[1] > 0, "no frames delivered"
        # Close while frames are still being published, so a delivery can be in flight.
        assert feed.close() is True
        count = feed.latest()[1]
        time.sleep(0.1)
        assert feed.latest()[1] == count
        assert feed.close() is True
    finally:
        stop.set()
        thread.join(timeout=2.0)
