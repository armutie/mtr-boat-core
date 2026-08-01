import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time

from boat_ros.camera_node import (
    INDEX_HTML,
    CameraNode,
    create_camera_http_server,
    frame_to_image,
)


class CameraNodeTests(unittest.TestCase):
    def test_bgr_frame_is_converted_to_ros_image(self) -> None:
        frame = np.arange(2 * 3 * 3, dtype=np.uint8).reshape((2, 3, 3))
        stamp = Time(sec=12, nanosec=34)

        message = frame_to_image(frame, stamp, "camera_optical_frame")

        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual(message.header.frame_id, "camera_optical_frame")
        self.assertEqual(message.height, 2)
        self.assertEqual(message.width, 3)
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 9)
        self.assertEqual(bytes(message.data), frame.tobytes())

    def test_browser_page_uses_stream_and_health_endpoints(self) -> None:
        self.assertIn('src="/stream.mjpg"', INDEX_HTML)
        self.assertIn('fetch("/health"', INDEX_HTML)
        self.assertIn('feed.style.visibility = value.connected', INDEX_HTML)

    def test_web_server_can_be_disabled(self) -> None:
        logger = Mock()

        server = create_camera_http_server(
            False,
            ("0.0.0.0", 8081),
            Mock(),
            logger,
        )

        self.assertIsNone(server)
        logger.error.assert_not_called()

    def test_web_bind_failure_is_nonfatal(self) -> None:
        logger = Mock()
        with patch(
            "boat_ros.camera_node.CameraHttpServer",
            side_effect=OSError("address already in use"),
        ):
            server = create_camera_http_server(
                True,
                ("0.0.0.0", 8081),
                Mock(),
                logger,
            )

        self.assertIsNone(server)
        logger.error.assert_called_once()

    def test_disconnect_invalidates_cached_frame(self) -> None:
        state = SimpleNamespace(
            _condition=threading.Condition(),
            _connected=True,
            _jpeg=b"stale jpeg",
            _stamp=Time(sec=12, nanosec=34),
            _actual_width=1280,
            _actual_height=720,
            _measured_fps=30.0,
            _last_frame_monotonic=1.0,
        )

        CameraNode._set_disconnected(state)

        self.assertFalse(state._connected)
        self.assertIsNone(state._jpeg)
        self.assertIsNone(state._stamp)
        self.assertEqual(state._actual_width, 0)
        self.assertEqual(state._actual_height, 0)
        self.assertEqual(state._measured_fps, 0.0)
        self.assertEqual(state._last_frame_monotonic, 0.0)

    def test_wait_for_jpeg_blocks_until_fresh_frame_after_disconnect(
        self,
    ) -> None:
        state = SimpleNamespace(
            _condition=threading.Condition(),
            _sequence=4,
            _jpeg=None,
            stopping=False,
        )
        result = []
        done = threading.Event()

        def wait_for_frame() -> None:
            result.append(
                CameraNode.wait_for_jpeg(state, -1, timeout=1.0)
            )
            done.set()

        thread = threading.Thread(target=wait_for_frame)
        thread.start()
        try:
            self.assertFalse(done.wait(0.05))
            with state._condition:
                state._sequence = 5
                state._jpeg = b"fresh jpeg"
                state._condition.notify_all()

            self.assertTrue(done.wait(0.5))
            self.assertEqual(result, [(5, b"fresh jpeg")])
        finally:
            state.stopping = True
            with state._condition:
                state._condition.notify_all()
            thread.join(timeout=0.5)


if __name__ == "__main__":
    unittest.main()
