import unittest

import numpy as np
from builtin_interfaces.msg import Time

from boat_ros.camera_node import INDEX_HTML, frame_to_image


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


if __name__ == "__main__":
    unittest.main()
