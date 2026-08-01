from __future__ import annotations

import json
import threading
import time
from array import array
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

import cv2
import gi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>MTR Boat Camera</title>
  <style>
    :root { color-scheme: dark; }
    * { box-sizing: border-box; }
    html, body { width: 100%; height: 100%; margin: 0; background: #05080b; }
    body { overflow: hidden; font: 14px/1.4 system-ui, sans-serif; }
    #feed {
      position: fixed; left: 50%; top: 50%; width: 100vw; height: 100vh;
      object-fit: contain; display: block;
      transform: translate(-50%, -50%);
    }
    #status {
      position: fixed; top: 12px; left: 12px; padding: 8px 11px;
      border-radius: 8px; color: #eef7ff; background: #081018cc;
      border: 1px solid #ffffff24; backdrop-filter: blur(5px);
    }
    #status[data-ok="true"]::before { content: "● "; color: #4cff88; }
    #status[data-ok="false"]::before { content: "● "; color: #ff5964; }
    #hint {
      position: fixed; right: 12px; bottom: 12px; padding: 6px 9px;
      border-radius: 7px; color: #b9c7d3; background: #081018b8;
      cursor: pointer; user-select: none;
    }
  </style>
</head>
<body>
  <img id="feed" src="/stream.mjpg" alt="Live camera stream">
  <div id="status" data-ok="false">Connecting…</div>
  <div id="hint">Double-click: fullscreen · R: rotate</div>
  <script>
    const status = document.getElementById("status");
    const feed = document.getElementById("feed");
    const hint = document.getElementById("hint");
    let rotation = null;
    function setRotation(degrees) {
      const portrait = degrees === 90 || degrees === 270;
      feed.style.width = portrait ? "100vh" : "100vw";
      feed.style.height = portrait ? "100vw" : "100vh";
      feed.style.transform = `translate(-50%, -50%) rotate(${degrees}deg)`;
    }
    async function refreshStatus() {
      try {
        const value = await fetch("/health", {cache: "no-store"}).then(r => r.json());
        if (rotation === null) {
          const saved = Number(localStorage.getItem("mtr-camera-rotation"));
          rotation = [0, 90, 180, 270].includes(saved)
            ? saved : value.web_rotation_deg;
        }
        setRotation(rotation);
        status.dataset.ok = String(value.connected);
        feed.style.visibility = value.connected ? "visible" : "hidden";
        status.textContent = value.connected
          ? `${value.width}×${value.height} · ${value.fps.toFixed(1)} FPS · ${value.frames} frames`
          : `Camera disconnected — waiting for ${value.device}`;
      } catch (_) {
        feed.style.visibility = "hidden";
        status.dataset.ok = "false";
        status.textContent = "Stream server unavailable";
      }
    }
    setInterval(refreshStatus, 1000);
    refreshStatus();
    function rotate() {
      rotation = ((rotation ?? 0) + 90) % 360;
      localStorage.setItem("mtr-camera-rotation", String(rotation));
      setRotation(rotation);
    }
    hint.addEventListener("click", rotate);
    document.addEventListener("keydown", event => {
      if (event.key.toLowerCase() === "r") rotate();
    });
    document.addEventListener("dblclick", () => {
      if (!document.fullscreenElement) document.documentElement.requestFullscreen();
      else document.exitFullscreen();
    });
  </script>
</body>
</html>
"""


def frame_to_image(frame: Any, stamp: Any, frame_id: str) -> Image:
    """Convert one OpenCV BGR frame without depending on cv_bridge."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("camera frame must be an HxWx3 BGR image")
    if not frame.flags.c_contiguous:
        frame = frame.copy(order="C")

    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(frame.shape[0])
    message.width = int(frame.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = message.width * 3
    # rclpy's uint8[] setter copies Python bytes one integer at a time on
    # Humble. Passing the native array type avoids that multi-million-element
    # validation loop for every 720p frame.
    message.data = array("B", frame.tobytes())
    return message


class CameraHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], node: "CameraNode") -> None:
        self.camera_node = node
        super().__init__(address, CameraRequestHandler)


class CameraRequestHandler(BaseHTTPRequestHandler):
    server: CameraHttpServer

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/":
            self._send_bytes(
                INDEX_HTML.encode("utf-8"),
                "text/html; charset=utf-8",
            )
        elif path == "/health":
            payload = json.dumps(self.server.camera_node.status()).encode("utf-8")
            self._send_bytes(payload, "application/json", no_cache=True)
        elif path == "/snapshot.jpg":
            snapshot = self.server.camera_node.latest_jpeg()
            if snapshot is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No camera frame yet")
            else:
                self._send_bytes(snapshot, "image/jpeg", no_cache=True)
        elif path == "/stream.mjpg":
            self._stream_mjpeg()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _send_bytes(
        self,
        payload: bytes,
        content_type: str,
        *,
        no_cache: bool = False,
    ) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        if no_cache:
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        self.wfile.write(payload)

    def _stream_mjpeg(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        sequence = -1
        try:
            while not self.server.camera_node.stopping:
                item = self.server.camera_node.wait_for_jpeg(sequence, timeout=2.0)
                if item is None:
                    continue
                sequence, jpeg = item
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def log_message(self, _format: str, *_args: Any) -> None:
        return


def create_camera_http_server(
    enabled: bool,
    address: tuple[str, int],
    node: "CameraNode",
    logger: Any,
) -> CameraHttpServer | None:
    """Create the optional web server without making camera capture depend on it."""
    if not enabled:
        return None
    try:
        return CameraHttpServer(address, node)
    except OSError as exc:
        logger.error(
            f"Camera web viewer disabled because {address[0]}:{address[1]} "
            f"could not be bound: {exc}"
        )
        return None


class CameraNode(Node):
    """Own a UVC camera, publish ROS images, and serve a browser livestream."""

    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("device", "/dev/mtr_camera")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("publish_ros", True)
        self.declare_parameter("enable_web", True)
        self.declare_parameter("web_bind", "0.0.0.0")
        self.declare_parameter("web_port", 8081)
        self.declare_parameter("web_rotation_deg", 0)
        self.declare_parameter("reconnect_delay_s", 2.0)

        self.device = str(self.get_parameter("device").value)
        self.requested_width = max(int(self.get_parameter("width").value), 1)
        self.requested_height = max(int(self.get_parameter("height").value), 1)
        self.requested_fps = max(float(self.get_parameter("fps").value), 1.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.topic = str(self.get_parameter("topic").value)
        self.publish_ros = bool(self.get_parameter("publish_ros").value)
        self.enable_web = bool(self.get_parameter("enable_web").value)
        self.web_rotation_deg = int(self.get_parameter("web_rotation_deg").value)
        if self.web_rotation_deg not in (0, 90, 180, 270):
            raise ValueError("web_rotation_deg must be one of 0, 90, 180, or 270")
        self.reconnect_delay_s = max(
            float(self.get_parameter("reconnect_delay_s").value),
            0.1,
        )

        self.publisher = (
            self.create_publisher(Image, self.topic, qos_profile_sensor_data)
            if self.publish_ros
            else None
        )
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._stamp: Any | None = None
        self._sequence = -1
        self._frames = 0
        self._ros_frames = 0
        self._connected = False
        self._actual_width = 0
        self._actual_height = 0
        self._measured_fps = 0.0
        self._measured_ros_fps = 0.0
        self._last_frame_monotonic = 0.0
        self._pipeline: Gst.Pipeline | None = None

        web_bind = str(self.get_parameter("web_bind").value)
        web_port = int(self.get_parameter("web_port").value)
        if not 0 <= web_port <= 65535:
            raise ValueError("web_port must be between 0 and 65535")

        self._http_server = create_camera_http_server(
            self.enable_web,
            (web_bind, web_port),
            self,
            self.get_logger(),
        )
        self._http_thread = (
            threading.Thread(
                target=self._http_server.serve_forever,
                name="camera-http",
                daemon=True,
            )
            if self._http_server is not None
            else None
        )
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="camera-capture",
            daemon=True,
        )
        self._ros_thread = (
            threading.Thread(
                target=self._ros_publish_loop,
                name="camera-ros-publisher",
                daemon=True,
            )
            if self.publisher is not None
            else None
        )
        if self._http_thread is not None:
            self._http_thread.start()
        self._capture_thread.start()
        if self._ros_thread is not None:
            self._ros_thread.start()

        if self._http_server is not None:
            bound_host, bound_port = self._http_server.server_address[:2]
            self.get_logger().info(
                f"Camera web viewer listening on http://{bound_host}:{bound_port}/"
            )
        elif not self.enable_web:
            self.get_logger().info("Camera web viewer disabled by configuration")
        self.get_logger().info(
            f"Waiting for {self.device}; requested "
            f"native MJPEG {self.requested_width}x{self.requested_height} "
            f"at {self.requested_fps:g} FPS"
        )

    @property
    def stopping(self) -> bool:
        return self._stop_event.is_set()

    def _open_pipeline(self) -> tuple[Gst.Pipeline, Any] | None:
        requested_fps = max(int(round(self.requested_fps)), 1)
        description = (
            f'v4l2src device="{self.device}" io-mode=mmap ! '
            f"image/jpeg,width={self.requested_width},height={self.requested_height},"
            f"framerate={requested_fps}/1 ! "
            "appsink name=camera_sink sync=false max-buffers=1 drop=true"
        )
        try:
            pipeline = Gst.parse_launch(description)
        except Exception as exc:
            self.get_logger().error(f"Cannot construct camera pipeline: {exc}")
            return None

        sink = pipeline.get_by_name("camera_sink")
        if sink is None or pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            return None

        self._pipeline = pipeline
        self.get_logger().info(
            f"Opened {self.device}: native MJPEG "
            f"{self.requested_width}x{self.requested_height} "
            f"at {requested_fps} FPS"
        )
        return pipeline, sink

    def _capture_loop(self) -> None:
        sample_started = time.monotonic()
        sample_frames = 0
        warned_unavailable = False

        while not self.stopping and rclpy.ok():
            opened = self._open_pipeline()
            if opened is None:
                if not warned_unavailable:
                    self.get_logger().warning(
                        f"Cannot open {self.device}; retrying when the camera is available"
                    )
                    warned_unavailable = True
                self._set_disconnected()
                self._stop_event.wait(self.reconnect_delay_s)
                continue

            pipeline, sink = opened
            warned_unavailable = False
            sample_started = time.monotonic()
            sample_frames = 0

            while not self.stopping and rclpy.ok():
                sample = sink.emit("try-pull-sample", Gst.SECOND // 2)
                if sample is None:
                    bus = pipeline.get_bus()
                    message = bus.pop_filtered(
                        Gst.MessageType.ERROR | Gst.MessageType.EOS
                    )
                    if message is not None:
                        self.get_logger().warning(
                            f"Camera stream stopped on {self.device}; reconnecting"
                        )
                        break
                    continue

                buffer = sample.get_buffer()
                jpeg = buffer.extract_dup(0, buffer.get_size())
                caps = sample.get_caps()
                structure = caps.get_structure(0)
                _, actual_width = structure.get_int("width")
                _, actual_height = structure.get_int("height")
                now = time.monotonic()
                stamp = self.get_clock().now().to_msg()
                sample_frames += 1
                elapsed = now - sample_started
                if elapsed >= 1.0:
                    self._measured_fps = sample_frames / elapsed
                    sample_started = now
                    sample_frames = 0

                with self._condition:
                    self._jpeg = jpeg
                    self._stamp = stamp
                    self._sequence += 1
                    self._frames += 1
                    self._connected = True
                    self._actual_width = actual_width
                    self._actual_height = actual_height
                    self._last_frame_monotonic = now
                    self._condition.notify_all()

            pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._set_disconnected()
            if not self.stopping:
                self._stop_event.wait(self.reconnect_delay_s)

    def _ros_publish_loop(self) -> None:
        sequence = -1
        sample_started = time.monotonic()
        sample_frames = 0
        while not self.stopping and rclpy.ok():
            item = self._wait_for_frame(sequence, timeout=1.0)
            if item is None:
                continue
            sequence, jpeg, stamp = item
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning("Failed to decode a camera MJPEG frame")
                continue
            try:
                self.publisher.publish(frame_to_image(frame, stamp, self.frame_id))
            except Exception:
                if not rclpy.ok():
                    break
                raise

            now = time.monotonic()
            sample_frames += 1
            with self._condition:
                self._ros_frames += 1
            elapsed = now - sample_started
            if elapsed >= 1.0:
                with self._condition:
                    self._measured_ros_fps = sample_frames / elapsed
                sample_started = now
                sample_frames = 0

    def _set_disconnected(self) -> None:
        with self._condition:
            self._connected = False
            self._jpeg = None
            self._stamp = None
            self._actual_width = 0
            self._actual_height = 0
            self._measured_fps = 0.0
            self._last_frame_monotonic = 0.0
            self._condition.notify_all()

    def latest_jpeg(self) -> bytes | None:
        with self._condition:
            return self._jpeg

    def wait_for_jpeg(
        self,
        after_sequence: int,
        *,
        timeout: float,
    ) -> tuple[int, bytes] | None:
        with self._condition:
            self._condition.wait_for(
                lambda: (
                    (
                        self._sequence != after_sequence
                        and self._jpeg is not None
                    )
                    or self.stopping
                ),
                timeout=timeout,
            )
            if self._sequence == after_sequence or self._jpeg is None:
                return None
            return self._sequence, self._jpeg

    def _wait_for_frame(
        self,
        after_sequence: int,
        *,
        timeout: float,
    ) -> tuple[int, bytes, Any] | None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence != after_sequence or self.stopping,
                timeout=timeout,
            )
            if (
                self._sequence == after_sequence
                or self._jpeg is None
                or self._stamp is None
            ):
                return None
            return self._sequence, self._jpeg, self._stamp

    def status(self) -> dict[str, Any]:
        with self._condition:
            age_ms = (
                (time.monotonic() - self._last_frame_monotonic) * 1000.0
                if self._last_frame_monotonic
                else None
            )
            return {
                "connected": self._connected,
                "device": self.device,
                "width": self._actual_width,
                "height": self._actual_height,
                "fps": self._measured_fps,
                "frames": self._frames,
                "ros_fps": self._measured_ros_fps,
                "ros_frames": self._ros_frames,
                "age_ms": age_ms,
                "ros_topic": self.topic if self.publisher is not None else None,
                "frame_id": self.frame_id,
                "web_enabled": self.enable_web,
                "web_available": self._http_server is not None,
                "web_rotation_deg": self.web_rotation_deg,
            }

    def destroy_node(self) -> bool:
        if not self.stopping:
            self._stop_event.set()
            with self._condition:
                self._condition.notify_all()
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
            if self._http_server is not None:
                self._http_server.shutdown()
                self._http_server.server_close()
            self._capture_thread.join(timeout=3.0)
            if self._ros_thread is not None:
                self._ros_thread.join(timeout=3.0)
            if self._http_thread is not None:
                self._http_thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
