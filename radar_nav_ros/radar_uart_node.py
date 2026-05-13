import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from mmwave_uart import MmwaveUartParser, send_cfg
from radar_nav_ros.point_cloud import points_to_cloud


class RadarUartNode(Node):
    def __init__(self) -> None:
        super().__init__("radar_uart_node")

        self.declare_parameter("cfg_port", "")
        self.declare_parameter("cfg_file", "")
        self.declare_parameter("data_port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("frame_id", "radar")
        self.declare_parameter("poll_hz", 30.0)
        self.declare_parameter("raw_points_topic", "radar/raw_points")

        cfg_port = self.get_parameter("cfg_port").value
        cfg_file = self.get_parameter("cfg_file").value
        if cfg_port and cfg_file:
            self.get_logger().info(f"Sending radar config {cfg_file} on {cfg_port}")
            send_cfg(cfg_port, cfg_file)

        self.parser = MmwaveUartParser(
            self.get_parameter("data_port").value,
            baud=int(self.get_parameter("baud").value),
        )
        self.frame_id = self.get_parameter("frame_id").value

        qos_depth = 10
        self.raw_points_pub = self.create_publisher(
            PointCloud2, self.get_parameter("raw_points_topic").value, qos_depth
        )

        poll_hz = float(self.get_parameter("poll_hz").value)
        self.timer = self.create_timer(1.0 / max(poll_hz, 1.0), self.poll_once)
        self.get_logger().info(
            f"Publishing radar UART frames from {self.get_parameter('data_port').value} "
            f"to {self.get_parameter('raw_points_topic').value}"
        )

    def poll_once(self) -> None:
        decoded = self.parser.read_decoded_frame()
        if decoded is None:
            return

        stamp = self.get_clock().now().to_msg()
        self.raw_points_pub.publish(points_to_cloud(decoded.get("combined_points", []), stamp, self.frame_id))

    def destroy_node(self) -> bool:
        self.parser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarUartNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
