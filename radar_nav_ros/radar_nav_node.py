from __future__ import annotations

import json
from dataclasses import asdict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from radar_nav import NavConfig, RadarNavPipeline
from radar_nav.logging import output_to_record
from radar_nav_ros.point_cloud import cloud_to_points, points_to_cloud


def nav_config_from_params(node: Node) -> NavConfig:
    return NavConfig(
        min_y=node.get_parameter("min_y").value,
        max_y=node.get_parameter("max_y").value,
        lateral_limit=node.get_parameter("lateral_limit").value,
        min_snr_raw=node.get_parameter("min_snr_raw").value,
        cluster_eps_m=node.get_parameter("cluster_eps_m").value,
        front_half_width=node.get_parameter("front_half_width").value,
        alpha=node.get_parameter("alpha").value,
        front_on_thresh=node.get_parameter("front_on_thresh").value,
        front_off_thresh=node.get_parameter("front_off_thresh").value,
        command_lock_s=node.get_parameter("command_lock_s").value,
        throttle_down_alpha=node.get_parameter("throttle_down_alpha").value,
        throttle_up_alpha=node.get_parameter("throttle_up_alpha").value,
        steering_alpha=node.get_parameter("steering_alpha").value,
    )


class RadarNavNode(Node):
    def __init__(self) -> None:
        super().__init__("radar_nav_node")

        self.declare_parameter("raw_points_topic", "radar/raw_points")
        self.declare_parameter("filtered_points_topic", "radar/filtered_points")
        self.declare_parameter("clusters_topic", "radar/clusters_json")
        self.declare_parameter("nav_state_topic", "radar/nav_state_json")
        self.declare_parameter("output_frame_id", "")

        self.declare_parameter("min_y", 0.15)
        self.declare_parameter("max_y", 2.5)
        self.declare_parameter("lateral_limit", 1.2)
        self.declare_parameter("min_snr_raw", 120)
        self.declare_parameter("cluster_eps_m", 0.35)
        self.declare_parameter("front_half_width", 0.25)
        self.declare_parameter("alpha", 0.10)
        self.declare_parameter("front_on_thresh", 0.70)
        self.declare_parameter("front_off_thresh", 0.40)
        self.declare_parameter("command_lock_s", 0.35)
        self.declare_parameter("throttle_down_alpha", 0.18)
        self.declare_parameter("throttle_up_alpha", 0.06)
        self.declare_parameter("steering_alpha", 0.12)

        cfg = nav_config_from_params(self)
        cfg.clamp_values()
        self.pipeline = RadarNavPipeline(cfg)

        qos_depth = 10
        self.create_subscription(
            PointCloud2,
            self.get_parameter("raw_points_topic").value,
            self.on_raw_points,
            qos_depth,
        )
        self.filtered_points_pub = self.create_publisher(
            PointCloud2, self.get_parameter("filtered_points_topic").value, qos_depth
        )
        self.clusters_pub = self.create_publisher(String, self.get_parameter("clusters_topic").value, qos_depth)
        self.nav_state_pub = self.create_publisher(String, self.get_parameter("nav_state_topic").value, qos_depth)

        self.get_logger().info(
            f"Subscribed to {self.get_parameter('raw_points_topic').value}; "
            f"publishing {self.get_parameter('filtered_points_topic').value} "
            f"and {self.get_parameter('nav_state_topic').value}"
        )

    def on_raw_points(self, cloud: PointCloud2) -> None:
        points = cloud_to_points(cloud)
        output = self.pipeline.process_points(
            points,
            metadata={
                "source_topic": self.get_parameter("raw_points_topic").value,
                "source_frame_id": cloud.header.frame_id,
            },
        )

        frame_id = self.get_parameter("output_frame_id").value or cloud.header.frame_id
        self.filtered_points_pub.publish(points_to_cloud(output.filtered_points, cloud.header.stamp, frame_id))

        clusters_msg = String()
        clusters_msg.data = json.dumps([asdict(cluster) for cluster in output.clusters], separators=(",", ":"))
        self.clusters_pub.publish(clusters_msg)

        nav_msg = String()
        nav_msg.data = json.dumps(output_to_record(output), separators=(",", ":"))
        self.nav_state_pub.publish(nav_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
