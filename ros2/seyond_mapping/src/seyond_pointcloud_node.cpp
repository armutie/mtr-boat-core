#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "seyond_mapping/point_cloud.hpp"

#include "src/sdk_common/inno_lidar_api.h"
#include "src/sdk_common/inno_lidar_other_api.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/inno_lidar_log.h"

class SeyondPointCloudNode final : public rclcpp::Node {
 public:
  SeyondPointCloudNode() : Node("seyond_pointcloud_node") {
    lidar_ip_ = declare_parameter<std::string>("lidar_ip", "172.168.1.10");
    lidar_port_ = declare_parameter<int>("lidar_port", 8010);
    udp_port_ = declare_parameter<int>("udp_port", 8010);
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_link");
    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 50.0);
    publish_second_return_ = declare_parameter<bool>("publish_second_return", true);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS());

    inno_lidar_set_log_level(INNO_LOG_LEVEL_WARNING);
    handle_ = inno_lidar_open_live("ros2", lidar_ip_.c_str(), lidar_port_,
                                   INNO_LIDAR_PROTOCOL_PCS_UDP, udp_port_);
    if (handle_ <= 0) {
      throw std::runtime_error("inno_lidar_open_live failed");
    }
    inno_lidar_set_callbacks_data_type(handle_, INNO_CALLBACK_XYZ_FRAME);
    const int callback_result = inno_lidar_set_callbacks(
        handle_, &SeyondPointCloudNode::messageCallback,
        &SeyondPointCloudNode::dataCallback, &SeyondPointCloudNode::statusCallback,
        nullptr, this);
    if (callback_result != 0) {
      inno_lidar_close(handle_);
      handle_ = -1;
      throw std::runtime_error("inno_lidar_set_callbacks failed");
    }
    if (inno_lidar_start(handle_) != 0) {
      inno_lidar_close(handle_);
      handle_ = -1;
      throw std::runtime_error("inno_lidar_start failed; check address, interface, and sensor");
    }
    RCLCPP_INFO(get_logger(), "Streaming %s:%d UDP %d -> points",
                lidar_ip_.c_str(), lidar_port_, udp_port_);
  }

  ~SeyondPointCloudNode() override {
    stopping_.store(true);
    if (handle_ > 0) {
      inno_lidar_stop(handle_);
      inno_lidar_close(handle_);
    }
  }

 private:
  static void messageCallback(int, void *context, uint32_t,
                              InnoMessageLevel level, InnoMessageCode code,
                              const char *message) {
    auto *self = static_cast<SeyondPointCloudNode *>(context);
    if (self->stopping_.load()) return;
    switch (level) {
      case INNO_MESSAGE_LEVEL_FATAL:
      case INNO_MESSAGE_LEVEL_CRITICAL:
      case INNO_MESSAGE_LEVEL_ERROR:
        RCLCPP_ERROR(self->get_logger(), "SDK message %d: %s", code, message);
        break;
      case INNO_MESSAGE_LEVEL_WARNING:
        RCLCPP_WARN(self->get_logger(), "SDK message %d: %s", code, message);
        break;
      default:
        break;
    }
  }

  static int statusCallback(int, void *, const InnoStatusPacket *) { return 0; }

  static int dataCallback(int, void *context, const InnoDataPacket *packet) {
    auto *self = static_cast<SeyondPointCloudNode *>(context);
    if (!self->stopping_.load() && packet != nullptr) self->publish(*packet);
    return 0;
  }

  void publish(const InnoDataPacket &packet) {
    auto cloud = seyond_mapping::make_point_cloud(
        packet, min_range_, max_range_, publish_second_return_);
    if (cloud.width == 0) return;
    cloud.header.frame_id = frame_id_;
    // The D1-R timestamp is device-relative unless external time sync is
    // configured. Stamp at host acquisition so this cloud can be fused with
    // the boat's GNSS, IMU, radar, and camera ROS streams.
    cloud.header.stamp = now();
    publisher_->publish(std::move(cloud));
  }

  std::string lidar_ip_;
  std::string frame_id_;
  int lidar_port_{8010};
  int udp_port_{8010};
  double min_range_{0.15};
  double max_range_{50.0};
  bool publish_second_return_{true};
  int handle_{-1};
  std::atomic<bool> stopping_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SeyondPointCloudNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("seyond_pointcloud_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
