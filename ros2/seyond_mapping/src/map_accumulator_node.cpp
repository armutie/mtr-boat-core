#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {
struct VoxelKey {
  int64_t x, y, z;
  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};
struct VoxelHash {
  size_t operator()(const VoxelKey &key) const {
    size_t h = std::hash<int64_t>{}(key.x);
    h ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
struct Voxel {
  double x{0}, y{0}, z{0};
  uint32_t samples{0};
};
struct Pose {
  double tx, ty, tz, qx, qy, qz, qw;
};
uint64_t stampKey(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}
std::array<double, 3> transform(double x, double y, double z, const Pose &p) {
  const double tx = 2.0 * (p.qy * z - p.qz * y);
  const double ty = 2.0 * (p.qz * x - p.qx * z);
  const double tz = 2.0 * (p.qx * y - p.qy * x);
  return {x + p.qw * tx + (p.qy * tz - p.qz * ty) + p.tx,
          y + p.qw * ty + (p.qz * tx - p.qx * tz) + p.ty,
          z + p.qw * tz + (p.qx * ty - p.qy * tx) + p.tz};
}
}

class MapAccumulator final : public rclcpp::Node {
 public:
  MapAccumulator() : Node("map_accumulator_node") {
    voxel_size_ = declare_parameter<double>("voxel_size", 0.08);
    max_voxels_ = declare_parameter<int64_t>("max_voxels", 3000000);
    output_prefix_ = declare_parameter<std::string>("output_prefix", "maps/handheld_map");
    publish_period_ = declare_parameter<double>("publish_period", 2.0);
    if (voxel_size_ <= 0.0) throw std::runtime_error("voxel_size must be positive");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(*msg); });
    frame_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "frame", qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onFrame(*msg); });
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "map", rclcpp::QoS(1).transient_local().reliable());
    save_service_ = create_service<std_srvs::srv::Trigger>(
        "save_map", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = save();
          response->message = response->success
              ? "Saved " + output_prefix_ + ".pcd and .ply"
              : "Could not save map; see node log";
        });
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "clear_map", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          voxels_.clear(); poses_.clear(); pending_frames_.clear();
          response->success = true; response->message = "Map cleared";
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_),
                               [this]() { publishMap(); });
  }

 private:
  void onOdom(const nav_msgs::msg::Odometry &msg) {
    const auto &position = msg.pose.pose.position;
    const auto &orientation = msg.pose.pose.orientation;
    const uint64_t key = stampKey(msg.header.stamp);
    const Pose pose{position.x, position.y, position.z, orientation.x, orientation.y,
                    orientation.z, orientation.w};
    std::optional<sensor_msgs::msg::PointCloud2> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      poses_[key] = pose;
      while (poses_.size() > 20) poses_.erase(poses_.begin());
      auto frame = pending_frames_.find(key);
      if (frame != pending_frames_.end()) {
        pending = std::move(frame->second);
        pending_frames_.erase(frame);
      }
    }
    if (pending) accumulate(*pending, pose);
  }

  void onFrame(const sensor_msgs::msg::PointCloud2 &msg) {
    Pose pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = poses_.find(stampKey(msg.header.stamp));
      if (it == poses_.end()) {
        pending_frames_[stampKey(msg.header.stamp)] = msg;
        while (pending_frames_.size() > 5) pending_frames_.erase(pending_frames_.begin());
        return;
      }
      pose = it->second;
    }
    accumulate(msg, pose);
  }

  void accumulate(const sensor_msgs::msg::PointCloud2 &msg, const Pose &pose) {
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t count = static_cast<size_t>(msg.width) * msg.height;
    for (size_t i = 0; i < count; ++i, ++x, ++y, ++z) {
      const auto point = transform(*x, *y, *z, pose);
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) continue;
      VoxelKey key{static_cast<int64_t>(std::floor(point[0] / voxel_size_)),
                   static_cast<int64_t>(std::floor(point[1] / voxel_size_)),
                   static_cast<int64_t>(std::floor(point[2] / voxel_size_))};
      auto [it, inserted] = voxels_.try_emplace(key);
      if (inserted && static_cast<int64_t>(voxels_.size()) > max_voxels_) {
        voxels_.erase(it);
        continue;
      }
      auto &voxel = it->second;
      ++voxel.samples;
      const double n = voxel.samples;
      voxel.x += (point[0] - voxel.x) / n;
      voxel.y += (point[1] - voxel.y) / n;
      voxel.z += (point[2] - voxel.z) / n;
    }
  }

  std::vector<Voxel> snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Voxel> points;
    points.reserve(voxels_.size());
    for (const auto &[key, voxel] : voxels_) {
      (void)key;
      points.push_back(voxel);
    }
    return points;
  }

  void publishMap() {
    const auto points = snapshot();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "odom_lidar";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto &point : points) {
      *x = point.x; *y = point.y; *z = point.z;
      ++x; ++y; ++z;
    }
    cloud.is_dense = true;
    map_pub_->publish(std::move(cloud));
  }

  bool save() {
    const auto points = snapshot();
    if (points.empty()) {
      RCLCPP_ERROR(get_logger(), "Refusing to save an empty map");
      return false;
    }
    const std::filesystem::path prefix(output_prefix_);
    std::error_code error;
    if (!prefix.parent_path().empty()) {
      std::filesystem::create_directories(prefix.parent_path(), error);
      if (error) {
        RCLCPP_ERROR(get_logger(), "Cannot create output directory: %s", error.message().c_str());
        return false;
      }
    }
    std::ofstream pcd(output_prefix_ + ".pcd");
    std::ofstream ply(output_prefix_ + ".ply");
    if (!pcd || !ply) return false;
    pcd << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        << "COUNT 1 1 1\nWIDTH " << points.size() << "\nHEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
    ply << "ply\nformat ascii 1.0\nelement vertex " << points.size()
        << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    pcd << std::setprecision(7);
    ply << std::setprecision(7);
    for (const auto &point : points) {
      pcd << point.x << ' ' << point.y << ' ' << point.z << '\n';
      ply << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }
    RCLCPP_INFO(get_logger(), "Saved %zu voxels to %s.[pcd|ply]",
                points.size(), output_prefix_.c_str());
    return static_cast<bool>(pcd) && static_cast<bool>(ply);
  }

  double voxel_size_;
  int64_t max_voxels_;
  std::string output_prefix_;
  double publish_period_;
  std::mutex mutex_;
  std::unordered_map<VoxelKey, Voxel, VoxelHash> voxels_;
  std::unordered_map<uint64_t, Pose> poses_;
  std::unordered_map<uint64_t, sensor_msgs::msg::PointCloud2> pending_frames_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr frame_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAccumulator>());
  rclcpp::shutdown();
  return 0;
}
