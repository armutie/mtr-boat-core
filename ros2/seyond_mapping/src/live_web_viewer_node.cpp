#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;

namespace {
constexpr uint32_t kFrameMagic = 0x31564453;  // "SDV1", little endian.
constexpr size_t kHeaderBytes = 32;

template <typename T>
void append(std::vector<uint8_t> &buffer, const T &value) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&value);
  buffer.insert(buffer.end(), bytes, bytes + sizeof(T));
}

int fieldOffset(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  for (const auto &field : cloud.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float percentile(std::vector<float> values, double fraction) {
  if (values.empty()) return 0.0F;
  const size_t index = std::min(values.size() - 1,
                                static_cast<size_t>(fraction * (values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}
}  // namespace

class LiveWebViewer final : public rclcpp::Node {
 public:
  LiveWebViewer() : Node("live_web_viewer_node") {
    port_ = declare_parameter<int>("port", 8080);
    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    html_path_ = declare_parameter<std::string>("html_path", "");
    max_points_ = declare_parameter<int>("max_points", 80000);
    input_native_coordinates_ = declare_parameter<bool>("input_native_coordinates", false);
    if (html_path_.empty()) throw std::runtime_error("html_path parameter is required");
    std::ifstream html(html_path_);
    if (!html) throw std::runtime_error("Cannot open viewer HTML: " + html_path_);
    html_ = std::string(std::istreambuf_iterator<char>(html), {});

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { onCloud(*cloud); });
    server_thread_ = std::thread([this]() { serverLoop(); });
  }

  ~LiveWebViewer() override {
    stopping_.store(true);
    frame_cv_.notify_all();
    if (acceptor_) {
      beast::error_code error;
      acceptor_->close(error);
    }
    if (server_thread_.joinable()) server_thread_.join();
    std::lock_guard<std::mutex> lock(client_threads_mutex_);
    for (auto &thread : client_threads_) {
      if (thread.joinable()) thread.join();
    }
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2 &cloud) {
    const int x_offset = fieldOffset(cloud, "x");
    const int y_offset = fieldOffset(cloud, "y");
    const int z_offset = fieldOffset(cloud, "z");
    int reflectance_offset = fieldOffset(cloud, "intensity");
    if (reflectance_offset < 0) reflectance_offset = fieldOffset(cloud, "reflectance");
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_ERROR_ONCE(get_logger(), "PointCloud2 needs FLOAT32 x, y, and z fields");
      return;
    }

    const size_t input_count = static_cast<size_t>(cloud.width) * cloud.height;
    const size_t stride = std::max<size_t>(1, (input_count + max_points_ - 1) / max_points_);
    std::vector<float> points;
    std::vector<float> reflectances;
    points.reserve((input_count / stride + 1) * 4);
    reflectances.reserve(input_count / stride + 1);
    std::vector<float> heights;
    heights.reserve(input_count / stride + 1);
    float farthest = 0.0F;

    for (size_t index = 0; index < input_count; index += stride) {
      const size_t row = index / cloud.width;
      const size_t column = index % cloud.width;
      const size_t base = row * cloud.row_step + column * cloud.point_step;
      if (base + cloud.point_step > cloud.data.size()) break;
      float x, y, z, reflectance = 1.0F;
      std::memcpy(&x, cloud.data.data() + base + x_offset, sizeof(float));
      std::memcpy(&y, cloud.data.data() + base + y_offset, sizeof(float));
      std::memcpy(&z, cloud.data.data() + base + z_offset, sizeof(float));
      if (reflectance_offset >= 0) {
        std::memcpy(&reflectance, cloud.data.data() + base + reflectance_offset, sizeof(float));
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          !std::isfinite(reflectance)) {
        continue;
      }

      // The ROS driver publishes REP-103 X=forward, Y=left, Z=up. Retain an
      // opt-in native conversion for direct SDK/legacy cloud sources.
      const float world_x = input_native_coordinates_ ? z : x;
      const float world_y = input_native_coordinates_ ? -y : y;
      const float world_z = input_native_coordinates_ ? x : z;
      points.insert(points.end(), {world_x, world_y, world_z, reflectance});
      reflectances.push_back(reflectance);
      heights.push_back(world_z);
      farthest = std::max(farthest, std::sqrt(x * x + y * y + z * z));
    }

    const uint32_t count = static_cast<uint32_t>(points.size() / 4);
    if (count == 0) return;
    float reflectance_low = percentile(reflectances, 0.02);
    float reflectance_high = percentile(reflectances, 0.98);
    if (reflectance_high <= reflectance_low) reflectance_high = reflectance_low + 1.0F;
    const float floor_height = percentile(heights, 0.02);
    const uint64_t stamp_ns =
        static_cast<uint64_t>(cloud.header.stamp.sec) * 1000000000ULL +
        cloud.header.stamp.nanosec;

    auto frame = std::make_shared<std::vector<uint8_t>>();
    frame->reserve(kHeaderBytes + points.size() * sizeof(float));
    append(*frame, kFrameMagic);
    append(*frame, count);
    append(*frame, stamp_ns);
    append(*frame, reflectance_low);
    append(*frame, reflectance_high);
    append(*frame, floor_height);
    append(*frame, farthest);
    const auto *point_bytes = reinterpret_cast<const uint8_t *>(points.data());
    frame->insert(frame->end(), point_bytes, point_bytes + points.size() * sizeof(float));
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = std::move(frame);
      ++frame_sequence_;
    }
    frame_cv_.notify_all();
  }

  void serverLoop() {
    try {
      auto address = asio::ip::make_address(bind_address_);
      acceptor_ = std::make_unique<tcp::acceptor>(io_context_, tcp::endpoint(address, port_));
      acceptor_->set_option(asio::socket_base::reuse_address(true));
      acceptor_->non_blocking(true);
      RCLCPP_INFO(get_logger(), "Live viewer: http://%s:%d", bind_address_.c_str(), port_);
      while (!stopping_.load()) {
        beast::error_code error;
        tcp::socket socket(io_context_);
        acceptor_->accept(socket, error);
        if (!error) {
          std::lock_guard<std::mutex> lock(client_threads_mutex_);
          client_threads_.emplace_back(
              [this, socket = std::move(socket)]() mutable { handleClient(std::move(socket)); });
        } else if (error == asio::error::would_block || error == asio::error::try_again) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } else if (!stopping_.load()) {
          RCLCPP_WARN(get_logger(), "Viewer accept error: %s", error.message().c_str());
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Viewer server stopped: %s", error.what());
    }
  }

  void handleClient(tcp::socket socket) {
    try {
      beast::flat_buffer request_buffer;
      http::request<http::string_body> request;
      http::read(socket, request_buffer, request);
      if (websocket::is_upgrade(request) && request.target() == "/stream") {
        websocket::stream<tcp::socket> stream(std::move(socket));
        stream.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        stream.accept(request);
        stream.binary(true);
        uint64_t sent_sequence = 0;
        while (!stopping_.load()) {
          std::shared_ptr<const std::vector<uint8_t>> frame;
          {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait_for(lock, std::chrono::milliseconds(500), [&]() {
              return stopping_.load() || frame_sequence_ != sent_sequence;
            });
            if (stopping_.load()) break;
            if (frame_sequence_ == sent_sequence || !latest_frame_) continue;
            sent_sequence = frame_sequence_;
            frame = latest_frame_;
          }
          beast::error_code error;
          stream.write(asio::buffer(*frame), error);
          if (error) break;
        }
        beast::error_code error;
        stream.close(websocket::close_code::normal, error);
        return;
      }

      http::response<http::string_body> response{http::status::ok, request.version()};
      response.set(http::field::server, "seyond-live-viewer");
      response.set(http::field::content_type, "text/html; charset=utf-8");
      response.set(http::field::cache_control, "no-store");
      response.keep_alive(false);
      response.body() = html_;
      response.prepare_payload();
      http::write(socket, response);
    } catch (const std::exception &) {
      // Browsers disconnect and reconnect routinely; no noisy log is needed.
    }
  }

  int port_{8080};
  int max_points_{80000};
  bool input_native_coordinates_{false};
  std::string bind_address_;
  std::string html_path_;
  std::string html_;
  std::atomic<bool> stopping_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  asio::io_context io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread server_thread_;
  std::mutex client_threads_mutex_;
  std::vector<std::thread> client_threads_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::shared_ptr<const std::vector<uint8_t>> latest_frame_;
  uint64_t frame_sequence_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LiveWebViewer>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("live_web_viewer_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
