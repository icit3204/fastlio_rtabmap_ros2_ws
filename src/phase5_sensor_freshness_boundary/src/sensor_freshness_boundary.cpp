#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "phase5_sensor_freshness_boundary/boundary_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
double stamp_sec(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}

class SensorFreshnessBoundary final : public rclcpp::Node {
public:
  SensorFreshnessBoundary() : Node("phase5_sensor_freshness_boundary") {
    const bool enabled = declare_parameter<bool>("startup_freshness_boundary_enabled", true);
    const double lidar_age = declare_parameter<double>("startup_lidar_max_age_sec", 0.50);
    const double imu_age = declare_parameter<double>("startup_imu_max_age_sec", 0.05);
    const double stable_window = declare_parameter<double>("startup_stable_window_sec", 2.0);
    lidar_input_ = declare_parameter<std::string>("lidar_input_topic", "/livox/lidar");
    imu_input_ = declare_parameter<std::string>("imu_input_topic", "/livox/imu");
    lidar_output_ = declare_parameter<std::string>("lidar_output_topic", "/phase5/livox/lidar_fresh");
    imu_output_ = declare_parameter<std::string>("imu_output_topic", "/phase5/livox/imu_fresh");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/phase5/livox/freshness_status");
    core_ = std::make_unique<phase5_sensor_freshness_boundary::BoundaryCore>(
      phase5_sensor_freshness_boundary::BoundaryConfig{enabled, lidar_age, imu_age, stable_window});

    // Livox ROS 2 uses the default reliable/volatile/keep-last QoS with its
    // bounded queue. Match that contract at the boundary input.
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();
    const auto lidar_output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
    const auto imu_output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_output_, lidar_output_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_, imu_output_qos);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>(diagnostics_topic_, rclcpp::QoS(10));
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_input_, input_qos, std::bind(&SensorFreshnessBoundary::lidar_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_, input_qos, std::bind(&SensorFreshnessBoundary::imu_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SensorFreshnessBoundary::publish_diagnostics, this));
    RCLCPP_INFO(get_logger(), "Phase-5 sensor freshness boundary %s enabled=%s lidar_age=%.3f imu_age=%.3f stable_window=%.3f",
      enabled ? "CLOSED" : "OPEN_BYPASS", enabled ? "true" : "false", lidar_age, imu_age, stable_window);
  }

private:
  void lidar_callback(livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
    const double now = get_clock()->now().seconds();
    const double header = stamp_sec(msg->header.stamp);
    const auto max_offset = std::max_element(msg->points.begin(), msg->points.end(),
      [](const auto &a, const auto &b) { return a.offset_time < b.offset_time; });
    const double effective_end = header + (max_offset == msg->points.end() ? 0.0 : static_cast<double>(max_offset->offset_time) * 1e-9);
    const double header_age = now - header;
    const double effective_age = now - effective_end;
    const bool finite = std::isfinite(header) && std::isfinite(header_age) && std::isfinite(effective_age);
    const bool advancing = !have_lidar_ || header > last_lidar_stamp_;
    last_lidar_stamp_ = header;
    have_lidar_ = true;
    const auto decision = core_->observe_lidar(now, header_age, effective_age, finite, advancing);
    if (decision == phase5_sensor_freshness_boundary::Decision::FORWARD_OPEN) lidar_pub_->publish(std::move(msg));
    if (!was_open_ && core_->open()) {
      was_open_ = true;
      RCLCPP_INFO(get_logger(), "Phase-5 sensor freshness boundary OPEN at ROS time %.9f", core_->open_time_sec());
    }
  }

  void imu_callback(sensor_msgs::msg::Imu::UniquePtr msg) {
    const double now = get_clock()->now().seconds();
    const double header = stamp_sec(msg->header.stamp);
    const double age = now - header;
    const bool finite = std::isfinite(header) && std::isfinite(age);
    const bool advancing = !have_imu_ || header > last_imu_stamp_;
    last_imu_stamp_ = header;
    have_imu_ = true;
    const auto decision = core_->observe_imu(now, age, finite, advancing);
    if (decision == phase5_sensor_freshness_boundary::Decision::FORWARD_OPEN) imu_pub_->publish(std::move(msg));
    if (!was_open_ && core_->open()) {
      was_open_ = true;
      RCLCPP_INFO(get_logger(), "Phase-5 sensor freshness boundary OPEN at ROS time %.9f", core_->open_time_sec());
    }
  }

  void publish_diagnostics() {
    const auto &c = core_->counters();
    const double now = get_clock()->now().seconds();
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"state\":\"" << (core_->open() ? "OPEN" : "CLOSED")
        << "\",\"closed_lidar_dropped\":" << c.closed_lidar_dropped
        << ",\"closed_imu_dropped\":" << c.closed_imu_dropped
        << ",\"closed_lidar_stale\":" << c.closed_lidar_stale
        << ",\"closed_imu_stale\":" << c.closed_imu_stale
        << ",\"stable_window_starts\":" << c.stable_window_starts
        << ",\"stable_window_resets\":" << c.stable_window_resets
        << ",\"lidar_fresh_now\":" << (core_->lidar_fresh_now(now) ? "true" : "false")
        << ",\"imu_fresh_now\":" << (core_->imu_fresh_now(now) ? "true" : "false")
        << ",\"lidar_valid_until_sec\":" << core_->lidar_valid_until_sec()
        << ",\"imu_valid_until_sec\":" << core_->imu_valid_until_sec()
        << ",\"open_time_sec\":" << core_->open_time_sec()
        << ",\"open_lidar_received\":" << c.open_lidar_received
        << ",\"open_imu_received\":" << c.open_imu_received
        << ",\"open_lidar_forwarded\":" << c.open_lidar_forwarded
        << ",\"open_imu_forwarded\":" << c.open_imu_forwarded << "}";
    msg.data = out.str();
    diagnostics_pub_->publish(msg);
  }

  std::unique_ptr<phase5_sensor_freshness_boundary::BoundaryCore> core_;
  std::string lidar_input_, imu_input_, lidar_output_, imu_output_, diagnostics_topic_;
  bool have_lidar_{false}, have_imu_{false}, was_open_{false};
  double last_lidar_stamp_{0.0}, last_imu_stamp_{0.0};
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFreshnessBoundary>());
  rclcpp::shutdown();
  return 0;
}
