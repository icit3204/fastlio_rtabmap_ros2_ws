#pragma once

#include <cstdint>
#include <cstring>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace phase5_sensor_freshness_boundary {

inline uint32_t float_bits(float value) { uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); return bits; }
inline uint64_t double_bits(double value) { uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); return bits; }

inline bool custom_msg_semantically_equal(
  const livox_ros_driver2::msg::CustomMsg &a,
  const livox_ros_driver2::msg::CustomMsg &b) {
  if (a.header.stamp.sec != b.header.stamp.sec || a.header.stamp.nanosec != b.header.stamp.nanosec ||
      a.header.frame_id != b.header.frame_id || a.timebase != b.timebase || a.point_num != b.point_num ||
      a.lidar_id != b.lidar_id || a.rsvd != b.rsvd || a.points.size() != b.points.size()) return false;
  for (size_t i = 0; i < a.points.size(); ++i) {
    const auto &x = a.points[i]; const auto &y = b.points[i];
    if (x.offset_time != y.offset_time || float_bits(x.x) != float_bits(y.x) ||
        float_bits(x.y) != float_bits(y.y) || float_bits(x.z) != float_bits(y.z) ||
        x.reflectivity != y.reflectivity || x.tag != y.tag || x.line != y.line) return false;
  }
  return true;
}

inline bool imu_semantically_equal(const sensor_msgs::msg::Imu &a, const sensor_msgs::msg::Imu &b) {
  if (a.header.stamp.sec != b.header.stamp.sec || a.header.stamp.nanosec != b.header.stamp.nanosec ||
      a.header.frame_id != b.header.frame_id) return false;
  if (double_bits(a.orientation.x) != double_bits(b.orientation.x) || double_bits(a.orientation.y) != double_bits(b.orientation.y) ||
      double_bits(a.orientation.z) != double_bits(b.orientation.z) || double_bits(a.orientation.w) != double_bits(b.orientation.w)) return false;
  for (size_t i = 0; i < 9; ++i) if (double_bits(a.orientation_covariance[i]) != double_bits(b.orientation_covariance[i])) return false;
  if (double_bits(a.angular_velocity.x) != double_bits(b.angular_velocity.x) || double_bits(a.angular_velocity.y) != double_bits(b.angular_velocity.y) ||
      double_bits(a.angular_velocity.z) != double_bits(b.angular_velocity.z)) return false;
  for (size_t i = 0; i < 9; ++i) if (double_bits(a.angular_velocity_covariance[i]) != double_bits(b.angular_velocity_covariance[i])) return false;
  if (double_bits(a.linear_acceleration.x) != double_bits(b.linear_acceleration.x) || double_bits(a.linear_acceleration.y) != double_bits(b.linear_acceleration.y) ||
      double_bits(a.linear_acceleration.z) != double_bits(b.linear_acceleration.z)) return false;
  for (size_t i = 0; i < 9; ++i) if (double_bits(a.linear_acceleration_covariance[i]) != double_bits(b.linear_acceleration_covariance[i])) return false;
  return true;
}

}  // namespace phase5_sensor_freshness_boundary
