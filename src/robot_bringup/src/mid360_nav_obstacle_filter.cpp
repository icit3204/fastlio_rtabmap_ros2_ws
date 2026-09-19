#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_bringup/mid360_nav_filter.hpp"

namespace robot_bringup
{

class Mid360NavObstacleFilter : public rclcpp::Node
{
public:
  Mid360NavObstacleFilter()
  : Node("mid360_nav_obstacle_filter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_rtabmap");
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/cloud_registered_nav2_obstacles");
    clearing_topic_ = declare_parameter<std::string>(
      "clearing_topic", "/cloud_registered_nav2_clearing");
    expected_input_frame_ = declare_parameter<std::string>("expected_input_frame", "body");
    filter_frame_ = declare_parameter<std::string>("filter_frame", "base_footprint");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.10);
    bounds_ = {
      declare_parameter<double>("obstacle_min_height", 0.15),
      declare_parameter<double>("obstacle_max_height", 1.60),
      declare_parameter<double>("clearing_min_height", -0.20),
      declare_parameter<double>("clearing_max_height", 2.00),
      declare_parameter<double>("obstacle_min_range", 0.10),
      declare_parameter<double>("obstacle_max_range", 3.00),
      declare_parameter<double>("clearing_min_range", 0.10),
      declare_parameter<double>("clearing_max_range", 3.50)};

    if (input_topic_.empty() || obstacle_topic_.empty() || clearing_topic_.empty() ||
      obstacle_topic_ == clearing_topic_ || expected_input_frame_.empty() ||
      filter_frame_.empty() ||
      !std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ <= 0.0 || !bounds_.valid())
    {
      throw std::runtime_error("Invalid MID-360 Nav2 filter configuration");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, qos);
    clearing_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&Mid360NavObstacleFilter::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "MID-360 Nav2 branch input=%s obstacles=%s clearing=%s filter_frame=%s "
      "obstacle_z=[%.2f,%.2f] obstacle_r=[%.2f,%.2f] clearing_z=[%.2f,%.2f] "
      "clearing_r=[%.2f,%.2f]",
      input_topic_.c_str(), obstacle_topic_.c_str(), clearing_topic_.c_str(), filter_frame_.c_str(),
      bounds_.obstacle_min_height, bounds_.obstacle_max_height,
      bounds_.obstacle_min_range, bounds_.obstacle_max_range,
      bounds_.clearing_min_height, bounds_.clearing_max_height,
      bounds_.clearing_min_range, bounds_.clearing_max_range);
  }

private:
  static bool xyz_fields_are_float32(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    bool x = false;
    bool y = false;
    bool z = false;
    for (const auto & field : cloud.fields) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1) {
        continue;
      }
      x = x || field.name == "x";
      y = y || field.name == "y";
      z = z || field.name == "z";
    }
    return x && y && z;
  }

  static sensor_msgs::msg::PointCloud2 empty_like(const sensor_msgs::msg::PointCloud2 & input)
  {
    sensor_msgs::msg::PointCloud2 output = input;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.data.clear();
    return output;
  }

  static void append_point(
    const sensor_msgs::msg::PointCloud2 & input, std::size_t index,
    sensor_msgs::msg::PointCloud2 & output)
  {
    const std::size_t row = index / input.width;
    const std::size_t column = index % input.width;
    const std::size_t offset = row * input.row_step + column * input.point_step;
    output.data.insert(
      output.data.end(), input.data.begin() + offset,
      input.data.begin() + offset + input.point_step);
    ++output.width;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    // Fail closed only on this Nav2 MID branch. The source FAST-LIO, RTAB and
    // independent safety paths remain untouched if this filter drops input.
    if (cloud->header.frame_id != expected_input_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping MID Nav2 cloud: expected frame '%s', got '%s'",
        expected_input_frame_.c_str(), cloud->header.frame_id.c_str());
      return;
    }
    if (cloud->is_bigendian || cloud->width == 0 || cloud->point_step == 0 ||
      cloud->row_step < cloud->point_step * cloud->width ||
      cloud->data.size() < cloud->row_step * cloud->height || !xyz_fields_are_float32(*cloud))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping malformed MID Nav2 cloud");
      return;
    }

    tf2::Transform filter_from_input;
    if (cloud->header.frame_id == filter_frame_) {
      filter_from_input.setIdentity();
    } else {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          filter_frame_, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
        const auto & t = stamped.transform.translation;
        const auto & q = stamped.transform.rotation;
        filter_from_input = tf2::Transform(
          tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping MID Nav2 cloud: transform %s <- %s unavailable: %s",
          filter_frame_.c_str(), cloud->header.frame_id.c_str(), error.what());
        return;
      }
    }

    auto obstacles = empty_like(*cloud);
    auto clearing = empty_like(*cloud);
    obstacles.data.reserve(cloud->data.size() / 4);
    clearing.data.reserve(cloud->data.size());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    std::size_t index = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      const tf2::Vector3 point = filter_from_input * tf2::Vector3(*iter_x, *iter_y, *iter_z);
      if (bounds_.is_obstacle(point.x(), point.y(), point.z())) {
        append_point(*cloud, index, obstacles);
      }
      if (bounds_.is_clearing_return(point.x(), point.y(), point.z())) {
        append_point(*cloud, index, clearing);
      }
    }

    obstacles.row_step = obstacles.width * obstacles.point_step;
    clearing.row_step = clearing.width * clearing.point_step;
    obstacle_publisher_->publish(obstacles);
    clearing_publisher_->publish(clearing);
  }

  std::string input_topic_;
  std::string obstacle_topic_;
  std::string clearing_topic_;
  std::string expected_input_frame_;
  std::string filter_frame_;
  double tf_timeout_sec_;
  Mid360NavFilterBounds bounds_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace robot_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_bringup::Mid360NavObstacleFilter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mid360_nav_obstacle_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
