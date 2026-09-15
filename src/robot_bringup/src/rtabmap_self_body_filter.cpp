#include <algorithm>
#include <chrono>
#include <cstring>
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

#include "robot_bringup/asymmetric_self_crop.hpp"

using namespace std::chrono_literals;

namespace robot_bringup
{

class RtabmapSelfBodyFilter : public rclcpp::Node
{
public:
  RtabmapSelfBodyFilter()
  : Node("rtabmap_self_body_filter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_registered_rtabmap");
    expected_input_frame_ = declare_parameter<std::string>("expected_input_frame", "body");
    crop_frame_ = declare_parameter<std::string>("crop_frame", "base_footprint");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.10);
    bounds_ = {
      declare_parameter<double>("x_min", 0.0),
      declare_parameter<double>("x_max", 0.0),
      declare_parameter<double>("y_min", 0.0),
      declare_parameter<double>("y_max", 0.0),
      declare_parameter<double>("z_min", 0.0),
      declare_parameter<double>("z_max", 0.0)};

    if (!bounds_.valid() || input_topic_.empty() || output_topic_.empty() ||
      expected_input_frame_.empty() || crop_frame_.empty() ||
      !std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("Invalid RTAB self-body filter configuration");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&RtabmapSelfBodyFilter::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "TEMPORARY_MKMINI_SELF_BODY_CROP input=%s output=%s crop_frame=%s "
      "x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f]",
      input_topic_.c_str(), output_topic_.c_str(), crop_frame_.c_str(),
      bounds_.x_min, bounds_.x_max, bounds_.y_min, bounds_.y_max,
      bounds_.z_min, bounds_.z_max);
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

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    // Fail closed for the RTAB branch. Raw FAST-LIO and safety topics are not
    // republished or modified by this node.
    if (cloud->header.frame_id != expected_input_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping RTAB cloud: expected frame '%s', got '%s'",
        expected_input_frame_.c_str(), cloud->header.frame_id.c_str());
      return;
    }
    if (cloud->is_bigendian || cloud->point_step == 0 ||
      cloud->row_step < cloud->point_step * cloud->width ||
      cloud->data.size() < cloud->row_step * cloud->height ||
      !xyz_fields_are_float32(*cloud))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping malformed RTAB cloud");
      return;
    }

    tf2::Transform crop_from_input;
    if (cloud->header.frame_id == crop_frame_) {
      crop_from_input.setIdentity();
    } else {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          crop_frame_, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
        const auto & t = stamped.transform.translation;
        const auto & q = stamped.transform.rotation;
        crop_from_input = tf2::Transform(
          tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping RTAB cloud: transform %s <- %s unavailable: %s",
          crop_frame_.c_str(), cloud->header.frame_id.c_str(), error.what());
        return;
      }
    }

    sensor_msgs::msg::PointCloud2 output = *cloud;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.data.clear();
    output.data.reserve(cloud->data.size());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    std::size_t input_index = 0;
    std::size_t removed = 0;
    const std::size_t total = static_cast<std::size_t>(cloud->width) * cloud->height;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++input_index) {
      const tf2::Vector3 point_crop = crop_from_input * tf2::Vector3(*iter_x, *iter_y, *iter_z);
      if (bounds_.contains(point_crop.x(), point_crop.y(), point_crop.z())) {
        ++removed;
        continue;
      }
      const std::size_t row = input_index / cloud->width;
      const std::size_t column = input_index % cloud->width;
      const std::size_t offset = row * cloud->row_step + column * cloud->point_step;
      output.data.insert(
        output.data.end(), cloud->data.begin() + offset,
        cloud->data.begin() + offset + cloud->point_step);
      ++output.width;
    }

    output.row_step = output.width * output.point_step;
    publisher_->publish(output);
    RCLCPP_DEBUG(
      get_logger(), "RTAB self crop retained=%zu removed=%zu total=%zu",
      total - removed, removed, total);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string expected_input_frame_;
  std::string crop_frame_;
  double tf_timeout_sec_;
  CropBounds bounds_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace robot_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_bringup::RtabmapSelfBodyFilter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("rtabmap_self_body_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
