#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_bringup/tmini_scan_normalizer.hpp"

class TminiScanNormalizer final : public rclcpp::Node
{
public:
  TminiScanNormalizer()
  : Node("tmini_scan_normalizer")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/scan_raw");
    const auto output = declare_parameter<std::string>("output_topic", "/scan");
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr input_scan) {
        auto output_scan = *input_scan;
        for (auto & range : output_scan.ranges) {
          range = robot_bringup::normalize_tmini_range(
            range, output_scan.range_min, output_scan.range_max);
        }
        publisher_->publish(output_scan);
      });
    RCLCPP_INFO(get_logger(), "T-mini scan normalization %s -> %s", input.c_str(), output.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TminiScanNormalizer>());
  rclcpp::shutdown();
  return 0;
}
