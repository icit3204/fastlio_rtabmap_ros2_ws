#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "wheelchair_controller/mkmini_backend_core.hpp"

class WheelchairController : public rclcpp::Node
{
public:
  WheelchairController()
  : Node("wheelchair_controller_node"), control_paused_(true), running_(true)
  {
    declare_parameters();
    output_transport_ = get_parameter("output_transport").as_string();
    min_command_interval_ms_ = get_parameter("min_command_interval_ms").as_double();
    control_paused_ = !get_parameter("auto_start").as_bool();

    wheelchair_controller::BackendConfig config;
    config.can_id = static_cast<uint32_t>(get_parameter("can_frame_id").as_int());
    config.velocity_limit_mmps = get_parameter("can_velocity_limit").as_int();
    config.straight_radius_threshold_mm =
      get_parameter("can_straight_radius_threshold_mm").as_double();
    config.minimum_turn_radius_mm = get_parameter("can_min_turn_radius_mm").as_double();
    config.invert_radius = get_parameter("can_invert_radius").as_bool();
    config.wheelbase_mm = get_parameter("wheelbase_mm").as_double();
    config.maximum_steering_deg = get_parameter("max_steer_angle_deg").as_double();
    config.command_timeout_sec = get_parameter("command_timeout_ms").as_double() / 1000.0;
    core_ = std::make_unique<wheelchair_controller::MkminiBackendCore>(config);

    if (output_transport_ == "can") {
      init_can();
    } else if (output_transport_ == "udp") {
      init_udp();
    } else if (output_transport_ == "mock") {
      mock_transport_ = std::make_unique<wheelchair_controller::MockTransport>();
      mock_raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/phase5/mk2e4/mock_can_frame", 100);
      mock_decoded_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/phase5/mk2e4/mock_can_decoded", 100);
      RCLCPP_INFO(get_logger(), "MockTransport enabled: SocketCAN will not be opened");
    } else {
      throw std::runtime_error("output_transport must be can, udp, or mock");
    }

    command_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wheelchair_control_command", 10,
      std::bind(&WheelchairController::command_callback, this, std::placeholders::_1));

    const auto period_ms = std::max<int64_t>(
      1, std::llround(get_parameter("can_send_period_ms").as_double()));
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&WheelchairController::heartbeat_callback, this));

    if (isatty(STDIN_FILENO) && output_transport_ != "mock") {
      keyboard_thread_ = std::thread(&WheelchairController::keyboard_listener, this);
    }
    RCLCPP_INFO(
      get_logger(), "MK-mini labmate backend ready: transport=%s paused=%s period=%ldms",
      output_transport_.c_str(), control_paused_ ? "true" : "false", period_ms);
  }

  ~WheelchairController() override
  {
    running_ = false;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    if (can_sockfd_ >= 0) {
      close(can_sockfd_);
    }
    if (udp_sockfd_ >= 0) {
      close(udp_sockfd_);
    }
  }

private:
  static double monotonic_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void declare_parameters()
  {
    declare_parameter<std::string>("target_ip", "10.42.0.1");
    declare_parameter<int>("target_port", 9999);
    declare_parameter<bool>("auto_start", false);
    declare_parameter<double>("min_command_interval_ms", 20.0);
    declare_parameter<std::string>("output_transport", "can");
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<int>("can_frame_id", 0x18C4D2D0);
    declare_parameter<int>("can_velocity_limit", 16380);
    declare_parameter<double>("can_wheel_half_track_mm", 300.0);
    declare_parameter<double>("can_straight_radius_threshold_mm", 10000.0);
    declare_parameter<double>("can_min_turn_radius_mm", 1000.0);
    declare_parameter<bool>("can_invert_radius", true);
    declare_parameter<double>("can_send_period_ms", 20.0);
    declare_parameter<double>("command_timeout_ms", 500.0);
    declare_parameter<bool>("can_use_command_distance", false);
    declare_parameter<int>("can_default_distance", 0);
    declare_parameter<double>("wheelbase_mm", 1000.0);
    declare_parameter<double>("max_steer_angle_deg", 30.0);
  }

  void init_can()
  {
    can_interface_ = get_parameter("can_interface").as_string();
    can_sockfd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_sockfd_ < 0) {
      throw std::runtime_error("SocketCAN socket creation failed");
    }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_sockfd_, SIOCGIFINDEX, &ifr) < 0) {
      close(can_sockfd_);
      can_sockfd_ = -1;
      throw std::runtime_error("SocketCAN interface lookup failed");
    }
    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_sockfd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      close(can_sockfd_);
      can_sockfd_ = -1;
      throw std::runtime_error("SocketCAN bind failed");
    }
  }

  void init_udp()
  {
    udp_sockfd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (udp_sockfd_ < 0) {
      throw std::runtime_error("UDP socket creation failed");
    }
    std::memset(&udp_address_, 0, sizeof(udp_address_));
    udp_address_.sin_family = AF_INET;
    udp_address_.sin_port = htons(get_parameter("target_port").as_int());
    if (inet_pton(
        AF_INET, get_parameter("target_ip").as_string().c_str(), &udp_address_.sin_addr) <= 0)
    {
      throw std::runtime_error("invalid UDP target address");
    }
  }

  void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (has_callback_time_ &&
      std::chrono::duration<double, std::milli>(now - last_callback_time_).count() <
      min_command_interval_ms_)
    {
      return;
    }
    last_callback_time_ = now;
    has_callback_time_ = true;
    if (message->data.size() < 3U ||
      !std::isfinite(message->data[0]) || !std::isfinite(message->data[1]) ||
      !std::isfinite(message->data[2]))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid lower-backend command");
      return;
    }
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_->accept_command(
      message->data[0], message->data[1], message->data[2], monotonic_seconds());

    if (output_transport_ == "udp" && !control_paused_) {
      send_udp(message->data[0], message->data[1]);
    }
  }

  void heartbeat_callback()
  {
    if (output_transport_ == "udp") {
      return;
    }
    wheelchair_controller::EncodedFrame frame;
    {
      std::lock_guard<std::mutex> lock(core_mutex_);
      frame = core_->tick(monotonic_seconds(), control_paused_);
    }
    if (output_transport_ == "mock") {
      mock_transport_->emit(frame);
      publish_mock(frame);
    } else {
      send_can(frame);
    }
  }

  void send_can(const wheelchair_controller::EncodedFrame & encoded)
  {
    if (can_sockfd_ < 0) {
      return;
    }
    struct can_frame frame {};
    frame.can_id = encoded.can_id | CAN_EFF_FLAG;
    frame.can_dlc = encoded.dlc;
    std::copy(encoded.data.begin(), encoded.data.end(), frame.data);
    if (write(can_sockfd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "SocketCAN write failed");
    }
  }

  void publish_mock(const wheelchair_controller::EncodedFrame & frame)
  {
    std_msgs::msg::UInt8MultiArray raw;
    raw.data.assign(frame.data.begin(), frame.data.end());
    mock_raw_pub_->publish(raw);
    std_msgs::msg::Float32MultiArray decoded;
    decoded.data = {
      static_cast<float>(frame.gear), static_cast<float>(frame.speed_mmps),
      static_cast<float>(frame.steering_deg), static_cast<float>(frame.alive),
      frame.stale_fallback ? 1.0F : 0.0F,
      wheelchair_controller::checksum_valid(frame) ? 1.0F : 0.0F,
      wheelchair_controller::reserved_bits_zero(frame) ? 1.0F : 0.0F};
    mock_decoded_pub_->publish(decoded);
  }

  void send_udp(double radius, double velocity)
  {
    if (udp_sockfd_ < 0) {
      return;
    }
    char buffer[16];
    const double values[2] = {radius, velocity};
    for (int i = 0; i < 2; ++i) {
      uint64_t host_value;
      std::memcpy(&host_value, &values[i], sizeof(double));
      const uint64_t net_value = htobe64(host_value);
      std::memcpy(buffer + i * 8, &net_value, sizeof(uint64_t));
    }
    sendto(
      udp_sockfd_, buffer, sizeof(buffer), 0,
      reinterpret_cast<const struct sockaddr *>(&udp_address_), sizeof(udp_address_));
  }

  void keyboard_listener()
  {
    struct termios previous {}, raw {};
    tcgetattr(STDIN_FILENO, &previous);
    raw = previous;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    const int previous_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, previous_flags | O_NONBLOCK);
    while (running_ && rclcpp::ok()) {
      char key;
      if (read(STDIN_FILENO, &key, 1) > 0 && key == ' ') {
        control_paused_ = !control_paused_;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &previous);
    fcntl(STDIN_FILENO, F_SETFL, previous_flags);
  }

  std::string output_transport_;
  double min_command_interval_ms_{20.0};
  std::unique_ptr<wheelchair_controller::MkminiBackendCore> core_;
  std::unique_ptr<wheelchair_controller::MockTransport> mock_transport_;
  std::mutex core_mutex_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr mock_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mock_decoded_pub_;

  int can_sockfd_{-1};
  std::string can_interface_{"can0"};
  int udp_sockfd_{-1};
  struct sockaddr_in udp_address_ {};

  std::atomic<bool> control_paused_;
  std::atomic<bool> running_;
  std::thread keyboard_thread_;
  std::chrono::steady_clock::time_point last_callback_time_;
  bool has_callback_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelchairController>());
  rclcpp::shutdown();
  return 0;
}
