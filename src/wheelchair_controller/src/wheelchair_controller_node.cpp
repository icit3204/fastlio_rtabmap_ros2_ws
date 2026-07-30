#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

class WheelchairController : public rclcpp::Node {
public:
    WheelchairController() : Node("wheelchair_controller"), control_paused_(true), running_(true) {
        this->declare_parameter<std::string>("target_ip", "10.42.0.1");
        this->declare_parameter<int>("target_port", 9999);
        this->declare_parameter<bool>("auto_start", false);
        this->declare_parameter<double>("min_command_interval_ms", 20.0);
        this->declare_parameter<std::string>("output_transport", "can");
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("can_frame_id", 0x801400);
        this->declare_parameter<int>("can_velocity_limit", 16380);
        this->declare_parameter<double>("can_wheel_half_track_mm", 300.0);
        this->declare_parameter<double>("can_straight_radius_threshold_mm", 10000.0);
        this->declare_parameter<double>("can_min_turn_radius_mm", 1000.0);
        this->declare_parameter<bool>("can_invert_radius", true);
        this->declare_parameter<double>("can_send_period_ms", 20.0);
        this->declare_parameter<double>("command_timeout_ms", 500.0);
        this->declare_parameter<bool>("can_use_command_distance", false);
        this->declare_parameter<int>("can_default_distance", 0);

        output_transport_ = this->get_parameter("output_transport").as_string();
        min_interval_ms_ = this->get_parameter("min_command_interval_ms").as_double();
        command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_double();

        if (this->get_parameter("auto_start").as_bool()) {
            control_paused_ = false;
        }

        if (output_transport_ == "udp") {
            InitUdp();
        } else if (output_transport_ == "can") {
            InitCan();
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unsupported output_transport '%s', expected 'can' or 'udp'",
                         output_transport_.c_str());
            throw std::runtime_error("unsupported output_transport");
        }

        sub_command_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheelchair_control_command", 10,
            std::bind(&WheelchairController::CommandCallback, this, std::placeholders::_1));

        auto can_period = std::chrono::milliseconds(
            std::max<int64_t>(1, static_cast<int64_t>(std::llround(this->get_parameter("can_send_period_ms").as_double()))));
        can_timer_ = this->create_wall_timer(can_period, std::bind(&WheelchairController::CanTimerCallback, this));

        if (isatty(STDIN_FILENO)) {
            keyboard_thread_ = std::thread(&WheelchairController::KeyboardListener, this);
        } else {
            RCLCPP_WARN(this->get_logger(), "Not running in a terminal, keyboard pause disabled.");
        }

        RCLCPP_INFO(this->get_logger(), "Wheelchair Controller initialized. output_transport=%s",
                    output_transport_.c_str());
        RCLCPP_INFO(this->get_logger(), "Press SPACE in this terminal to START/PAUSE control.");
        if (control_paused_) {
            RCLCPP_WARN(this->get_logger(), "*** CONTROL IS CURRENTLY PAUSED ***");
        }
    }

    ~WheelchairController() {
        running_ = false;
        if (udp_sockfd_ >= 0) {
            close(udp_sockfd_);
        }
        if (can_sockfd_ >= 0) {
            SendCanFrame(0.0, 0.0, 0.0);
            close(can_sockfd_);
        }
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    void InitUdp() {
        const std::string target_ip = this->get_parameter("target_ip").as_string();
        const int target_port = this->get_parameter("target_port").as_int();

        udp_sockfd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (udp_sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed");
            return;
        }

        memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_port = htons(target_port);
        if (inet_pton(AF_INET, target_ip.c_str(), &servaddr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", target_ip.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "UDP output enabled: %s:%d", target_ip.c_str(), target_port);
    }

    void InitCan() {
        can_interface_ = this->get_parameter("can_interface").as_string();
        can_frame_id_ = static_cast<canid_t>(this->get_parameter("can_frame_id").as_int());
        can_velocity_limit_ = this->get_parameter("can_velocity_limit").as_int();
        can_wheel_half_track_mm_ = this->get_parameter("can_wheel_half_track_mm").as_double();
        can_straight_radius_threshold_mm_ = this->get_parameter("can_straight_radius_threshold_mm").as_double();
        can_min_turn_radius_mm_ = this->get_parameter("can_min_turn_radius_mm").as_double();
        can_invert_radius_ = this->get_parameter("can_invert_radius").as_bool();
        can_use_command_distance_ = this->get_parameter("can_use_command_distance").as_bool();
        can_default_distance_ = this->get_parameter("can_default_distance").as_int();

        can_sockfd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
        if (can_sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "SocketCAN socket creation failed for %s", can_interface_.c_str());
            return;
        }

        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_sockfd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot find CAN interface %s", can_interface_.c_str());
            close(can_sockfd_);
            can_sockfd_ = -1;
            return;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SocketCAN bind failed on %s", can_interface_.c_str());
            close(can_sockfd_);
            can_sockfd_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "CAN output enabled: interface=%s frame_id=0x%X velocity_limit=%d",
                    can_interface_.c_str(), can_frame_id_, can_velocity_limit_);
    }

    void CommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(now - last_send_time_).count();
        if (elapsed_ms < min_interval_ms_) {
            RCLCPP_DEBUG(this->get_logger(), "Throttling: ignore command");
            return;
        }
        last_send_time_ = now;
        if (msg->data.size() < 3) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received invalid command array size < 3");
            return;
        }

        if (control_paused_) {
            SendControlMsg(0.0, 0.0, 0.0);
            return;
        }

        double radius = static_cast<double>(msg->data[0]);
        double velocity = static_cast<double>(msg->data[1]);
        double distance = static_cast<double>(msg->data[2]);

        SendControlMsg(radius, velocity, distance);
    }

    void SendControlMsg(double radius, double velocity, double distance) {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            last_radius_ = radius;
            last_velocity_ = velocity;
            last_distance_ = distance;
            last_command_time_ = std::chrono::steady_clock::now();
            has_command_ = true;
            sent_timeout_stop_ = false;
        }

        if (output_transport_ == "udp") {
            SendUdpMsg(radius, velocity);
        } else if (output_transport_ == "can") {
            SendCanFrame(radius, velocity, distance);
        }
    }

    void SendUdpMsg(double radius, double velocity) {
        if (udp_sockfd_ < 0) {
            return;
        }

        char buffer[16];
        double data[2] = {radius, velocity};

        for (int i = 0; i < 2; ++i) {
            uint64_t host_value;
            memcpy(&host_value, &data[i], sizeof(double));
            uint64_t net_value = htobe64(host_value);
            memcpy(buffer + i * 8, &net_value, sizeof(uint64_t));
        }

        ssize_t send_bytes = sendto(udp_sockfd_, buffer, sizeof(buffer), 0,
                                    (const struct sockaddr*)&servaddr_, sizeof(servaddr_));

        if (send_bytes == -1) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "sendto failed");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Sent UDP -> R: %.2f mm, V: %.2f mm/s", radius, velocity);
        }
    }

    void CanTimerCallback() {
        if (output_transport_ != "can" || can_sockfd_ < 0) {
            return;
        }

        double radius = 0.0;
        double velocity = 0.0;
        double distance = 0.0;
        bool should_send = false;

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!has_command_) {
                return;
            }

            const auto elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - last_command_time_).count();
            if (elapsed_ms <= command_timeout_ms_ && !control_paused_) {
                radius = last_radius_;
                velocity = last_velocity_;
                distance = last_distance_;
                should_send = true;
            } else if (!sent_timeout_stop_) {
                sent_timeout_stop_ = true;
                should_send = true;
            } else {
                return;
            }
        }

        if (should_send) {
            SendCanFrame(radius, velocity, distance);
        }
    }

    void SendCanFrame(double radius, double velocity, double distance) {
        if (can_sockfd_ < 0) {
            return;
        }

        int vel = ClampToInt16(static_cast<int>(std::llround(velocity)), can_velocity_limit_);
        const double normalized_radius = NormalizeCanRadius(radius);
        const int radius_int = static_cast<int>(std::llround(normalized_radius));
        int v_left = 0;
        int v_right = 0;

        if (vel == 0) {
            v_left = 0;
            v_right = 0;
        } else if (radius_int == 0) {
            v_left = vel;
            v_right = -vel;
        } else if (normalized_radius >= can_straight_radius_threshold_mm_) {
            v_left = -vel;
            v_right = -vel;
        } else {
            v_left = -static_cast<int>(((normalized_radius + can_wheel_half_track_mm_) * vel / normalized_radius) + 0.5);
            v_right = -static_cast<int>(((normalized_radius - can_wheel_half_track_mm_) * vel / normalized_radius) + 0.5);
        }

        v_left = ClampToInt16(v_left, can_velocity_limit_);
        v_right = ClampToInt16(v_right, can_velocity_limit_);

        const int distance_value = can_use_command_distance_
            ? ClampToInt16(static_cast<int>(std::llround(distance)), 32767)
            : ClampToInt16(can_default_distance_, 32767);

        struct can_frame frame {};
        frame.can_id = can_frame_id_ | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        PutInt16LE(frame.data, 0, static_cast<int16_t>(v_left));
        PutInt16LE(frame.data, 2, static_cast<int16_t>(v_right));
        PutInt16LE(frame.data, 4, static_cast<int16_t>(distance_value));
        PutInt16LE(frame.data, 6, static_cast<int16_t>(distance_value));

        const ssize_t nbytes = write(can_sockfd_, &frame, sizeof(frame));
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "CAN send failed on %s", can_interface_.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                         "Sent CAN id=0x%X R=%.1f normalized_R=%.1f V=%.1f -> left=%d right=%d dis=%d",
                         can_frame_id_, radius, normalized_radius, velocity, v_left, v_right, distance_value);
        }
    }

    double NormalizeCanRadius(double radius) const {
        if (std::abs(radius) < 1e-6 || radius >= can_straight_radius_threshold_mm_) {
            return radius;
        }

        double normalized = can_invert_radius_ ? -radius : radius;
        if (normalized > 0.0 && normalized < can_min_turn_radius_mm_) {
            normalized = can_min_turn_radius_mm_;
        } else if (normalized < 0.0 && normalized > -can_min_turn_radius_mm_) {
            normalized = -can_min_turn_radius_mm_;
        }
        return normalized;
    }

    static int ClampToInt16(int value, int abs_limit) {
        const int limit = std::min(std::abs(abs_limit), 32767);
        return std::max(-limit, std::min(limit, value));
    }

    static void PutInt16LE(unsigned char *data, int offset, int16_t value) {
        const uint16_t u = static_cast<uint16_t>(value);
        data[offset] = static_cast<unsigned char>(u & 0xFF);
        data[offset + 1] = static_cast<unsigned char>((u >> 8) & 0xFF);
    }

    void KeyboardListener() {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

        while (running_ && rclcpp::ok()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                if (c == ' ') {
                    control_paused_ = !control_paused_;
                    if (control_paused_) {
                        RCLCPP_INFO(this->get_logger(), "\n*** CONTROL PAUSED *** (Sending STOP command)");
                        SendControlMsg(0.0, 0.0, 0.0);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "\n*** CONTROL RESUMED ***");
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }

    int udp_sockfd_ = -1;
    struct sockaddr_in servaddr_;

    int can_sockfd_ = -1;
    std::string can_interface_ = "can0";
    canid_t can_frame_id_ = 0x801400;
    int can_velocity_limit_ = 16380;
    double can_wheel_half_track_mm_ = 300.0;
    double can_straight_radius_threshold_mm_ = 10000.0;
    double can_min_turn_radius_mm_ = 1000.0;
    bool can_invert_radius_ = true;
    bool can_use_command_distance_ = false;
    int can_default_distance_ = 0;

    std::string output_transport_ = "can";
    std::atomic<bool> control_paused_;
    std::atomic<bool> running_;
    std::thread keyboard_thread_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_command_;
    rclcpp::TimerBase::SharedPtr can_timer_;

    std::chrono::steady_clock::time_point last_send_time_;
    double min_interval_ms_;
    double command_timeout_ms_ = 500.0;

    std::mutex command_mutex_;
    double last_radius_ = 0.0;
    double last_velocity_ = 0.0;
    double last_distance_ = 0.0;
    std::chrono::steady_clock::time_point last_command_time_;
    bool has_command_ = false;
    bool sent_timeout_stop_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelchairController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
