#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        // 声明并获取参数
        this->declare_parameter("lookahead_distance", 1.8);  // 前瞻距离
        this->declare_parameter("avoidance_lookahead_distance", 0.8);  // 避障时使用更短前瞻，避免切过临时外偏点
        this->declare_parameter("min_turning_radius", 1.5);   // 最小转弯半径
        this->declare_parameter("linear_velocity", 3.0);      // 巡航速度
        this->declare_parameter("goal_tolerance", 0.5);       // 到达终点的误差容限
        this->declare_parameter("goal_yaw_tolerance", 0.1); // 到达终点的朝向误差容限(弧度)，约5.7度
        this->declare_parameter("rotate_in_place_speed", 1.6); // 原地调整朝向的速度
        this->declare_parameter("stop_on_position_reached", true); // 到达终点距离容差后直接停止，不再原地调朝向
        this->declare_parameter("path_topic_name", "/plan_nav");     // 订阅路径话题名称
        this->declare_parameter("control_topic_name", "/wheelchair_control_command"); // 发布控制指令话题名称
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        avoidance_lookahead_distance_ = this->get_parameter("avoidance_lookahead_distance").as_double();
        min_turning_radius_ = this->get_parameter("min_turning_radius").as_double();
        linear_velocity_ = this->get_parameter("linear_velocity").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        goal_yaw_tolerance_ = this->get_parameter("goal_yaw_tolerance").as_double();
        rotate_in_place_speed_ = this->get_parameter("rotate_in_place_speed").as_double();
        stop_on_position_reached_ = this->get_parameter("stop_on_position_reached").as_bool();

        path_topic_name_ = this->get_parameter("path_topic_name").as_string();
        control_topic_name_ = this->get_parameter("control_topic_name").as_string();
        // 订阅话题
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_name_, 10, std::bind(&PurePursuitController::PathCallback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/baselink2map", 10, std::bind(&PurePursuitController::OdomCallback, this, std::placeholders::_1));

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PurePursuitController::CmdVelCallback, this, std::placeholders::_1));
        sub_laser_avoidance_state_ = this->create_subscription<std_msgs::msg::String>(
            "/laser_avoidance_state", 10, std::bind(&PurePursuitController::LaserAvoidanceStateCallback, this, std::placeholders::_1));

        // 发布话题
        pub_control_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(control_topic_name_, 10);
        pub_lookahead_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_point", 10);
        pub_path_future_ = this->create_publisher<nav_msgs::msg::Path>("/path_future", 10);

        // 初始化TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 定时器：持续发布控制指令（即使路径没有更新）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PurePursuitController::ControlTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "PurePursuitController initialized");
    }

private:
    void PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_ = *msg;
        path_updated_ = true;
        goal_reached_latched_ = false;
        // 重置当前索引，因为路径可能更新了
        // current_path_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", path_.poses.size());
        RCLCPP_INFO(this->get_logger(), "current_path_idx_ is %d", current_path_idx_);
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_ = *msg;
    }

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 这里可以根据需要处理外部速度命令，例如限制最大速度等
        cmd_vel_linear_velocity_ = msg->linear.x;
        cmd_vel_angular_velocity_ = msg->angular.z;
    }

    void LaserAvoidanceStateCallback(const std_msgs::msg::String::SharedPtr msg) {
        laser_avoidance_active_ = msg->data.rfind("AVOIDING_", 0) == 0;
    }

    void ControlTimerCallback() {
        // 获取路径和位姿的副本
        nav_msgs::msg::Path path;
        nav_msgs::msg::Odometry odom;

        bool path_just_updated = false; // 用于记录路径是否刚更新
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (path_.poses.empty()) {
                return; // 无路径，不发布控制指令
            }
            path = path_;
            // 安全读取并重置更新标志
            if (path_updated_) {
                path_just_updated = true;
                path_updated_ = false;
            }

        }
        if (goal_reached_latched_) {
            PublishStopCommand();
            PublishFuturePath(path, current_path_idx_);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom = odom_;
        }

        // 获取当前机器人位姿（在map坐标系下）
        Eigen::Matrix4d baselink2map = Eigen::Matrix4d::Identity();
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map", "base_link", odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
            baselink2map = Eigen::Matrix4d::Identity();
            Eigen::Vector3d translation(transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z);
            Eigen::Quaterniond rotation(transform.transform.rotation.w,
                                        transform.transform.rotation.x,
                                        transform.transform.rotation.y,
                                        transform.transform.rotation.z);
            baselink2map.block<3, 3>(0, 0) = rotation.matrix();
            baselink2map.block<3, 1>(0, 3) = translation;
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", e.what());
            return;
        }

        // 当前机器人位置（map坐标系下）
        Eigen::Vector3d current_pos = baselink2map.block<3, 1>(0, 3);
        // 当前机器人的朝向（map坐标系下，取X轴方向）
        Eigen::Quaterniond current_rot(baselink2map.block<3, 3>(0, 0));
        Eigen::Vector3d current_dir_3d = current_rot * Eigen::Vector3d::UnitX();
        Eigen::Vector2d current_dir_xy(current_dir_3d.x(), current_dir_3d.y());
        if (current_dir_xy.norm() > 1e-3) current_dir_xy.normalize();

        // 1. 在路径上寻找最近点（从当前索引开始搜索，避免走回头路）
        // 安全检查索引范围 + 搜索策略
        int search_start = current_path_idx_;
        int search_end = static_cast<int>(path.poses.size());
        
        // 如果路径刚更新，或者当前索引已经越界，必须进行全局重置/搜索
        if (path_just_updated || current_path_idx_ >= static_cast<int>(path.poses.size())) {
            // 策略：新路径到来时，为了安全，从索引0开始找最近点（或者进行全局搜索）
            search_start = 0; 
            RCLCPP_INFO(this->get_logger(), "Path updated or index out of range, resetting search to start.");
        } else {
            // 正常追踪时，限制搜索范围（向后看5个点，向前看100个点）
            search_start = std::max(0, current_path_idx_ - 5);
            search_end = std::min(static_cast<int>(path.poses.size()), current_path_idx_ + 100);
        }

        double min_dist = std::numeric_limits<double>::max();
        for (int i = search_start; i < search_end; ++i) {
            const auto &pose = path.poses[i].pose;
            double dx = pose.position.x - current_pos.x();
            double dy = pose.position.y - current_pos.y();
            double dz = pose.position.z - current_pos.z();
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < min_dist) {
                min_dist = dist;
                current_path_idx_ = i;
            }
        }

        // 2. 从当前最近点开始，寻找前瞻点（距离当前机器人位置至少为前瞻距离的点）
        int lookahead_idx = current_path_idx_;
        bool found_lookahead = false;
        double active_lookahead_distance = laser_avoidance_active_ ? avoidance_lookahead_distance_ : lookahead_distance_;
        for (int i = current_path_idx_; i < static_cast<int>(path.poses.size()); ++i) {
            const auto &pose = path.poses[i].pose;
            double dx = pose.position.x - current_pos.x();
            double dy = pose.position.y - current_pos.y();
            double dz = pose.position.z - current_pos.z();
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist >= active_lookahead_distance) {
                lookahead_idx = i;
                found_lookahead = true;
                break;
            }
        }
        if (!found_lookahead) {
            // 如果没有找到足够远的点，就使用最后一个点
            lookahead_idx = path.poses.size() - 1;
        }

        // 3. 终点判断（同时考虑位置和朝向）
        double dist_to_end = 0.0;
        double yaw_error = 0.0;
        {
            const auto &last_pose = path.poses.back().pose;
            double dx = last_pose.position.x - current_pos.x();
            double dy = last_pose.position.y - current_pos.y();
            double dz = last_pose.position.z - current_pos.z();
            dist_to_end = std::sqrt(dx*dx + dy*dy + dz*dz);

            // 计算目标朝向的偏航角
            tf2::Quaternion target_q(
                last_pose.orientation.x, last_pose.orientation.y,
                last_pose.orientation.z, last_pose.orientation.w);
            double roll, pitch, target_yaw;
            tf2::Matrix3x3(target_q).getRPY(roll, pitch, target_yaw);

            // 计算当前机器人朝向的偏航角
            Eigen::Vector3d current_dir_3d = current_rot * Eigen::Vector3d::UnitX();
            double current_yaw = std::atan2(current_dir_3d.y(), current_dir_3d.x());

            // 计算朝向误差并规范化到 [-PI, PI]
            yaw_error = target_yaw - current_yaw;
            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
        }

        // 如果接近终点位置，则进行朝向调整或停止
        if (dist_to_end < goal_tolerance_) {
            if (stop_on_position_reached_) {
                goal_reached_latched_ = true;
                PublishStopCommand();
                is_stopped_ = true;
                PublishFuturePath(path, current_path_idx_);
                return;
            } else if (std::abs(yaw_error) > goal_yaw_tolerance_) {
                // ===== 位置到达，但朝向未到达：原地转向调整 =====
                std_msgs::msg::Float32MultiArray align_msg;
                align_msg.data.resize(3);
                // 转弯半径设为0表示原地转向；右转为正，左转为负
                align_msg.data[0] = 0.0; 
                // 根据偏航误差决定转向方向：yaw_error > 0 表示需要左转(给负速度)，反之右转(给正速度)
                align_msg.data[1] = (yaw_error > 0) ? -rotate_in_place_speed_ * 1000.0 
                                                    : rotate_in_place_speed_ * 1000.0;
                align_msg.data[2] = 0.0;
                pub_control_->publish(align_msg);
                
                // 此时还未完全停止
                is_stopped_ = true; 

                // 发布未来轨迹（保持显示）
                PublishFuturePath(path, current_path_idx_);
                return;
            } else {
                // ===== 位置和朝向均到达：完全停止 =====
                goal_reached_latched_ = true;
                PublishStopCommand();
                is_stopped_ = true;

                // 发布删除指令清除残留的 Marker
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = "map";
                delete_marker.header.stamp = this->now();
                delete_marker.ns = "lookahead_point";
                delete_marker.id = 0;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                pub_lookahead_->publish(delete_marker);

                PublishFuturePath(path, current_path_idx_);
                return;
            }
        }

        // 4. 计算控制指令
        // 将前瞻点从map坐标系转换到base_link坐标系
        geometry_msgs::msg::PoseStamped lookahead_map;
        lookahead_map.header = path.header;
        lookahead_map.header.stamp.sec = 0;
        lookahead_map.header.stamp.nanosec = 0;
        lookahead_map.pose = path.poses[lookahead_idx].pose;
        geometry_msgs::msg::PoseStamped lookahead_base;
        try {
            lookahead_base = tf_buffer_->transform(lookahead_map, "base_link", tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Could not transform lookahead point: %s", e.what());
            return;
        }

        double dx = lookahead_base.pose.position.x; // 前向距离
        double dy = lookahead_base.pose.position.y; // 横向距离（左正右负）
        double dist_to_lookahead = std::sqrt(dx*dx + dy*dy);
        double angle_to_target = std::atan2(dy, dx); 
        double angle_go_forward_min = 5;
        double angle_go_forward_max = laser_avoidance_active_ ? 8 : 30;
        // 计算转弯半径 R = (L^2) / (2*dy)  其中 dy>0 左转，dy<0 右转
        // 注意：如果 dy 接近0，则转弯半径很大（近似直行）
        double R = 10.0; // 默认大转弯半径，近似直行
        double base_vel = linear_velocity_;
        is_stopped_ = false;
        // if (fabs(cmd_vel_linear_velocity_) <1e-6 && fabs(cmd_vel_angular_velocity_) < 1e-6 ) { // 如果外部速度命令为0，则停止 可能到了禁止区域 但是掉头速度也为0
        //     R = 0.0; 
        //     base_vel = 0.0;
        //     is_stopped_ = true;
        // }else 
        if (dx < 0 || fabs(angle_to_target)>angle_go_forward_max * M_PI / 180.0 ) {
            // 根据横向偏移决定原地转向方向：左转给-速度，右转给+速度
            R = 0.0; 
            base_vel = (dy > 0)? -rotate_in_place_speed_ : rotate_in_place_speed_; // 不前进，只转向
        }else if (angle_to_target > angle_go_forward_min * M_PI / 180.0) {
            // dy > 0 时，算出的 R 必然为正
            if (std::abs(dy) > 0.01) {
                R = (dx*dx + dy*dy) / (2.0 * dy); 
            } else {
                R = min_turning_radius_;
            }
            // 限制最小转弯半径 (MapDrawer 逻辑)
            if (R > 0 && R < min_turning_radius_) {
                R = min_turning_radius_;
            }
            base_vel = 2.0;
        }
        // 情况 C：目标点在前方，且偏右 (超出死区，约 < -5度)
        else if (angle_to_target < -1*angle_go_forward_min * M_PI / 180.0) {
            // dy < 0 时，算出的 R 必然为负
            if (std::abs(dy) > 0.01) {
                R = (dx*dx + dy*dy) / (2.0 * dy); 
            } else {
                R = -min_turning_radius_;
            }
            // 限制最小转弯半径 (MapDrawer 逻辑)
            if (R < 0 && R > -min_turning_radius_) {
                R = -min_turning_radius_;
            }
            base_vel = 2.0;
        }
        // 情况 D：目标点在正前方死区内 (约 ± angle_go_forward_min度以内)
        else {
            R = 10.0; // 大转弯半径，近似直行
            base_vel = 5.0; // 直行时设定一个较高的速度
        }


        // 转换为底层协议：右转为正，左转为负
        // 数学推导：左转(dy>0)算出R>0，乘-1变负；右转(dy<0)算出R<0，乘-1变正
        if (R != 10.0 && R != 0.0) {
            R *= -1;
        }

        // 发布控制指令
        std_msgs::msg::Float32MultiArray cmd_msg;
        cmd_msg.data.resize(3);
        cmd_msg.data[0] = R * 1000.0;  // 转弯半径转换为毫米
        cmd_msg.data[1] = base_vel * 1000.0; // 速度转换为毫米/秒
        cmd_msg.data[2] = dist_to_lookahead * 1000.0; // 前瞻距离转换为毫米
        pub_control_->publish(cmd_msg);

        // 发布前瞻点
        if(!is_stopped_){
            PublishLookaheadMarker(path.poses[lookahead_idx].pose, path.header.frame_id, 0.5, 1.0, 0.0, 0.0); // 红色
        }

        // 发布未来轨迹（从当前索引到终点）
        PublishFuturePath(path, current_path_idx_);
    }

    void PublishLookaheadMarker(const geometry_msgs::msg::Pose &pose, const std::string &frame_id, double scale, double r, double g, double b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id; // 使用传入的坐标系
        marker.header.stamp = this->now();
        marker.ns = "lookahead_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        pub_lookahead_->publish(marker);
    }

    void PublishStopCommand() {
        std_msgs::msg::Float32MultiArray stop_msg;
        stop_msg.data.resize(3);
        stop_msg.data[0] = 0.0;
        stop_msg.data[1] = 0.0;
        stop_msg.data[2] = 0.0;
        pub_control_->publish(stop_msg);
    }

    void PublishFuturePath(const nav_msgs::msg::Path &path, int start_idx) {
        nav_msgs::msg::Path future_path;
        future_path.header = path.header;
        future_path.header.stamp = this->now();
        for (int i = start_idx; i < static_cast<int>(path.poses.size()); ++i) {
            future_path.poses.push_back(path.poses[i]);
        }
        pub_path_future_->publish(future_path);
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_laser_avoidance_state_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_control_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lookahead_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_future_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;
    std::mutex path_mutex_;
    std::mutex odom_mutex_;

    int current_path_idx_ = 0;
    bool path_updated_ = false;
    bool laser_avoidance_active_ = false;

    // 参数
    double lookahead_distance_;
    double avoidance_lookahead_distance_;
    double min_turning_radius_;
    double linear_velocity_;
    double cmd_vel_linear_velocity_;
    double cmd_vel_angular_velocity_;
    double goal_tolerance_;
    double goal_yaw_tolerance_;
    double rotate_in_place_speed_;
    bool stop_on_position_reached_;
    
    std::string path_topic_name_;
    std::string control_topic_name_;
    bool is_stopped_ = false;
    bool goal_reached_latched_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
