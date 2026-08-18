#ifndef PARKING_ROBOT_NAV2_PLUGINS__MISSION_MANAGER_PROGRESS_CHECKER_HPP_
#define PARKING_ROBOT_NAV2_PLUGINS__MISSION_MANAGER_PROGRESS_CHECKER_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace parking_robot_nav2_plugins
{

class MissionManagerProgressChecker final : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  void reset() override;
};

}  // namespace parking_robot_nav2_plugins

#endif  // PARKING_ROBOT_NAV2_PLUGINS__MISSION_MANAGER_PROGRESS_CHECKER_HPP_
