#include "parking_robot_nav2_plugins/mission_manager_progress_checker.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace parking_robot_nav2_plugins
{

void MissionManagerProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & /* parent */,
  const std::string & /* plugin_name */)
{
}

bool MissionManagerProgressChecker::check(
  geometry_msgs::msg::PoseStamped & /* current_pose */)
{
  return true;
}

void MissionManagerProgressChecker::reset()
{
}

}  // namespace parking_robot_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  parking_robot_nav2_plugins::MissionManagerProgressChecker,
  nav2_core::ProgressChecker)
