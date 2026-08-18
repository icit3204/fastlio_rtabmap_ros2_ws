#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

constexpr char kPluginType[] =
  "parking_robot_nav2_plugins::MissionManagerProgressChecker";

class MissionManagerProgressCheckerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  nav2_core::ProgressChecker::Ptr load()
  {
    auto checker = loader_.createSharedInstance(kPluginType);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_checker_test");
    checker->initialize(node_, "mission_manager_progress_checker");
    return checker;
  }

  pluginlib::ClassLoader<nav2_core::ProgressChecker> loader_{
    "nav2_core", "nav2_core::ProgressChecker"};
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

TEST_F(MissionManagerProgressCheckerTest, PluginlibLoadsOnlyRegisteredClass)
{
  auto checker = load();
  ASSERT_NE(checker, nullptr);
  EXPECT_THROW(
    loader_.createSharedInstance(
      "parking_robot_nav2_plugins::MissingProgressChecker"),
    pluginlib::PluginlibException);
  EXPECT_THROW(
    loader_.createSharedInstance(
      "nav2_controller::DefinitelyNotAProgressChecker"),
    pluginlib::PluginlibException);
}

TEST_F(MissionManagerProgressCheckerTest, AlwaysValidForArbitraryPoses)
{
  auto checker = load();
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(checker->check(pose));

  pose.pose.position.x = 1.0;
  pose.pose.position.y = -2.0;
  pose.pose.orientation.z = 0.7071067811865475;
  pose.pose.orientation.w = 0.7071067811865476;
  EXPECT_TRUE(checker->check(pose));

  pose.pose.position.x = 1.0e12;
  pose.pose.position.y = -1.0e12;
  pose.pose.position.z = std::numeric_limits<double>::max();
  EXPECT_TRUE(checker->check(pose));
  EXPECT_TRUE(checker->check(pose));
}

TEST_F(MissionManagerProgressCheckerTest, ResetIsRepeatableAndHasNoGoalState)
{
  auto checker = load();
  geometry_msgs::msg::PoseStamped pose;
  for (int goal = 0; goal < 100; ++goal) {
    checker->reset();
    pose.pose.position.x = static_cast<double>(goal);
    EXPECT_TRUE(checker->check(pose));
    checker->reset();
    EXPECT_TRUE(checker->check(pose));
  }
}

}  // namespace
