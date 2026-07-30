cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# <修改 version2 安装preview_point_publisher脚本>
install(PROGRAMS scripts/preview_point_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/fake_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/scan_to_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
)
# <原版-路径录制和回放脚本>
# <修改 version4 增加离线导航与避障脚本: odom_from_cmd_vel, path_publisher, path_waypoint_sender, add_obstacles_to_map, extract_path_yaml>
install(PROGRAMS scripts/record_path.py scripts/replay_path.py scripts/fake_odom_sim.py scripts/fake_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/odom_from_cmd_vel.py scripts/path_publisher.py scripts/path_waypoint_sender.py scripts/add_obstacles_to_map.py scripts/extract_path_yaml.py scripts/make_offline_nav_maps.py scripts/generate_offline_nav_assets.py scripts/obstacle_marker_publisher.py scripts/robot_marker_publisher.py scripts/clicked_obstacle_publisher.py scripts/plan_nav_laser_avoidance.py scripts/laser_command_safety_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
