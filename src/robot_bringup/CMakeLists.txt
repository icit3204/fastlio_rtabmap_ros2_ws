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
install(PROGRAMS scripts/mk2c_costmap_observer.py scripts/mk2c_floor_return_observer.py scripts/mk2c1r_reconciliation_observer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/mk2f1_mppi_stationary_runner.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/mk2f2_planner_stationary_runner.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/mk2f3_navigate_to_pose_stationary_runner.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/mk2f4_mission_manager_stationary_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_mk2c_static_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2c_static_contract.py)
  ament_add_test(test_mk2c_retired_detector_guard
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2c_retired_detector_guard.py)
  ament_add_test(test_mk2e4_labmate_mock_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2e4_labmate_mock_contract.py)
  ament_add_test(test_mk2e5_canonical_backend_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2e5_canonical_backend_contract.py)
  ament_add_test(test_mk2f1_mppi_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2f1_mppi_contract.py)
  ament_add_test(test_mk2f2_planner_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2f2_planner_contract.py)
  ament_add_test(test_mk2f2a_map_runtime_hygiene_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2f2a_map_runtime_hygiene_contract.py)
  ament_add_test(test_mk2f3_bt_navigator_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2f3_bt_navigator_contract.py)
  ament_add_test(test_mk2f4_mission_manager_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mk2f4_mission_manager_contract.py)
  set_tests_properties(
    test_mk2c_static_contract
    test_mk2c_retired_detector_guard
    test_mk2e4_labmate_mock_contract
    test_mk2e5_canonical_backend_contract
    test_mk2f1_mppi_contract
    test_mk2f2_planner_contract
    test_mk2f2a_map_runtime_hygiene_contract
    test_mk2f3_bt_navigator_contract
    test_mk2f4_mission_manager_contract
    PROPERTIES ENVIRONMENT "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()
