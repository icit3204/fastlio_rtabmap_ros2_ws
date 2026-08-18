cmake_minimum_required(VERSION 3.8)
project(parking_robot_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mission_manager_progress_checker SHARED
  src/mission_manager_progress_checker.cpp
)
target_compile_features(mission_manager_progress_checker PUBLIC cxx_std_17)
target_include_directories(mission_manager_progress_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_manager_progress_checker
  geometry_msgs
  nav2_core
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  nav2_core mission_manager_progress_checker_plugin.xml
)

install(TARGETS mission_manager_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest(test_mission_manager_progress_checker
    test/test_mission_manager_progress_checker.cpp
  )
  if(TARGET test_mission_manager_progress_checker)
    ament_target_dependencies(test_mission_manager_progress_checker
      geometry_msgs
      nav2_core
      pluginlib
      rclcpp
      rclcpp_lifecycle
    )
  endif()
endif()

ament_export_dependencies(geometry_msgs nav2_core pluginlib rclcpp_lifecycle)
ament_export_include_directories(include)
ament_export_libraries(mission_manager_progress_checker)
ament_package()
