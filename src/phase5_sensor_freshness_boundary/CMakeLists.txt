cmake_minimum_required(VERSION 3.8)
project(phase5_sensor_freshness_boundary)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wformat=2 -Werror=format)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(sensor_freshness_boundary_core INTERFACE)
target_include_directories(sensor_freshness_boundary_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(sensor_freshness_boundary src/sensor_freshness_boundary.cpp)
target_link_libraries(sensor_freshness_boundary sensor_freshness_boundary_core)
ament_target_dependencies(sensor_freshness_boundary rclcpp livox_ros_driver2 sensor_msgs std_msgs)

install(TARGETS sensor_freshness_boundary DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(sensor_freshness_boundary_core_test test/sensor_freshness_boundary_core_test.cpp)
  target_link_libraries(sensor_freshness_boundary_core_test sensor_freshness_boundary_core)
  ament_target_dependencies(sensor_freshness_boundary_core_test rclcpp livox_ros_driver2 sensor_msgs)
endif()

ament_package()
