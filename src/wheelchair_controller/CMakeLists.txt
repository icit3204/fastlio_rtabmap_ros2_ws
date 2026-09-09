cmake_minimum_required(VERSION 3.8)
project(wheelchair_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(mkmini_backend_core src/mkmini_backend_core.cpp)
target_include_directories(mkmini_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(wheelchair_controller_node src/wheelchair_controller_node.cpp)
ament_target_dependencies(wheelchair_controller_node rclcpp std_msgs)
target_link_libraries(wheelchair_controller_node mkmini_backend_core)
target_include_directories(wheelchair_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  tf2_eigen
)
target_include_directories(pure_pursuit_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ---------------- Install --------------- #
install(TARGETS wheelchair_controller_node pure_pursuit_controller_node mkmini_backend_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_mkmini_backend_core test/test_mkmini_backend_core.cpp)
  target_link_libraries(test_mkmini_backend_core mkmini_backend_core)
  add_executable(mk2e4_matrix_dump test/mk2e4_matrix_dump.cpp)
  target_link_libraries(mk2e4_matrix_dump mkmini_backend_core)
endif()

ament_package()
