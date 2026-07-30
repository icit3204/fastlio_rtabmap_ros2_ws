cmake_minimum_required(VERSION 3.8)
project(rtsp_camera_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rtsp_camera_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
