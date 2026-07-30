#!/bin/bash
# 编译 rtsp_camera_bridge 包
# 在 rtabmap_nav2_stack 工作空间下执行 colcon build

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$WS_ROOT"

echo "Workspace: $WS_ROOT"
echo "Building rtsp_camera_bridge ..."

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rtsp_camera_bridge

echo "Build done. Source: source $WS_ROOT/install/setup.bash"
