#!/bin/bash
# 启动 RTSP 相机桥接节点
# 用法：可编辑下方 RTSP_URL 后执行；或通过环境变量覆盖：
#   RTSP_URL='rtsp://...' ./start_rtsp_bridge.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 默认 RTSP 地址（密码中的 @ 需写成 %40）
RTSP_URL="${RTSP_URL:-rtsp://admin:Basic%402021@192.168.168.25:554/cam/realmonitor?channel=1&subtype=0}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/sensors/camera/rgb/image_rect}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/sensors/camera/rgb/camera_info}"
FRAME_ID="${FRAME_ID:-camera_link}"

source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"

echo "Starting rtsp_camera_bridge ..."
echo "  rtsp_url=$RTSP_URL"
echo "  image_topic=$IMAGE_TOPIC"
echo "  camera_info_topic=$CAMERA_INFO_TOPIC"

ros2 launch rtsp_camera_bridge rtsp_camera_bridge.launch.py \
  rtsp_url:="$RTSP_URL" \
  image_topic:="$IMAGE_TOPIC" \
  camera_info_topic:="$CAMERA_INFO_TOPIC" \
  frame_id:="$FRAME_ID"
