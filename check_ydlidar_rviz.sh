#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAMS_FILE="${1:-${WS_DIR}/src/ydlidar_ros2_driver/params/ydlidar.yaml}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  for setup in /opt/ros/*/setup.bash; do
    if [[ -f "${setup}" ]]; then
      # shellcheck source=/dev/null
      set +u
      source "${setup}"
      set -u
      break
    fi
  done
fi

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${WS_DIR}/install/setup.bash"
  set -u
else
  echo "[ERROR] 未找到 ${WS_DIR}/install/setup.bash，请先在工作区执行 colcon build。"
  exit 1
fi

set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 命令不可用，请确认 ROS2 环境已安装。"
  exit 1
fi

if ! ros2 pkg prefix ydlidar_ros2_driver >/dev/null 2>&1; then
  echo "[ERROR] 未找到 ydlidar_ros2_driver 包，请确认已经编译并 source 工作区。"
  exit 1
fi

echo "[INFO] 当前工作区: ${WS_DIR}"
echo "[INFO] 参数文件: ${PARAMS_FILE}"
echo "[INFO] 可疑串口设备:"
shopt -s nullglob
devices=()
[[ -e /dev/ydlidar ]] && devices+=(/dev/ydlidar)
devices+=(/dev/ttyUSB* /dev/ttyACM*)
if (( ${#devices[@]} == 0 )); then
  echo "  未发现 /dev/ydlidar、/dev/ttyUSB* 或 /dev/ttyACM*"
else
  for dev in "${devices[@]}"; do
    ls -l "${dev}"
  done
fi
shopt -u nullglob

cleanup() {
  if [[ -n "${SCAN_CHECK_PID:-}" ]]; then
    kill "${SCAN_CHECK_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[INFO] 启动 YDLidar 驱动和 RViz..."
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py params_file:="${PARAMS_FILE}" &
LAUNCH_PID=$!

(
  sleep 3
  echo "[INFO] 等待 /scan 第一帧，最多 12 秒..."
  if timeout 12 ros2 topic echo --qos-profile sensor_data --once /scan sensor_msgs/msg/LaserScan >/tmp/ydlidar_scan_once.txt 2>/tmp/ydlidar_scan_once.err; then
    echo "[OK] 已收到 /scan 数据，YDLidar 驱动正在发布 LaserScan。"
    echo "[INFO] 可用 ros2 topic hz /scan 查看频率。"
  else
    echo "[WARN] 暂未收到 /scan 数据。请检查 USB 接线、串口权限、参数文件中的 port/baudrate。"
    if [[ -s /tmp/ydlidar_scan_once.err ]]; then
      sed 's/^/[WARN] /' /tmp/ydlidar_scan_once.err
    fi
  fi
) &
SCAN_CHECK_PID=$!

wait "${LAUNCH_PID}"
