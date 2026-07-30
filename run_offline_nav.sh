#!/bin/bash
# ============================================================================
# run_offline_nav.sh
# 离线2D导航与避障演示脚本
# 
# 功能:
#   1. 加载2D栅格地图（从RTAB-Map数据库提取）
#   2. 在RViz中显示建图轨迹（绿色路径）
#   3. 机器人沿路径运动（坐标在RViz中移动）
#   4. 路径上遇到障碍物时自动避障（Nav2 + MPPI）
#
# 使用方式:
#   bash run_offline_nav.sh [obstacle|clean] [rviz|no_rviz]
#     obstacle : 使用带障碍物的地图（默认，测试避障）
#     clean    : 使用干净地图（无障碍物，纯路径跟踪）
#
# 硬件要求: 无（完全离线，不需要实物机器人/雷达/相机）
# ============================================================================

set -e

MODE="${1:-obstacle}"
RVIZ_MODE="${2:-rviz}"

# 项目路径
PROJECT_DIR="/home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack"
MAP_DIR="${PROJECT_DIR}/scripts/offline_nav_maps"
DB_PATH="${OFFLINE_NAV_DB_PATH:-/data/maps/db/first_version_0514.db}"
PATH_YAML="${MAP_DIR}/path_waypoints.yaml"
OBSTACLES_YAML="${MAP_DIR}/obstacles.yaml"

# 日志目录
LOG_DIR="${PROJECT_DIR}/log/offline_nav"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${LOG_DIR}/ros_${TIMESTAMP}"
mkdir -p "${ROS_LOG_DIR}"

echo "=============================================="
echo "  离线2D导航与避障系统"
echo "  时间: $(date)"
echo "  模式: ${MODE}"
echo "  RViz: ${RVIZ_MODE}"
echo "=============================================="

# ── 1. 环境设置 ──
echo "[1/5] 设置ROS2环境..."
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"

# Python 3.10 兼容性 (ROS2 Humble 需要 Python 3.10)
export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3.10

# ── 2. 检查文件 ──
echo "[2/5] 检查必要文件..."

check_file() {
    if [ -f "$1" ]; then
        echo "  [OK] $2"
    else
        echo "  [MISSING] $2: $1"
        exit 1
    fi
}

check_file "${DB_PATH}" "RTAB-Map数据库"
check_file "${PATH_YAML}" "路径文件"

if [ "${MODE}" = "clean" ]; then
    MAP_YAML="${MAP_DIR}/clean_map.yaml"
    check_file "${MAP_YAML}" "干净地图YAML"
    check_file "${MAP_DIR}/clean_map.pgm" "干净地图PGM"
    SHOW_OBSTACLES="false"
    echo "  [INFO] 使用干净地图（无障碍物）"
else
    MAP_YAML="${MAP_DIR}/obstacle_map.yaml"
    check_file "${MAP_YAML}" "障碍物地图YAML"
    check_file "${MAP_DIR}/obstacle_map.pgm" "障碍物地图PGM"
    check_file "${OBSTACLES_YAML}" "障碍物配置"
    SHOW_OBSTACLES="true"
    echo "  [INFO] 使用障碍物地图（测试避障功能）"
fi

# ── 3. 清理旧进程 ──
echo "[3/5] 清理旧进程..."
pkill -f "ros2 bag" 2>/dev/null || true
pkill -f "fastlio_mapping" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "offline_avoidance.launch.py" 2>/dev/null || true
pkill -f "nav2_map_server" 2>/dev/null || true
pkill -f "nav2_lifecycle_manager" 2>/dev/null || true
pkill -f "nav2_controller" 2>/dev/null || true
pkill -f "nav2_planner" 2>/dev/null || true
pkill -f "nav2_bt_navigator" 2>/dev/null || true
pkill -f "nav2_waypoint_follower" 2>/dev/null || true
pkill -f "nav2_velocity_smoother" 2>/dev/null || true
pkill -f "nav2_smoother" 2>/dev/null || true
pkill -f "nav2_behaviors" 2>/dev/null || true
pkill -f "path_waypoint_sender.py" 2>/dev/null || true
pkill -f "odom_from_cmd_vel.py" 2>/dev/null || true
pkill -f "path_publisher.py" 2>/dev/null || true
pkill -f "clicked_obstacle_publisher.py" 2>/dev/null || true
pkill -f "robot_marker_publisher.py" 2>/dev/null || true
pkill -f "obstacle_marker_publisher.py" 2>/dev/null || true
sleep 2
echo "  [OK] 已清理"

# ── 4. 启动离线导航 ──
echo "[4/5] 启动离线导航系统..."
echo ""
echo "  启动组件:"
echo "    - map_server         : 加载2D栅格地图"
echo "    - path_publisher     : 发布建图轨迹 (/mapping_path)"
echo "    - odom_from_cmd_vel  : 模拟里程计 (cmd_vel→Odometry+TF)"
echo "    - Nav2导航栈         : 全局规划 + MPPI局部避障"
echo "    - path_waypoint_sender: 发送路径点给Nav2"
echo "    - rviz2              : 可视化界面"
echo ""

ros2 launch robot_bringup offline_avoidance.launch.py \
    enable_rviz:=$([ "${RVIZ_MODE}" = "no_rviz" ] && echo false || echo true) \
    map_yaml:="${MAP_YAML}" \
    path_yaml:="${PATH_YAML}" \
    database_path:="${DB_PATH}" \
    obstacles_yaml:="${OBSTACLES_YAML}" \
    show_obstacles:="${SHOW_OBSTACLES}" \
    2>&1 | tee "${LOG_DIR}/offline_nav_${TIMESTAMP}.log"

# ── 5. 清理 ──
echo ""
echo "[5/5] 系统已停止"
echo "  日志: ${LOG_DIR}/offline_nav_${TIMESTAMP}.log"
