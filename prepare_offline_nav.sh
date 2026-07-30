#!/bin/bash
# ============================================================================
# prepare_offline_nav.sh
# Generate all offline Nav2 assets directly from the RTAB-Map database.
# ============================================================================
set -e

export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3.10

PROJECT_DIR="/home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack"
MAP_DIR="${PROJECT_DIR}/scripts/offline_nav_maps"
DB_PATH="${1:-/data/maps/db/first_version_0514.db}"
CORRIDOR_RADIUS="${OFFLINE_NAV_CORRIDOR_RADIUS:-5.0}"
Z_MIN="${OFFLINE_NAV_Z_MIN:--0.3}"
Z_MAX="${OFFLINE_NAV_Z_MAX:-1.5}"

echo "=== 离线导航资产生成 ==="
echo "数据库: ${DB_PATH}"
echo "输出目录: ${MAP_DIR}"
echo "通行区域半径: ${CORRIDOR_RADIUS} m"
echo "点云高度过滤: ${Z_MIN} m <= z <= ${Z_MAX} m"

source /opt/ros/humble/setup.bash

python3.10 "${PROJECT_DIR}/src/robot_bringup/scripts/generate_offline_nav_assets.py" \
    "${DB_PATH}" \
    "${MAP_DIR}" \
    0.05 \
    2.0 \
    "${CORRIDOR_RADIUS}" \
    "${Z_MIN}" \
    "${Z_MAX}"

echo ""
echo "=== 准备完成 ==="
echo "  干净地图:   ${MAP_DIR}/clean_map.yaml"
echo "  障碍物地图: ${MAP_DIR}/obstacle_map.yaml"
echo "  轨迹文件:   ${MAP_DIR}/path_waypoints.yaml"
echo "  障碍物文件: ${MAP_DIR}/obstacles.yaml"
echo ""
echo "运行:"
echo "  bash ${PROJECT_DIR}/run_offline_nav.sh obstacle"
echo "  bash ${PROJECT_DIR}/run_offline_nav.sh clean"
