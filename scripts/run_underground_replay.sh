#!/bin/bash
# ============================================================================
# 回放 underground 数据集，让rviz中机器人坐标沿路径运动
# 流程: rosbag play + FAST-LIO + RTAB-Map + rviz
# 日志: log/task_underground_replay/
# ============================================================================

LOG_DIR="/home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack/log/task_underground_replay"
TS=$(date +%Y%m%d_%H%M%S)
MAIN_LOG="${LOG_DIR}/full_${TS}.log"

exec > >(tee -a "$MAIN_LOG") 2>&1

echo "===== START $(date) ====="

# Source
source /opt/ros/humble/setup.bash
source /home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack/install/setup.bash

# Prepare
BAG_DIR="/home/dog/catkin_byd_1/wheeltec/datasets/lidar_imu_underGround"
DB_DIR="$BAG_DIR/rtabmap_db"
mkdir -p "$DB_DIR"
rm -f "$DB_DIR/rtabmap.db"

# Clean
pkill -f "ros2 bag" 2>/dev/null || true
pkill -f "fastlio_mapping" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# ── Start bag (background, stdin from /dev/null to disable keyboard) ──
echo "=== Starting rosbag at $(date) ==="
ros2 bag play "$BAG_DIR" --clock -r 2.0 < /dev/null &
BAG_PID=$!
echo "Bag PID=$BAG_PID"

# Wait for bag to start publishing
sleep 10

# ── Start FAST-LIO + RTAB-Map + rviz ──
echo "=== Starting fastlio_mapping at $(date) ==="
ros2 launch robot_bringup fastlio_mapping.launch.py \
    start_livox:=false \
    use_sim_time:=true \
    rviz:=true \
    rtabmap_viz:=false \
    database_path:="$DB_DIR/rtabmap.db" \
    delete_db_on_start:=true \
    < /dev/null &
LAUNCH_PID=$!
echo "Launch PID=$LAUNCH_PID"

echo "===== PIPELINE STARTED $(date) ====="
echo "Bag PID=$BAG_PID, Launch PID=$LAUNCH_PID"
echo "Bag duration: ~7 min (2x speed of 856s recording)"
echo "Log: $MAIN_LOG"

# ── Wait for bag to finish ──
wait $BAG_PID 2>/dev/null || true
echo "=== Bag finished at $(date) ==="

# Let RTAB-Map finalize
sleep 30
echo "=== Finalizing at $(date) ==="

# Cleanup
kill $LAUNCH_PID 2>/dev/null || true
sleep 3

echo "===== DONE $(date) ====="
echo "Log: $MAIN_LOG"
