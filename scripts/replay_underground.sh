#!/bin/bash
# ============================================================================
# replay_underground.sh
# 回放 underground 数据集: rosbag + FAST-LIO + RTAB-Map + rviz
# 让rviz中机器人坐标沿录制路径运动
# ============================================================================
set -e

LOG_DIR="/home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack/log/task_underground_replay"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="${LOG_DIR}/replay_${TIMESTAMP}.log"

# 错误处理函数
log() {
    echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log_error() {
    echo "[$(date '+%H:%M:%S')] [ERROR] $*" | tee -a "$LOG_FILE"
}

# 开始日志
echo "==============================================" | tee "$LOG_FILE"
log "Underground Dataset Replay Task"
log "Dataset: /home/dog/catkin_byd_1/wheeltec/datasets/lidar_imu_underGround"
log "=============================================="

# 1. Source 环境
log "Step 1: Sourcing ROS2 and workspace..."
source /opt/ros/humble/setup.bash
source /home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack/install/setup.bash

# 检查关键可执行文件
log "Checking executables..."
for exe in ros2 fastlio_mapping rviz2; do
    if command -v $exe &>/dev/null; then
        log "  [OK] $exe"
    else
        log_error "  [MISSING] $exe"
        exit 1
    fi
done

# 验证 bag 文件
BAG_PATH="/home/dog/catkin_byd_1/wheeltec/datasets/lidar_imu_underGround"
if [ -f "${BAG_PATH}/lidar_imu_underGround_0.db3" ]; then
    log "[OK] Bag file found"
else
    log_error "[FATAL] Bag file not found at ${BAG_PATH}"
    exit 1
fi

# 2. 确保数据库目录存在
DB_DIR="/home/dog/catkin_byd_1/wheeltec/datasets/lidar_imu_underGround/rtabmap_db"
mkdir -p "$DB_DIR"
DB_PATH="${DB_DIR}/rtabmap.db"
log "Database path: $DB_PATH"

# 清理旧数据库(全新建图)
rm -f "$DB_PATH"

# 3. 启动 rosbag 播放 (后台, 带 --clock)
log "Step 2: Starting rosbag playback with --clock..."
log "Topics: /livox/lidar, /livox/imu"
ros2 bag play "${BAG_PATH}" --clock -r 1.0 &
BAG_PID=$!
log "rosbag PID: $BAG_PID"

# 等待 clock 发布
sleep 2

# 检查 bag 是否成功启动
if ! kill -0 $BAG_PID 2>/dev/null; then
    log_error "rosbag failed to start!"
    exit 1
fi
log "[OK] rosbag is running"

# 4. 启动 FAST-LIO + RTAB-Map + rviz
log "Step 3: Launching fastlio_mapping (FAST-LIO + RTAB-Map + rviz)..."
log "Parameters: start_livox:=false use_sim_time:=true rviz:=true"

# <修改 version1 去除rtabmap_viz以避免独立GUI干扰>
# <修改 version2 使用本地数据库路径>
ros2 launch robot_bringup fastlio_mapping.launch.py \
    start_livox:=false \
    use_sim_time:=true \
    rviz:=true \
    rtabmap_viz:=false \
    database_path:="$DB_PATH" \
    delete_db_on_start:=true \
    2>&1 | tee -a "$LOG_FILE" &
LAUNCH_PID=$!
log "Launch PID: $LAUNCH_PID"

# 等待启动
log "Waiting for nodes to initialize..."
sleep 5

# 检查关键节点
log "Step 4: Checking nodes..."
ros2 node list 2>&1 | tee -a "$LOG_FILE"

# 检查话题
log "Checking topics..."
ros2 topic list 2>&1 | tee -a "$LOG_FILE"

log "=============================================="
log "Pipeline is running!"
log "Bag PID: $BAG_PID"
log "Launch PID: $LAUNCH_PID"
log "Log file: $LOG_FILE"
log ""
log "rviz should show:"
log "  - TF tree: map -> odom -> base_footprint -> base_link -> livox_frame"
log "  - Lidar cloud: /cloud_registered_body"
log "  - Odometry arrow: /Odometry (robot trajectory)"
log "  - Map: /rtabmap/map"
log ""
log "Press Ctrl+C to stop all processes"
log "=============================================="

# 5. 等待 bag 播放完成或用户中断
cleanup() {
    log "Cleaning up..."
    kill $BAG_PID 2>/dev/null || true
    kill $LAUNCH_PID 2>/dev/null || true
    wait $BAG_PID 2>/dev/null || true
    wait $LAUNCH_PID 2>/dev/null || true
    log "All processes stopped."
    log "Final log saved to: $LOG_FILE"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待 bag 进程结束
wait $BAG_PID 2>/dev/null || true
log "Bag playback finished."

# Bag 播放完后继续运行一段时间让 RTAB-Map 完成优化
log "Waiting 10s for RTAB-Map to finalize..."
sleep 10

# 清理
cleanup
