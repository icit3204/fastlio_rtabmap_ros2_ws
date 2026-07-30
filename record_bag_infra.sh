#!/bin/sh
# ============================================================
# ROS2 Bag 录制脚本：Livox MID360 + RealSense D435i (兼容 sh/dash)
# ============================================================
set -e

# ── 配置区 ──
BAG_NAME="mapping_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="./bags/${BAG_NAME}"
SENSOR_PROFILE="lidar_stereo"
START_LIVOX="true"
START_REALSENSE="true"
RECORD_LIMIT_SEC=0
TOPIC_WAIT_SEC=20

# ── 颜色输出 ──
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_green() { printf "${GREEN}%b${NC}\n" "$1"; }
log_yellow() { printf "${YELLOW}%b${NC}\n" "$1"; }
log_red() { printf "${RED}%b${NC}\n" "$1"; }

log_green "========================================"
log_green " ROS2 Bag 录制工具"
log_green "========================================"
log_yellow " Bag 名称 : ${BAG_NAME}"
log_yellow " 存储路径 : ${BAG_DIR}"
log_yellow " 传感器模式 : ${SENSOR_PROFILE}"
log_yellow " 启动 Livox : ${START_LIVOX}"
log_yellow " 启动相机 : ${START_REALSENSE}"
log_yellow " 话题等待 : ${TOPIC_WAIT_SEC}s"
echo ""

# ── 创建目录 ──
mkdir -p "$(dirname "${BAG_DIR}")"

# ============================================================
# Step 1: 启动传感器驱动（后台）
# ============================================================
log_green "[Step 1] 启动传感器驱动..."

LAUNCH_PID=""
BAG_PID=""

cleanup() {
    echo ""
    log_yellow "[清理] 正在停止所有进程..."
    # 先停录制
    if [ -n "$BAG_PID" ]; then
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
    # 再停驱动
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    log_green "[完成] Bag 已保存到: ${BAG_DIR}"
    exit 0
}

trap cleanup INT TERM

launch_running() {
    [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null
}

topic_exists() {
    ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

wait_for_topic() {
    topic="$1"
    waited=0
    while [ "$waited" -lt "$TOPIC_WAIT_SEC" ]; do
        if ! launch_running; then
            return 2
        fi
        if topic_exists "$topic"; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

ros2 launch robot_bringup record_bag_infra.launch.py \
    sensor_profile:=${SENSOR_PROFILE} \
    start_livox:=${START_LIVOX} \
    start_realsense:=${START_REALSENSE} \
    &
LAUNCH_PID=$!

# 等待传感器就绪
log_yellow "[等待] 传感器初始化中 (5秒)..."
sleep 5

if ! launch_running; then
    log_red "[错误] 传感器 launch 已退出，话题检查无效。请先查看上方 launch 报错。"
    wait "$LAUNCH_PID" 2>/dev/null || true
    exit 1
fi

# ============================================================
# Step 2: 检查话题是否就绪
# ============================================================
log_green "[Step 2] 检查话题..."

TOPICS_OK=true
CHECK_TOPICS=""

if [ "${START_LIVOX}" = "true" ]; then
    if wait_for_topic "/livox/lidar"; then
        log_green " ✓ /livox/lidar"
    else
        log_red " ✗ /livox/lidar 未找到！"
        TOPICS_OK=false
    fi
    if wait_for_topic "/livox/imu"; then
        log_green " ✓ /livox/imu"
    else
        log_red " ✗ /livox/imu 未找到！"
        TOPICS_OK=false
    fi
fi

if [ "${START_REALSENSE}" = "true" ]; then
    case ${SENSOR_PROFILE} in
        lidar_stereo)
            CHECK_TOPICS="/camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/infra1/camera_info /camera/infra2/camera_info /camera/depth/image_rect_raw"
            ;;
        lidar_rgbd)
            CHECK_TOPICS="/camera/infra1/image_rect_raw /camera/depth/image_rect_raw /camera/infra1/camera_info"
            ;;
        lidar_mono)
            CHECK_TOPICS="/camera/infra1/image_rect_raw /camera/infra1/camera_info"
            ;;
        lidar_only)
            CHECK_TOPICS=""
            ;;
    esac

    # 遍历空格分隔的字符串 (不加引号，使其按空格拆分)
    for topic in $CHECK_TOPICS; do
        if wait_for_topic "${topic}"; then
            log_green " ✓ ${topic}"
        else
            log_red " ✗ ${topic} 未找到！"
            TOPICS_OK=false
        fi
    done
fi

if [ "${TOPICS_OK}" = "false" ]; then
    log_red "[错误] 部分话题未就绪，请检查传感器连接！"
    log_yellow "当前已发现话题："
    ros2 topic list 2>/dev/null || true
    log_yellow "继续录制？(y/n)"
    read -r answer
    if [ "${answer}" != "y" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        exit 1
    fi
fi

# ============================================================
# Step 3: 构建录制话题列表
# ============================================================
log_green "[Step 3] 构建话题列表并开始录制..."

# 使用字符串代替数组，各项用空格分隔
RECORD_TOPICS="/tf /tf_static"

if [ "${START_LIVOX}" = "true" ]; then
    RECORD_TOPICS="${RECORD_TOPICS} /livox/lidar /livox/imu"
fi

if [ "${START_REALSENSE}" = "true" ]; then
    case ${SENSOR_PROFILE} in
        lidar_stereo)
            RECORD_TOPICS="${RECORD_TOPICS} /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/infra1/camera_info /camera/infra2/camera_info /camera/depth/image_rect_raw"
            ;;
        lidar_rgbd)
            RECORD_TOPICS="${RECORD_TOPICS} /camera/infra1/image_rect_raw /camera/depth/image_rect_raw /camera/infra1/camera_info"
            ;;
        lidar_mono)
            RECORD_TOPICS="${RECORD_TOPICS} /camera/infra1/image_rect_raw /camera/infra1/camera_info"
            ;;
    esac
fi

log_yellow "录制话题列表："
# 遍历字符串
for topic in $RECORD_TOPICS; do
    printf " 📦 %s\n" "${topic}"
done
echo ""

# ============================================================
# Step 4: 开始录制
# ============================================================
log_green "[Step 4] 开始录制..."
log_yellow "按 Ctrl+C 停止录制"
echo ""

# 直接拼接命令字符串（因为 ROS 话题名不含空格，这样是安全的）
RECORD_CMD="ros2 bag record -o ${BAG_DIR} ${RECORD_TOPICS}"

if [ "${RECORD_LIMIT_SEC}" -gt 0 ]; then
    RECORD_CMD="${RECORD_CMD} --duration ${RECORD_LIMIT_SEC}"
fi

# 执行命令
${RECORD_CMD} &
BAG_PID=$!

wait "$BAG_PID" 2>/dev/null || true

log_yellow "[清理] 停止传感器驱动..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo ""
log_green "========================================"
log_green " 录制完成！"
log_green "========================================"
log_yellow " Bag 路径: ${BAG_DIR}"
echo ""

log_green "[Bag 信息]"
ros2 bag info "${BAG_DIR}" 2>/dev/null || true
