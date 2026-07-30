# rtsp_camera_bridge

与 RTSP 节点相关的代码集中在此包内：从 RTSP 拉流，发布 `sensor_msgs/Image` 与 `sensor_msgs/CameraInfo`，支持断线重连。

## 脚本

- **`build_rtsp_bridge.sh`** — 编译本包（在 workspace 下执行 `colcon build --packages-select rtsp_camera_bridge`）
- **`start_rtsp_bridge.sh`** — 启动 RTSP 桥接节点（可编辑脚本内默认 `RTSP_URL` 或通过环境变量传入）

## 依赖

- ROS2 Humble
- `python3-opencv`、`python3-yaml`（`sudo apt install python3-opencv python3-yaml`）

## 单独运行节点

```bash
source /opt/ros/humble/setup.bash
source /path/to/install/setup.bash

# 方式一：launch（推荐）
ros2 launch rtsp_camera_bridge rtsp_camera_bridge.launch.py \
  rtsp_url:='rtsp://user:pass%40@192.168.1.10:554/stream' \
  image_topic:=/sensors/camera/rgb/image_rect \
  camera_info_topic:=/sensors/camera/rgb/camera_info

# 方式二：run
ros2 run rtsp_camera_bridge rtsp_camera_bridge.py --ros-args \
  -p rtsp_url:='rtsp://...' \
  -r image_raw:=/sensors/camera/rgb/image_rect \
  -r camera_info:=/sensors/camera/rgb/camera_info
```

URL 中密码若含 `@`，需写成 `%40`。

## 与 robot_bringup 集成

在 `bringup.launch.py` 中当 `rtsp_url` 非空时会自动启动本包节点，并传入 `rgb_topic`、`camera_info_topic`、`camera_frame_id` 等。

## 目录结构

```
rtabmap_ros/rtsp_camera_bridge/
├── package.xml
├── CMakeLists.txt
├── README.md
├── build_rtsp_bridge.sh        # 编译脚本
├── start_rtsp_bridge.sh        # 启动脚本
├── scripts/
│   └── rtsp_camera_bridge.py   # 节点实现
├── launch/
│   └── rtsp_camera_bridge.launch.py
└── config/
    └── README.md               # 标定文件说明
```
