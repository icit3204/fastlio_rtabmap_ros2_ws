# Ubuntu 22.04 环境需求清单

> 适用版本：Ubuntu 22.04 LTS（Jammy Jellyfish）
> 项目：Underground Map Editor（2D 版，基线 = `windows_version/2d`）
> 本副本路径：`F:\datasets\db\ubuntu_version\2d`

---

## 〇、与旧适配文档的差异说明

本目录依据**最新基线** `windows_version/2d` 重新适配，依赖以实际 `import` 为准：

- 本项目为 **2D 地图编辑器**，使用 PyQt5 `QGraphicsView` 渲染，**不依赖 Open3D**。
- 实测**未使用** Pillow、scipy、matplotlib，旧文档中的这些条目已移除。
- 实际第三方依赖仅：PyQt5、numpy、opencv-python、networkx、requests；ROS2（rclpy/tf2）仅"操作模式"需要。

---

## 一、系统基础依赖

```bash
sudo apt update && sudo apt install -y \
    python3.10 python3.10-venv python3-pip python3-dev \
    build-essential cmake git \
    libgl1-mesa-glx libglib2.0-0 \
    libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
    libxcb-xkb1 libxkbcommon-x11-0 libdbus-1-3 libegl1 libxcb-cursor0
```

> `libxcb-*` 系列是 PyQt5 在 Linux 下加载 `xcb` 平台插件所需，缺少会报
> `qt.qpa.plugin: could not load platform plugin "xcb"`。
> `libgl1-mesa-glx` 供 OpenCV / Qt 的 OpenGL 后端使用。

---

## 二、Python 依赖（pip）

建议用 virtualenv 隔离：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

安装依赖（与本项目实际 import 一致）：

```bash
pip install \
    PyQt5==5.15.9 PyQt5-sip \
    numpy>=1.24 \
    opencv-python>=4.8 \
    networkx>=3.1 \
    requests>=2.31
```

或直接用随附的 `requirements.txt`：

```bash
pip install -r requirements.txt
```

### 各包用途

| 包名 | 用途 | 对应功能 |
|------|------|----------|
| PyQt5 / PyQt5-sip | GUI 框架（主窗口、QGraphicsView 2D 视口、控件、信号槽） | F-2.x、F-8.x、F-13/14.x |
| numpy | 坐标变换、图像缓冲、矩阵运算 | F-2.2、视频处理 |
| opencv-python | 视频抽帧、图像读写（`core/video_processor.py`） | F-9.x |
| networkx | 拓扑图、Dijkstra 路径规划 | F-4.x、F-6.x |
| requests | Roboflow 目标检测 HTTP 调用（`core/object_detector.py`） | F-10.x |

---

## 三、ROS2 Humble（仅"操作模式"需要）

仅当使用**操作模式**（实时 TF2 位姿，`core/pose_receiver.py`）时安装。
调试 / 地图编辑全流程无需 ROS2。

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-geometry-msgs
```

> `core/pose_receiver.py` 已自动探测 `/opt/ros/<distro>/setup.bash` 并在子进程内 source，无需手动 source。
> `data/wheelchair_controller.launch.py` 使用 `gnome-terminal --` 前缀，Ubuntu 桌面环境原生可用。

---

## 四、字体（推荐安装，影响等宽排版与 emoji 图标）

```bash
# 等宽字体（代码 / 日志面板）
sudo apt install -y fonts-ubuntu fonts-dejavu fonts-liberation
# 车辆指示 🚗 emoji
sudo apt install -y fonts-noto-color-emoji
fc-cache -f -v
```

> 程序通过 `core/ui_font.py` 在运行时按
> `Courier New → Ubuntu Mono → DejaVu Sans Mono → Liberation Mono → Noto Sans Mono → Monospace`
> 顺序自动选择等宽字体，**即使一个都没装也能用 Qt 通用 Monospace 兜底**。
> emoji 字体缺失时，`core/car_indicator.py` 自动回退为 QPainter 矢量车形（`DrawnCarItem`），功能不受影响。

---

## 五、运行验证

```bash
source .venv/bin/activate
python3 -c "from PyQt5.QtWidgets import QApplication; print('PyQt5 OK')"
python3 -c "import cv2, numpy, networkx, requests; print('deps OK')"
python3 main.py
```

无显示器 / SSH 环境可先用离屏后端冒烟测试：

```bash
QT_QPA_PLATFORM=offscreen python3 -c "import main; print('import OK')"
```

---

## 六、常见问题

| 错误信息 | 原因 | 解决 |
|----------|------|------|
| `could not load platform plugin "xcb"` | 缺 Qt XCB 依赖 | 执行第一节 `libxcb-*` 安装 |
| `libGL error: unable to load` | 缺 Mesa OpenGL | `sudo apt install libgl1-mesa-glx` |
| 等宽未对齐 / 字体异常 | 无任何候选等宽字体 | 安装第四节字体并 `fc-cache -f -v` |
| `ModuleNotFoundError: PyQt5` | 未激活虚拟环境 | `source .venv/bin/activate` |
| ROS2 ImportError | 未装 `/opt/ros/humble` | 参考第三节（仅操作模式需要） |
