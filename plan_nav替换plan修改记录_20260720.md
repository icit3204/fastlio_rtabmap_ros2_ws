# /plan_nav 替换 /plan 修改记录

记录时间：2026-07-20 11:58 CST

## 修改目的

按“方案二”执行：让轮椅纯跟踪控制节点 `pure_pursuit_controller_node` 默认订阅 `main.py` 发布的 `/plan_nav`，而不是 Nav2 发布的 `/plan`。

修改后的控制链路：

```text
/home/dog/plan_nav/main.py
  -> /plan_nav
  -> pure_pursuit_controller_node
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> can0
  -> 轮椅
```

## 本次修改的文件

### 1. pure_pursuit_controller 默认路径话题

文件：

```text
/home/dog/fastlio_rtabmap_ros2_ws/src/wheelchair_controller/src/pure_pursuit_controller.cpp
```

修改位置：

```text
line 26
```

修改前：

```cpp
this->declare_parameter("path_topic_name", "/plan");
```

修改后：

```cpp
this->declare_parameter("path_topic_name", "/plan_nav");
```

作用：

```text
当不通过 launch 或命令行显式传入 path_topic_name 时，
pure_pursuit_controller_node 默认订阅 /plan_nav。
```

### 2. bringup_2d.launch.py 启动参数

文件：

```text
/home/dog/fastlio_rtabmap_ros2_ws/src/robot_bringup/launch/bringup_2d.launch.py
```

修改位置：

```text
line 381
```

修改前：

```python
'path_topic_name': "/plan",
```

修改后：

```python
'path_topic_name': "/plan_nav",
```

作用：

```text
使用 ros2 launch robot_bringup bringup_2d.launch.py 启动导航时，
pure_pursuit_controller_node 会订阅 /plan_nav。
```

该 launch 文件用于当前 run.txt 中的数据集导航和真机导航命令。

### 3. bringup_2d_infra.launch.py 启动参数

文件：

```text
/home/dog/fastlio_rtabmap_ros2_ws/src/robot_bringup/launch/bringup_2d_infra.launch.py
```

修改位置：

```text
line 445
```

修改前：

```python
'path_topic_name': "/plan",
```

修改后：

```python
'path_topic_name': "/plan_nav",
```

作用：

```text
使用 bringup_2d_infra.launch.py 启动导航时，
pure_pursuit_controller_node 会订阅 /plan_nav。
```

## 已执行构建

因为修改了 C++ 文件，已重新构建 `wheelchair_controller`：

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
colcon build --packages-select wheelchair_controller --symlink-install
```

构建结果：

```text
Summary: 1 package finished
```

说明：

```text
install/robot_bringup/share/robot_bringup/launch/bringup_2d.launch.py
install/robot_bringup/share/robot_bringup/launch/bringup_2d_infra.launch.py
```

是指向 `src/robot_bringup/launch/` 的软链接，因此 launch 文件修改后无需单独重新安装。

## 当前验证结果

已确认源码和 launch 文件中 `path_topic_name` 均为：

```text
/plan_nav
```

检查命令：

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
rg -n "path_topic_name|declare_parameter\\(\"path_topic_name\"" \
  src/wheelchair_controller/src/pure_pursuit_controller.cpp \
  src/robot_bringup/launch/bringup_2d.launch.py \
  src/robot_bringup/launch/bringup_2d_infra.launch.py
```

预期结果：

```text
src/robot_bringup/launch/bringup_2d.launch.py:381: 'path_topic_name': "/plan_nav",
src/robot_bringup/launch/bringup_2d_infra.launch.py:445: 'path_topic_name': "/plan_nav",
src/wheelchair_controller/src/pure_pursuit_controller.cpp:26: declare_parameter("path_topic_name", "/plan_nav")
```

## 启动后的验证方法

启动导航后检查 `pure_pursuit_controller` 实际订阅话题：

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
ros2 node info /pure_pursuit_controller
```

应看到：

```text
Subscribers:
  /plan_nav: nav_msgs/msg/Path
```

检查 `/plan_nav` 是否有数据：

```bash
ros2 topic hz /plan_nav
ros2 topic echo /plan_nav nav_msgs/msg/Path --once
```

检查 pure pursuit 是否输出轮椅控制命令：

```bash
ros2 topic hz /wheelchair_control_command
ros2 topic echo /wheelchair_control_command
```

检查底层轮椅控制节点是否接收命令：

```bash
ros2 topic info -v /wheelchair_control_command
```

应看到：

```text
Subscription count: 1
```

如果 `Subscription count: 0`，说明 `wheelchair_controller_node` 没有启动。

## 回退到 /plan 的方法

如果需要恢复 Nav2 `/plan` 控制链路，将以下三处改回 `/plan`。

### 1. 回退 pure_pursuit_controller.cpp

文件：

```text
src/wheelchair_controller/src/pure_pursuit_controller.cpp
```

改回：

```cpp
this->declare_parameter("path_topic_name", "/plan");
```

### 2. 回退 bringup_2d.launch.py

文件：

```text
src/robot_bringup/launch/bringup_2d.launch.py
```

改回：

```python
'path_topic_name': "/plan",
```

### 3. 回退 bringup_2d_infra.launch.py

文件：

```text
src/robot_bringup/launch/bringup_2d_infra.launch.py
```

改回：

```python
'path_topic_name': "/plan",
```

### 4. 回退后重新构建

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
colcon build --packages-select wheelchair_controller --symlink-install
```

然后重启导航。

## 风险说明

该修改会让轮椅跟随 `/plan_nav` 固定路径，而不是 Nav2 根据 costmap 生成的 `/plan`。

因此：

```text
不会自动保留 Nav2 动态避障能力。
```

如果 `/plan_nav` 对应的拓扑轨迹文件有错误，轮椅可能跟随错误轨迹。

此前已发现：

```text
/home/dog/fastlio_rtabmap_ros2_ws/map/采集/大厅2
```

中存在边和轨迹文件不匹配的问题：

```text
8 -> 1  引用了 edge_8_traj.txt，但 edge_8_traj.txt 实际对应 11 -> 10
8 -> 6  引用了 edge_9_traj.txt，但 edge_9_traj.txt 实际对应 10 -> 9
```

真机测试前应先确认 `/plan_nav` 路径在 RViz 中正确，并确保急停可用。
