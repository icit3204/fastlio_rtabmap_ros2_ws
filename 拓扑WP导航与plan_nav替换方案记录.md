# 拓扑 WP 导航与 /plan_nav 替换方案记录

本文记录两套方案：

1. 推荐方案：把每个 WP 点作为 Nav2 目标点，保留 Nav2 避障能力。
2. 临时方案：让 `main.py` 发布的 `/plan_nav` 直接替换 `/plan`，驱动现有轮椅控制链路。

## 当前背景

`/home/dog/plan_nav/main.py` 运行后会生成或使用拓扑路径数据，例如：

```text
/home/dog/fastlio_rtabmap_ros2_ws/map/采集/大厅2
```

该目录包含：

```text
nodes.txt          WP 节点
edges.txt          WP 之间的边
edge_N_traj.txt    每条边对应的轨迹点
```

`main.py` 在操作模式下会发布：

```text
/plan_nav
```

消息类型为：

```text
nav_msgs/msg/Path
```

当前导航启动文件中的 `pure_pursuit_controller_node` 默认订阅：

```text
/plan
```

也就是 Nav2 规划器输出的路径。

## 方案一：WP 作为 Nav2 目标点，保留避障能力

这是推荐方案。

### 核心思想

`main.py` 不直接发布最终控制路径给轮椅，而是只负责：

1. 读取拓扑图。
2. 选择起点 WP 和终点 WP。
3. 计算 WP 序列。
4. 把每个 WP 当作 Nav2 目标点依次发送。

Nav2 继续负责在实时 costmap 上规划路径，并输出：

```text
/plan
```

轮椅控制链路保持不变：

```text
Nav2 /plan
  -> pure_pursuit_controller
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> can0
  -> 轮椅
```

### 运行逻辑示例

如果拓扑规划结果为：

```text
WP-08 -> WP-01 -> WP-02 -> WP-03 -> WP-04
```

执行时不是把整条固定轨迹直接发给轮椅，而是：

```text
1. 发送 WP-01 给 Nav2
2. 到达 WP-01 后发送 WP-02
3. 到达 WP-02 后发送 WP-03
4. 到达 WP-03 后发送 WP-04
```

也可以进一步使用 Nav2 的 `NavigateThroughPoses`，一次性发送多个途经点：

```text
[WP-01, WP-02, WP-03, WP-04]
```

### 优点

保留 Nav2 动态避障和重规划能力。

```text
WP 负责决定去哪里
Nav2 负责决定怎么绕过去
```

当局部或全局 costmap 中出现障碍物时，Nav2 可以重新规划 `/plan`。

### 注意事项

该方案依赖 Nav2 costmap 和 planner 正常工作。

真机上还应补充近距离安全层，例如：

```text
collision_monitor
```

或者单独的 safety gate，负责最后一道强制停车或限速。

当前项目里需要特别注意：

1. `bringup_2d.launch.py` 中 `collision_monitor` 的加入动作目前是注释状态。
2. `pure_pursuit_controller.cpp` 中根据 `/cmd_vel` 停车的逻辑目前是注释状态。

因此，仅依靠当前代码直接运行，并不等于已经具备完整近距离急停保护。

## 方案二：/plan_nav 直接替换 /plan

这是之前讨论过的临时方案，不推荐作为最终真机导航方案。

### 核心思想

让 `pure_pursuit_controller_node` 不再订阅 `/plan`，而是订阅：

```text
/plan_nav
```

链路变为：

```text
main.py /plan_nav
  -> pure_pursuit_controller
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> can0
  -> 轮椅
```

### 临时运行方式

数据集导航时：

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash

pkill -f '/pure_pursuit_controller_node'

ros2 run wheelchair_controller pure_pursuit_controller_node --ros-args \
  -p use_sim_time:=true \
  -p path_topic_name:=/plan_nav \
  -p lookahead_distance:=1.0 \
  -p min_turning_radius:=1.5 \
  -p linear_velocity:=3.0 \
  -p goal_tolerance:=0.9 \
  -p goal_yaw_tolerance:=3.14
```

真机导航时：

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash

pkill -f '/pure_pursuit_controller_node'

ros2 run wheelchair_controller pure_pursuit_controller_node --ros-args \
  -p use_sim_time:=false \
  -p path_topic_name:=/plan_nav \
  -p lookahead_distance:=1.0 \
  -p min_turning_radius:=1.5 \
  -p linear_velocity:=3.0 \
  -p goal_tolerance:=0.9 \
  -p goal_yaw_tolerance:=3.14
```

底层 CAN 控制节点还需要启动：

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p auto_start:=true
```

### 验证方式

确认 `pure_pursuit_controller` 已改为订阅 `/plan_nav`：

```bash
ros2 node info /pure_pursuit_controller
```

应看到：

```text
/plan_nav: nav_msgs/msg/Path
```

确认 `/plan_nav` 有数据：

```bash
ros2 topic hz /plan_nav
ros2 topic echo /plan_nav nav_msgs/msg/Path --once
```

确认已经产生轮椅控制命令：

```bash
ros2 topic hz /wheelchair_control_command
ros2 topic echo /wheelchair_control_command
```

确认底层控制节点已订阅：

```bash
ros2 topic info -v /wheelchair_control_command
```

应看到：

```text
Subscription count: 1
```

### 风险

`/plan_nav` 是 `main.py` 根据拓扑边轨迹生成的固定路径，不经过 Nav2 的实时 costmap 规划。

因此该方案不会天然保留 Nav2 动态避障能力：

```text
main.py /plan_nav 是固定轨迹
Nav2 /plan 是基于 costmap 的实时规划路径
```

如果遇到临时障碍物，`/plan_nav` 不会自动绕开。

此外，如果拓扑边文件引用错误，例如 `edges.txt` 中某条边引用了不匹配的 `edge_N_traj.txt`，轮椅可能追踪到错误路径。

已发现示例：

```text
/home/dog/fastlio_rtabmap_ros2_ws/map/采集/大厅2
```

其中：

```text
8 -> 1  引用了 edge_8_traj.txt，但该轨迹实际对应 11 -> 10
8 -> 6  引用了 edge_9_traj.txt，但该轨迹实际对应 10 -> 9
```

## 结论

最终建议采用方案一：

```text
main.py 输出 WP 序列
Nav2 接收 WP 目标点
Nav2 输出 /plan
pure_pursuit_controller 继续订阅 /plan
轮椅继续按现有控制链路执行
```

方案二只适合作为临时验证 `main.py` 路径生成和轮椅跟踪能力的调试手段，不适合作为最终保留避障能力的真机导航方案。
