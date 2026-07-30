# 离线 Nav2 / RTAB-Map 演示修复记录

更新时间：2026-06-02

## 当前目标

从 `/data/maps/db/first_version_0514.db` 直接生成离线导航所需的 2D 地图、轨迹、障碍物配置。地图必须保留 RTAB-Map 导出的环境占据信息，视觉效果接近 `scripts/offline_nav_maps/rtabmap_20260530_173444_map.pgm`，而不是只有轨迹走廊。

同时保证 RViz 中地图、绿色轨迹、红色障碍物、机器人可视化都在同一个 `map` 坐标平面，并提高机器人导航速度。

## 资产生成方式

统一入口：

```bash
cd /home/dog/catkin_byd_1/wheeltec/rtabmap_nav2_stack
bash prepare_offline_nav.sh
```

该脚本调用：

```bash
src/robot_bringup/scripts/generate_offline_nav_assets.py
```

输入数据库：

```text
/data/maps/db/first_version_0514.db
```

也可以直接指定新的 RTAB-Map 数据库来替换当前离线导航地图和轨迹：

```bash
bash prepare_offline_nav.sh /data/maps/db/underGround_split1.db
```

当前 `prepare_offline_nav.sh` 会把第一个参数作为 DB 路径；如果不传参数，默认仍使用：

```text
/data/maps/db/first_version_0514.db
```

通行区域半径由 `OFFLINE_NAV_CORRIDOR_RADIUS` 控制，默认已经恢复为 `5.0m`：

```bash
OFFLINE_NAV_CORRIDOR_RADIUS=5.0 bash prepare_offline_nav.sh /data/maps/db/underGround_split1.db
```

输出目录：

```text
scripts/offline_nav_maps/
```

主要输出：

```text
scripts/offline_nav_maps/path_waypoints.yaml
scripts/offline_nav_maps/clean_map.yaml
scripts/offline_nav_maps/clean_map.pgm
scripts/offline_nav_maps/obstacle_map.yaml
scripts/offline_nav_maps/obstacle_map.pgm
scripts/offline_nav_maps/obstacles.yaml
```

当前生成结果：

```text
RTAB-Map 原始位姿：1195 个
连续导航位姿段：997 个
降采样轨迹点：171 个
使用扫描节点：1195 / 1195
地图尺寸：4009 x 4883
分辨率：0.05 m/cell
地图原点：[-54.0000, -167.0500, 0.0000]
```

对比旧 RTAB-Map 导出图：

```text
rtabmap_20260530_173444_map.pgm：3913 x 4821
clean_map.pgm：4009 x 4883
```

说明：

- 当前 `clean_map.pgm` 不再是只有轨迹附近的 free corridor。
- 地图从 DB 的 scan 数据投影生成，保留墙体/环境占据像素。
- 为避免 RTAB-Map 投影中的黑色轨迹/扫描击中点挡住机器人，沿连续轨迹开通行区域。当前按需求恢复为 `5.0m` 半径，通行范围比 `0.6m` 更宽。
- DB 扫描点存在少量 X 方向远距离离群点，生成器裁剪了极端 X 分位，避免地图画布被少量远点撑得过宽。
- DB 后段存在大跳变，因此导航轨迹仍只使用从起点开始的第一段连续位姿。
- `path_waypoints.yaml`、`clean_map.yaml`、`obstacle_map.yaml`、`obstacles.yaml` 均由同一次 DB 解析生成，坐标系一致。

## 运行方式

障碍物避障演示：

```bash
bash run_offline_nav.sh obstacle
```

无障碍纯导航：

```bash
bash run_offline_nav.sh clean
```

无图形环境验证：

```bash
bash run_offline_nav.sh obstacle no_rviz
bash run_offline_nav.sh clean no_rviz
```

## 关键修复

### 1. 地图保留环境信息

修改：

```text
src/robot_bringup/scripts/generate_offline_nav_assets.py
```

现在流程：

- 从 DB 读取 RTAB-Map `Node.pose`。
- 从 DB 读取 `Data.scan_info` / `Data.scan`，解压 cv::Mat 扫描数据。
- 使用 scan local transform 和优化位姿投影到 world/map 平面。
- 生成保留环境占据的 `clean_map.pgm`。
- 沿连续轨迹开 `5.0m` 半径通行区域，避免原始轨迹黑线被 Nav2 当成障碍，并扩大可通行范围。
- `obstacle` 模式在同一地图基础上额外写入测试障碍，生成 `obstacle_map.pgm`。

### 2. 绿色轨迹和地图同平面

修改：

```text
src/robot_bringup/scripts/path_publisher.py
```

效果：

- 优先读取 `scripts/offline_nav_maps/path_waypoints.yaml`。
- 发布 `/mapping_path` 时所有点 `z=0`。
- 不再把 DB 中原始 z 值带入 RViz Path。

### 3. clean 模式不显示红色障碍物

修改：

```text
run_offline_nav.sh
src/robot_bringup/launch/offline_avoidance.launch.py
```

效果：

- `clean` 模式加载 `clean_map.yaml` / `clean_map.pgm`。
- `clean` 模式传入 `show_obstacles:=false`，不启动 `obstacle_marker_publisher.py`。
- `obstacle` 模式加载 `obstacle_map.yaml` 并启动红色 Marker。

### 4. 机器人可见

新增：

```text
src/robot_bringup/scripts/robot_marker_publisher.py
```

效果：

- 发布 `/offline_robot_marker`。
- RViz 中显示绑定在 `base_footprint` 上的蓝色机器人本体 Marker。
- 不依赖 URDF / RobotModel。

### 5. 机器人速度和控制稳定性提升

修改：

```text
src/robot_bringup/config/nav2_offline.yaml
prepare_offline_nav.sh
```

当前离线 Nav2 参数：

```text
MPPI vx_max：1.00 m/s
MPPI wz_max：2.00 rad/s
velocity_smoother max_velocity：[1.00, 0.0, 2.00]
max_accel：[2.60, 0.0, 3.60]
controller_frequency：12 Hz
MPPI model_dt：0.083333 s
MPPI batch_size：900
MPPI iteration_count：1
expected_planner_frequency：4 Hz
轨迹路点间距：1.0m -> 2.0m
```

说明：

- 只改 `vx_max` 不够，因为 velocity smoother 也会限速。
- 自动沿轨迹运行时，路点间距从 1m 增加到 2m，减少频繁到点减速。
- MPPI 的 `controller_frequency` 必须和 `model_dt` 匹配，否则 controller_server 会报 `Controller period more then model dt` 并无法激活。
- 当前配置降低了 MPPI 采样负载，避免控制循环频繁 miss 导致绕障和转向变慢。

### 6. 障碍物位置调整

静态障碍物文件：

```text
scripts/offline_nav_maps/obstacles.yaml
```

格式：

```yaml
obstacles:
- cx: 12.3
  cy: 4.5
  width: 2.2
  height: 2.2
```

含义：

```text
cx / cy：障碍物中心点，map 坐标系，单位 m
width / height：障碍物矩形尺寸，单位 m
```

如果手动修改 `obstacles.yaml`，需要重新生成带障碍物地图，最直接方式是重新运行：

```bash
bash prepare_offline_nav.sh
```

注意：当前 `prepare_offline_nav.sh` 会按轨迹自动重新生成 5 个默认障碍物，因此如果想长期保留手动障碍点，需要改 `generate_offline_nav_assets.py` 里的 `make_obstacles()`，或后续把脚本改成优先读取已有 `obstacles.yaml`。

### 7. RViz 点击添加障碍物

新增：

```text
src/robot_bringup/scripts/clicked_obstacle_publisher.py
```

运行 `bash run_offline_nav.sh obstacle` 后，在 RViz 工具栏选择 `Publish Point`，在地图上单击即可添加运行时障碍物。

发布内容：

```text
/clicked_obstacles          红色 MarkerArray，用于 RViz 显示
/clicked_obstacle_points    PointCloud2，用于 Nav2 obstacle_layer 避障
```

Nav2 配置：

```text
src/robot_bringup/config/nav2_offline.yaml
```

其中 `local_costmap` 和 `global_costmap` 已加入 `clicked_obstacle_layer`，订阅 `/clicked_obstacle_points`。

点击障碍物说明：

- 点击生成的是运行时障碍物，不会写回 `obstacles.yaml`。
- 默认点击障碍物尺寸是 `1.6m x 1.6m`。
- 尺寸在 `offline_avoidance.launch.py` 的 `clicked_obstacle_publisher.py` 节点参数中修改：

```text
obstacle_width
obstacle_height
obstacle_z
```

避障调优：

```text
local_costmap inflation_radius：1.00m
global_costmap inflation_radius：1.20m
cost_scaling_factor：1.6
```

说明：

- 点击障碍物会同时进入 local/global costmap。
- 全局膨胀半径更大，目的是让 Nav2 更早重新规划绕行路线，而不是贴近障碍物后靠局部控制原地慢慢转。
- 点击障碍物本体从 `2.2m` 降到 `1.6m`，避免障碍物加膨胀层后过宽，把可通行区域挤得太窄。

### 8. RViz 2D Pose Estimate / 2D Goal Pose 接管自动轨迹

修改：

```text
src/robot_bringup/scripts/path_waypoint_sender.py
src/robot_bringup/launch/offline_avoidance.launch.py
```

当前行为：

- 启动后默认自动沿提取轨迹运行。
- 自动轨迹不再逐个 `NavigateToPose` 路点发送，而是一次发送 `NavigateThroughPoses` 路线，减少直线段到每个中间点时停下、对角度、原地转圈的问题。
- RViz 点击 `2D Pose Estimate` 后：
  - `path_waypoint_sender.py` 收到 `/initialpose`。
  - 取消当前自动路线。
  - `odom_from_cmd_vel.py` 把机器人放到指定位置。
  - 不再继续回到旧轨迹。
- RViz 点击 `2D Goal Pose` 后：
  - `path_waypoint_sender.py` 收到 `/goal_pose`。
  - 取消当前自动路线。
  - 等 Nav2 结束 `NavigateThroughPoses` 后，再用 `NavigateToPose` action 发送 RViz 指定目标。
  - 如果 Nav2 尚未完全空闲导致目标被拒，会自动重试。

重要实现：

```text
offline_avoidance.launch.py
```

在 Nav2 GroupAction 中加入：

```text
SetRemap('/goal_pose', '/nav2_direct_goal_pose_disabled')
```

原因：

- RViz 默认把 `2D Goal Pose` 发布到 `/goal_pose`。
- 如果 Nav2 自己直接订阅 `/goal_pose`，它会在自动 `NavigateThroughPoses` 尚未取消完成时抢先收到目标并拒绝，表现为点击 2D Goal Pose 没反应。
- 现在 `/goal_pose` 先由 `path_waypoint_sender.py` 接管，再通过 action client 发给 Nav2。

### 9. 启动清理增强

修改：

```text
run_offline_nav.sh
```

现在启动前会额外清理：

```text
map_server / lifecycle_manager
controller_server / planner_server / bt_navigator
waypoint_follower / velocity_smoother / smoother_server / behavior_server
path_waypoint_sender.py / odom_from_cmd_vel.py / path_publisher.py
clicked_obstacle_publisher.py / robot_marker_publisher.py / obstacle_marker_publisher.py
```

原因：

- 多次启动后，如果旧 Nav2 节点残留，会抢占 service/action/topic。
- 典型表现是第二次启动 RViz 或 map_server 生命周期失败、地图不显示、目标无响应。
- 新清理逻辑降低第二次启动失败概率。

## 验证结果

已执行：

```bash
python3.10 -m py_compile src/robot_bringup/scripts/generate_offline_nav_assets.py
python3.10 -m py_compile src/robot_bringup/scripts/path_waypoint_sender.py
python3.10 -c "import yaml; yaml.safe_load(open('src/robot_bringup/config/nav2_offline.yaml'))"
bash prepare_offline_nav.sh
timeout 90s bash run_offline_nav.sh obstacle no_rviz
```

验证到：

```text
map_server 加载：scripts/offline_nav_maps/obstacle_map.pgm
地图尺寸：4009 X 4883 @ 0.05 m/cell
path_publisher 加载：171 poses
obstacle_marker_publisher 加载：5 obstacle markers
clicked_obstacle_publisher 启动并等待 RViz Publish Point
Nav2 costmap 订阅 clicked_obstacles obstacle layer
path_waypoint_sender 移动：6 个落在占据像素上的 waypoint 到附近可通行像素
Nav2 正常发送 /cmd_vel_nav
odom pose 持续变化
自动路线第一次如遇 Nav2 尚未完全 active，会自动重试并成功发送
```

手动目标验证：

```text
发布 /initialpose 后：
path_waypoint_sender: Stopping automatic trajectory: RViz 2D Pose Estimate received
path_waypoint_sender: Canceled current Nav2 goal
odom_from_cmd_vel: Initial pose set: (2.00, -1.00, yaw=0.00)

发布 /goal_pose 后：
path_waypoint_sender: Stopping automatic trajectory: RViz 2D Goal Pose received
path_waypoint_sender: Sending RViz goal to Nav2: (5.00, -2.00)
bt_navigator: Begin navigating from current location ... to (5.00, -2.00)
controller_server: Reached the goal!
path_waypoint_sender: RViz goal finished
```

速度日志示例：

```text
cmd_vel_nav: vx=0.547 m/s
cmd_vel_nav: vx=0.571 m/s
odom pose: x=1.90, y=-0.77
odom pose: x=4.74, y=-1.88
```

说明速度已经明显高于旧版本约 `0.10 m/s` 的表现，同时避免了过重 MPPI 参数导致控制循环长时间 miss。

## 注意事项

- 当前 Codex 沙箱没有 X display，RViz 不能在这里目视检查；请在本机桌面环境运行不带 `no_rviz` 的命令查看界面。
- 沙箱日志中常见 `TRANSPORT_UDP Error` / `getifaddrs: Operation not permitted`，这是沙箱网络权限导致；本次 launch 内节点仍能通信并完成导航。
- `clean_map_0518.pgm` 是旧的走廊图遗留文件，当前运行脚本使用的是 `clean_map.pgm` 和 `obstacle_map.pgm`。

## 2026-06-04 自动路径中的点击障碍物重规划

用户需求：

```text
启动后机器人默认沿全局轨迹移动；
遇到 RViz Publish Point 设置的障碍物时，不需要再手动点击 2D Pose Estimate / 2D Goal Pose；
系统应自动规划绕开障碍物的局部/全局路径，绕开后继续回到原全局轨迹。
```

本次修改的核心文件：

```text
src/robot_bringup/scripts/path_waypoint_sender.py
src/robot_bringup/scripts/clicked_obstacle_publisher.py
src/robot_bringup/launch/offline_avoidance.launch.py
src/robot_bringup/config/nav2_offline.yaml
```

实现方式：

- 自动跟踪不再把整条轨迹作为不可变的穿点任务，而是按 `auto_goal_distance=8.0m` 选择前视目标，连续发送 `NavigateToPose`。
- `path_waypoint_sender.py` 现在订阅 `/clicked_point`。自动模式下收到 RViz Publish Point 后，会取消当前 Nav2 目标，并在 `auto_replan_delay=0.8s` 后重新发送同一个前视目标。
- 这个 0.8 秒延时是为了让 `clicked_obstacle_publisher.py` 先把障碍物点云发布到 `/clicked_obstacle_points`，再让 global/local costmap 标记障碍物。
- 重新发送同一个前视目标后，Nav2 会在已经包含点击障碍物的 `global_costmap` 上重新规划路径；MPPI controller 再沿新路径局部避障。
- 到达这个前视目标后，`path_waypoint_sender.py` 会继续选择后面的轨迹点，因此绕障后会回到原来的全局轨迹跟踪流程。
- 为避免“旧目标取消结果晚到”误伤新目标，`path_waypoint_sender.py` 给每个 Nav2 goal 增加了 `goal_sequence`，过期结果会被忽略。

点击障碍物显示和代价地图：

- RViz 的 Publish Point 仍发布到 `/clicked_point`。
- `clicked_obstacle_publisher.py` 把点击点转换为红色 MarkerArray `/clicked_obstacles` 和点云 `/clicked_obstacle_points`。
- global_costmap 和 local_costmap 的 `clicked_obstacle_layer` 都订阅 `/clicked_obstacle_points`。
- 点云采样间隔从 `0.20m` 改成 `0.05m`，避免障碍物在 costmap 中过于稀疏。

规划参数：

```text
planner_server/GridBased/use_astar: true
```

说明：

- 仍然使用 `nav2_navfn_planner/NavfnPlanner`。
- `use_astar=true` 后，重新规划时使用 A* 风格搜索，比默认 Dijkstra 更适合频繁点障碍后快速找到绕行路径。

验证：

```bash
python3.10 -m py_compile src/robot_bringup/scripts/path_waypoint_sender.py src/robot_bringup/scripts/clicked_obstacle_publisher.py
python3.10 -c "import yaml; yaml.safe_load(open('src/robot_bringup/config/nav2_offline.yaml'))"
timeout 75s bash run_offline_nav.sh obstacle no_rviz
```

验证到：

```text
clicked_obstacle_publisher 订阅 /clicked_point
path_waypoint_sender 订阅 /clicked_point
global_costmap/local_costmap 订阅 clicked_obstacles 点云源
自动模式可以连续发送前视 NavigateToPose 目标
Nav2 controller 持续收到并执行新 path
```

命令行测试 Publish Point 时推荐用持续发布，而不是 `--once`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 2s ros2 topic pub -r 10 /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 22.0, y: 0.8, z: 0.0}}"
```

原因：

- `ros2 topic pub --once` 在当前环境里有时会卡在等待订阅者匹配。
- RViz 的 Publish Point 不受这个问题影响，正常点击即可。

## 2026-06-04 红色局部路径终点放障碍物的修复

问题现象：

```text
自动模式会把全局轨迹拆成一段段 NavigateToPose。
如果障碍物放在当前红色局部路径中间，Nav2 能绕开。
如果障碍物放在这段红色路径的终点，终点本身变成障碍物，机器人仍可能贴着障碍物去“到达目标”，后续规划容易卡住。
```

本次修复：

```text
src/robot_bringup/scripts/path_waypoint_sender.py
src/robot_bringup/launch/offline_avoidance.launch.py
src/robot_bringup/config/nav2_offline.yaml
```

实现逻辑：

- `path_waypoint_sender.py` 记录自动模式下收到的 `/clicked_point` 动态障碍物坐标。
- 自动选择前视目标时，不再允许把动态障碍物安全半径内的轨迹点作为 Nav2 goal。
- 如果当前红色路径终点被点成障碍物，自动重规划时会从当前目标开始向后寻找第一个不在障碍物安全半径内的轨迹点。
- 找到后会记录类似日志：

```text
Auto target waypoint 9/171 is inside a clicked obstacle safety radius (1.93m); using waypoint 10/171 instead
```

当前参数：

```text
clicked_obstacle_width: 1.6
clicked_obstacle_height: 1.6
dynamic_goal_clearance: 0.8
dynamic_goal_block_radius: 0.0
```

说明：

- `dynamic_goal_block_radius=0.0` 表示自动计算安全半径。
- 计算方式约为：红色障碍物半对角线 + `dynamic_goal_clearance`。
- 现在 1.6m x 1.6m 障碍物对应的目标禁用半径约为 `1.93m`。
- 这个半径只用于“自动目标点能不能选”，不改变 RViz 红色障碍物尺寸。

同时收紧 NavfnPlanner 容差：

```text
planner_server/GridBased/tolerance: 0.20
```

原因：

- 原来 `0.50m` 容差较大，目标点被动态障碍物占据时，Nav2 可能仍把障碍物边缘附近当作可接受目标。
- 收紧到 `0.20m` 后，被障碍物占用的目标更不容易被误判为可达；自动模式会继续选择后面的安全路点。

验证：

```bash
python3.10 -m py_compile src/robot_bringup/scripts/path_waypoint_sender.py src/robot_bringup/scripts/clicked_obstacle_publisher.py
python3.10 -c "import yaml; yaml.safe_load(open('src/robot_bringup/config/nav2_offline.yaml'))"
```

测试建议：

- 启动 `bash run_offline_nav.sh obstacle`。
- 等红色局部路径出现后，在红色路径终点附近用 Publish Point 放障碍物。
- 期望表现：当前目标被取消，自动目标跳到障碍物后方的轨迹点，Nav2 重新规划绕开障碍物，然后继续沿后续全局轨迹走。

## 2026-06-04 禁止倒车并先转正再前进

问题现象：

```text
在 /data/maps/db/underGround_split1.db 上，机器人初始朝向和轨迹方向不一致。
Nav2 规划出路径后，机器人会先转圈、倒车，速度很慢，之后才正常。
在其他地图中测试动态障碍物时，偶尔也会出现倒车。
```

本次修改：

```text
src/robot_bringup/config/nav2_offline.yaml
src/robot_bringup/scripts/path_waypoint_sender.py
```

控制器修改：

```text
FollowPath:
  plugin: nav2_rotation_shim_controller::RotationShimController
  primary_controller: nav2_mppi_controller::MPPIController
  angular_dist_threshold: 0.45
  angular_disengage_threshold: 0.22
  forward_sampling_distance: 1.0
  rotate_to_heading_angular_vel: 1.4
  max_angular_accel: 3.6
  simulate_ahead_time: 1.0
  rotate_to_goal_heading: false
```

说明：

- `RotationShimController` 包在 MPPI 外面。
- 当机器人当前朝向和路径起始方向偏差较大时，先原地旋转对准路径。
- 对准后再交给 MPPI 前进，避免一开始用倒车/斜着挪的方式进入路径。

禁止倒车：

```text
MPPI vx_min: 0.00
velocity_smoother min_velocity: [0.00, 0.0, -2.00]
```

说明：

- MPPI 不再采样负的 x 方向速度。
- velocity smoother 也不再允许输出负 x 方向速度。
- 机器人仍然可以原地正反向旋转，因为角速度最小值仍保留 `-2.00 rad/s`。

轨迹朝向修正：

- `path_waypoint_sender.py` 新增 `trajectory_yaw()`。
- 初始 `/initialpose` 的 yaw 不再直接使用 DB 第一个姿态 yaw，而是使用第一个有效轨迹段方向。
- 自动目标点 yaw 也使用目标附近轨迹段方向。
- 对 `underGround_split1.db` 这类起点 yaw 和轨迹方向不一致的数据，启动后机器人头部会更接近轨迹方向。

验证：

```bash
python3.10 -m py_compile src/robot_bringup/scripts/path_waypoint_sender.py
python3.10 -c "import yaml; yaml.safe_load(open('src/robot_bringup/config/nav2_offline.yaml'))"
timeout 45s bash run_offline_nav.sh obstacle no_rviz
```

验证日志：

```text
Created controller : FollowPath of type nav2_rotation_shim_controller::RotationShimController
Created internal controller for rotation shimming: FollowPath of type nav2_mppi_controller::MPPIController
MPPIController: Activated MPPI Controller: FollowPath
odom_from_cmd_vel: Initial pose set: (-0.00, 0.00, yaw=0.45)
cmd_vel_nav: vx=0.000 m/s, wz=0.300 rad/s
cmd_vel_nav: vx=0.745 m/s, wz=-0.020 rad/s
```

含义：

- 控制器成功加载，没有崩溃。
- 起点 yaw 已按轨迹方向设置为约 `0.45 rad`。
- 航向偏差较大时先原地转向，`vx=0.000`。
- 转向后再前进，未观察到负 `vx` 倒车。

## 2026-06-05 clean 模式 costmap 无地图修复

用户日志：

```text
debug11.txt
bash run_offline_nav.sh clean
```

关键问题：

```text
map_server.rclcpp: failed to send response to /map_server/change_state (timeout)
nav2_costmap_2d: Robot is out of bounds of the costmap!
local_costmap.local_costmap: Can't update static costmap layer, no map received
global_costmap.global_costmap: Can't update static costmap layer, no map received
```

原因：

- `clean_map.pgm` 较大，map_server 配置/激活和发布 `/map` 需要时间。
- Nav2 navigation stack 过早启动，global/local costmap 在 `/map` 真正可用前开始初始化。
- costmap 没拿到 static map 时仍按默认小窗口运行，因此机器人初始点被判断为 `out of bounds`。
- waypoint sender 也过早启动，Nav2 未 fully active 时自动目标会被拒绝。
- waypoint sender 自己发布 `/initialpose` 的回调有时晚到，原先 1 秒忽略窗口太短，会误判成用户点击 `2D Pose Estimate`，从而停止自动模式。

本次修复：

```text
src/robot_bringup/launch/offline_avoidance.launch.py
src/robot_bringup/scripts/path_waypoint_sender.py
```

启动顺序调整：

```text
map_server + lifecycle_manager_map_server 立即启动
Nav2 navigation stack 延后 8 秒启动
path_waypoint_sender 延后 18 秒启动
```

map_server lifecycle manager 参数：

```text
bond_timeout: 20.0
```

作用：

- 给 map_server 更长时间完成 lifecycle bond，避免大地图加载时 lifecycle manager 过早超时。

waypoint sender 修改：

```text
ignore_initialpose_until: 1.0s -> 3.0s
```

作用：

- `path_waypoint_sender.py` 启动时会主动发布 5 次 `/initialpose`。
- 这些消息可能延迟回调到本节点。
- 忽略窗口加长后，不会把自己发布的初始位姿误认为用户手动点击了 `2D Pose Estimate`。

验证：

```bash
python3.10 -m py_compile src/robot_bringup/scripts/path_waypoint_sender.py src/robot_bringup/launch/offline_avoidance.launch.py
python3.10 -c "import yaml; yaml.safe_load(open('src/robot_bringup/config/nav2_offline.yaml'))"
timeout 55s bash run_offline_nav.sh clean no_rviz
```

验证结果：

```text
map_server 先进入 active
Nav2 延后启动
local_costmap StaticLayer: Resizing static layer to 1744 X 2683
global_costmap StaticLayer: Resizing costmap to 1744 X 2683
未再出现 Robot is out of bounds of the costmap
未再出现 Can't update static costmap layer, no map received
```

注意：

- 因为 Nav2 和 waypoint sender 被刻意延后，启动后自动运动会比之前晚几秒，这是为了换取 clean/obstacle 大地图启动稳定性。
