# CAN 测试与键盘控制说明

本文档用于当前工程的轮椅底盘 CAN 测试、ROS 控制节点测试、键盘直接控制，以及修改前进、后退、转向速度。

当前工程路径:

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
```

## 1. 安全注意

真机测试前建议先架空驱动轮，或确保急停开关可用。首次测试速度要小，确认方向正确后再提高速度。

当前 CAN 默认参数:

```text
CAN interface : can0
bitrate       : 500000
CAN ID        : 0x801400
frame type    : extended frame
DLC           : 8
```

CAN payload:

```text
byte 0-1: left velocity,  int16 little-endian
byte 2-3: right velocity, int16 little-endian
byte 4-5: left distance,  int16 little-endian
byte 6-7: right distance, int16 little-endian
```

用 `cansend` 手动发扩展帧时，ID 要写成 8 位:

```bash
cansend can0 00801400#0000000000000000
```

不要写成:

```bash
cansend can0 801400#0000000000000000
```

## 2. 环境准备

每个新终端先执行:

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认 `can-utils` 已安装:

```bash
which candump
which cansend
```

如果没有:

```bash
sudo apt-get install -y can-utils
```

配置 `can0` 为 500K:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

正常应看到:

```text
can0: <NOARP,UP,LOWER_UP,ECHO>
bitrate 500000
```

如果看到 `ERROR-PASSIVE`、`ERROR-WARNING`、`BUS-OFF`，优先检查:

- CAN_H/CAN_L 是否接反。
- 波特率是否 500K。
- 是否有 120 欧终端电阻。
- 对端控制器或 USB-CAN 是否上电并 ACK。
- GND 是否需要共地。

## 3. 本项目如何从 UDP 改成 CAN

原来的控制链路是:

```text
Nav2 / pure_pursuit
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> UDP 发送到下位机
  -> 下位机再转 CAN
```

现在改成:

```text
Nav2 / pure_pursuit
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> SocketCAN can0
  -> 轮椅控制器
```

核心原则:

- 没有改 `/wheelchair_control_command` 话题。
- 没有改 pure pursuit、Nav2、RTAB-Map、建图流程。
- 只改最终执行层 `wheelchair_controller_node`。
- 保留旧 UDP 能力，可通过参数切回 UDP。

本次修改的文件:

```text
src/wheelchair_controller/src/wheelchair_controller_node.cpp
src/wheelchair_controller/config/wheelchair_controller_param.yaml
scripts/keyboard_can_control.py
CAN测试与键盘控制说明.md
```

### 3.1 原 UDP 发送方式

旧节点收到:

```text
data[0] = radius
data[1] = velocity
data[2] = distance
```

然后通过 UDP 发 16 字节:

```text
前 8 字节: radius, big-endian double
后 8 字节: velocity, big-endian double
```

默认 UDP 目标:

```text
10.42.0.1:9999
```

### 3.2 新增输出方式参数

现在节点新增了 `output_transport` 参数:

```yaml
'output_transport': 'can'
```

可选值:

```text
can  直接通过 SocketCAN 发送到底盘
udp  保留旧 UDP 发送方式
```

切回 UDP 示例:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=udp \
  -p target_ip:=10.42.0.1 \
  -p target_port:=9999 \
  -p auto_start:=true
```

### 3.3 C++ 节点内部如何分流

节点启动时读取参数:

```cpp
output_transport_ = this->get_parameter("output_transport").as_string();
```

然后按参数初始化:

```cpp
if (output_transport_ == "udp") {
    InitUdp();
} else if (output_transport_ == "can") {
    InitCan();
}
```

收到 `/wheelchair_control_command` 后仍然读取同样的数据:

```cpp
double radius = msg->data[0];
double velocity = msg->data[1];
double distance = msg->data[2];
```

发送时再分流:

```cpp
if (output_transport_ == "udp") {
    SendUdpMsg(radius, velocity);
} else if (output_transport_ == "can") {
    SendCanFrame(radius, velocity, distance);
}
```

### 3.4 SocketCAN 初始化

CAN 模式使用 Linux SocketCAN raw socket，不依赖 `python-can`:

```cpp
can_sockfd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
ioctl(can_sockfd_, SIOCGIFINDEX, &ifr);
bind(can_sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
```

默认接口:

```yaml
'can_interface': 'can0'
```

CAN 口必须提前配置:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 3.5 CAN 帧如何生成

CAN 参数:

```text
CAN ID : 0x801400
帧类型 : 扩展帧
DLC    : 8
```

代码里会设置扩展帧标志:

```cpp
frame.can_id = can_frame_id_ | CAN_EFF_FLAG;
```

8 字节数据格式:

```text
byte 0-1: left velocity
byte 2-3: right velocity
byte 4-5: left distance
byte 6-7: right distance
```

所有字段都是 `int16 little-endian`。

当前默认不使用 topic 里的 distance:

```yaml
'can_use_command_distance': false
'can_default_distance': 0
```

所以左右距离默认都是 0。

### 3.6 半径和速度转换逻辑

输入仍是:

```text
radius   = data[0]
velocity = data[1]
distance = data[2]
```

先处理半径:

```text
radius == 0             原地转
radius >= 10000         直行
其他非直行半径          默认先取反
abs(radius) < 1000      限幅到最小 1000mm
```

保留旧系统行为的关键参数:

```yaml
'can_invert_radius': true
'can_min_turn_radius_mm': 1000.0
'can_straight_radius_threshold_mm': 10000.0
```

这个 `can_invert_radius: true` 是为了保留旧 UDP 下位机里“非直行半径先取反”的行为，否则左右转方向会和旧系统不一致。

左右轮速度计算:

```text
velocity == 0:
  left = 0
  right = 0

radius == 0:
  left = velocity
  right = -velocity

radius >= 10000:
  left = -velocity
  right = -velocity

其他转弯:
  left  = -int((radius + 300) * velocity / radius + 0.5)
  right = -int((radius - 300) * velocity / radius + 0.5)
```

最后限幅:

```yaml
'can_velocity_limit': 16380
```

### 3.7 保持发送和超时停车

旧 UDP 逻辑是收到一次 topic 就发送一次 UDP。

新 CAN 逻辑是:

1. 收到 `/wheelchair_control_command` 后立即发送一次 CAN。
2. 定时器按 `can_send_period_ms` 继续发送最近一条命令。
3. 超过 `command_timeout_ms` 没有新命令后，发送停止帧。

默认:

```yaml
'can_send_period_ms': 20.0
'command_timeout_ms': 500.0
```

所以执行一次:

```bash
ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [-100.0, 1000.0, 0.0]}"
```

`candump` 看到多条 CAN 帧是正常现象。

### 3.8 编译方式

改完后编译:

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
set +u
source /opt/ros/humble/setup.bash
source scripts/use_rtabmap_0234_env.sh
colcon build --packages-select wheelchair_controller --symlink-install
source install/setup.bash
```

确认节点存在:

```bash
ros2 pkg executables wheelchair_controller
```

应看到:

```text
wheelchair_controller pure_pursuit_controller_node
wheelchair_controller wheelchair_controller_node
```

## 4. 手动 CAN 测试

终端 A 监听 CAN:

```bash
candump -L can0
```

终端 B 发送停止帧:

```bash
cansend can0 00801400#0000000000000000
```

发送一个很小的双轮同向测试帧:

```bash
cansend can0 00801400#9CFF9CFF00000000
```

`9CFF` 是 int16 little-endian 的 `-100`。如果终端 A 能看到 `00801400#...`，说明 Jetson 的 CAN 发帧链路可用。

## 5. ROS 控制节点测试

当前 ROS 执行链路:

```text
/wheelchair_control_command
  -> wheelchair_controller_node
  -> SocketCAN can0
  -> 轮椅控制器
```

`/wheelchair_control_command` 类型:

```text
std_msgs/msg/Float32MultiArray
```

数组含义:

```text
data[0] = radius_mm
data[1] = velocity_mm_s
data[2] = distance_mm, 当前默认不用于 CAN 距离
```

启动 CAN 控制节点:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

另一个终端监听:

```bash
candump -L can0
```

发送停止命令:

```bash
ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0]}"
```

发送低速转弯测试:

```bash
ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [-100.0, 1000.0, 0.0]}"
```

节点会按 `can_send_period_ms` 保持发送最近一条命令，超过 `command_timeout_ms` 没有新命令后发送停止帧。因此执行一次 `ros2 topic pub --once` 后，`candump` 看到多帧是正常现象。

默认配置文件:

```text
src/wheelchair_controller/config/wheelchair_controller_param.yaml
```

关键参数:

```yaml
'output_transport': 'can'
'can_interface': 'can0'
'can_frame_id': 8393728
'can_velocity_limit': 16380
'can_send_period_ms': 20.0
'command_timeout_ms': 500.0
```

改完配置后重新编译:

```bash
colcon build --packages-select wheelchair_controller --symlink-install
source install/setup.bash
```

## 6. 键盘直接控制 CAN

键盘控制脚本:

```text
scripts/keyboard_can_control.py
```

它不经过 ROS，不启动 `wheelchair_controller_node`，直接通过 SocketCAN 发送 `0x801400` 扩展帧。

启动:

```bash
cd /home/dog/fastlio_rtabmap_ros2_ws
./scripts/keyboard_can_control.py --interface can0 --can-id 0x801400 --period-ms 20
```

按键:

```text
W  前进
S  后退
A  慢左转
D  慢右转
Q  快左转
E  快右转
Z  原地左转
C  原地右转
X  停止
空格 停止
P  停止
H  显示帮助
Esc 或 Ctrl+C 退出
```

默认是 deadman 模式，松开运动按键超过 `--hold-timeout-ms` 会自动停车。

如果想恢复“按一次后持续运动，直到按停止键”的模式:

```bash
./scripts/keyboard_can_control.py --latch
```

## 7. 修改键盘控制速度

键盘脚本的速度都可以通过启动参数修改，不需要改代码。

默认值:

```text
--forward-vel       1500   W 前进速度
--backward-vel     -1000   S 后退速度，通常为负数
--turn-vel          1000   A/D 慢速弧线转弯速度
--fast-turn-vel     1500   Q/E 快速弧线转弯速度
--spin-vel           700   Z/C 原地转向速度绝对值
--turn-radius       1800   A/D 转弯半径，单位 mm，越小转得越急
--fast-turn-radius  3000   Q/E 转弯半径，单位 mm，越大转得越缓
--straight-radius  10000   直行半径阈值
```

低速安全测试:

```bash
./scripts/keyboard_can_control.py \
  --forward-vel 600 \
  --backward-vel -400 \
  --turn-vel 500 \
  --fast-turn-vel 700 \
  --spin-vel 300 \
  --turn-radius 2500 \
  --fast-turn-radius 4000
```

提高前进速度:

```bash
./scripts/keyboard_can_control.py --forward-vel 2000
```

降低后退速度:

```bash
./scripts/keyboard_can_control.py --backward-vel -500
```

降低普通转弯速度:

```bash
./scripts/keyboard_can_control.py --turn-vel 600
```

降低原地转向速度:

```bash
./scripts/keyboard_can_control.py --spin-vel 400
```

让 A/D 转弯更急:

```bash
./scripts/keyboard_can_control.py --turn-radius 1200
```

让 A/D 转弯更缓:

```bash
./scripts/keyboard_can_control.py --turn-radius 3000
```

组合示例:

```bash
./scripts/keyboard_can_control.py \
  --interface can0 \
  --can-id 0x801400 \
  --period-ms 20 \
  --forward-vel 1200 \
  --backward-vel -800 \
  --turn-vel 800 \
  --fast-turn-vel 1200 \
  --spin-vel 500 \
  --turn-radius 2000 \
  --fast-turn-radius 3500
```

速度方向说明:

- `--forward-vel` 一般为正数。
- `--backward-vel` 一般为负数。
- `--spin-vel` 填正数，脚本内部会把 `Z` 转成负速度、`C` 转成正速度。
- `A` 使用正 `turn_radius`。
- `D` 使用负 `turn_radius`。
- `Q` 使用正 `fast_turn_radius`。
- `E` 使用负 `fast_turn_radius`。

## 8. 修改脚本默认速度

如果希望以后不带参数也使用新的默认速度，修改:

```text
scripts/keyboard_can_control.py
```

找到这些行:

```python
parser.add_argument("--forward-vel", type=int, default=1500, help="W velocity, lower is safer")
parser.add_argument("--backward-vel", type=int, default=-1000, help="S velocity, should normally be negative")
parser.add_argument("--turn-vel", type=int, default=1000, help="A/D arc-turn velocity")
parser.add_argument("--fast-turn-vel", type=int, default=1500, help="Q/E larger-radius turn velocity")
parser.add_argument("--spin-vel", type=int, default=700, help="Z/C in-place spin velocity magnitude")
parser.add_argument("--turn-radius", type=int, default=1800, help="A/D turn radius in mm; smaller is sharper")
parser.add_argument("--fast-turn-radius", type=int, default=3000, help="Q/E turn radius in mm; larger is gentler")
```

只改 `default=` 后面的数字即可。

修改后检查脚本语法:

```bash
python3 -m py_compile scripts/keyboard_can_control.py
```

## 9. 导航时如何控制 CAN

导航时不需要运行键盘脚本。导航链路是:

```text
/plan
  -> pure_pursuit_controller_node
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> can0
```

单独启动底盘 CAN 控制节点:

```bash
ros2 launch wheelchair_controller wheelchair_controller.launch.py
```

如果配置里的 `auto_start: false`，需要在 `wheelchair_controller_node` 所在终端按一次空格开始控制。测试时也可以临时覆盖:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

确认导航是否真的发控制命令:

```bash
ros2 topic echo /wheelchair_control_command
```

确认 CAN 是否真的输出:

```bash
candump -L can0
```

## 10. 常见问题

`candump` 没有任何帧:

- 确认 `can0` 已经 `UP`。
- 确认正在运行键盘脚本或 `wheelchair_controller_node`。
- 确认 ROS 侧真的有 `/wheelchair_control_command`。

`candump` 有帧但真机不动:

- 确认对端使用扩展帧 `0x801400`。
- 确认波特率 500K。
- 确认控制器处于使能状态。
- 确认速度没有太小。
- 检查 `ip -details link show can0` 是否有 `ERROR-PASSIVE` 或 `BUS-OFF`。

方向反了:

- 先不要改导航。
- 键盘脚本可先用低速确认每个按键方向。
- ROS 节点中 `can_invert_radius: true` 是为了保留旧 UDP 到 CAN 方案的左右转方向。如果改成 `false`，导航转向方向会变化。
