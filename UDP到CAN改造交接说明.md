# UDP 到 CAN 改造交接说明

本文档给接手 Agent 使用。目标是从“旧 UDP 项目”重新开始时，能直接复现当前项目的 CAN 直连控制能力。

当前项目路径:

```bash
cd /home/dog/7.11backup
```

参考 CAN 原型代码:

```text
/home/dog/Downloads/Can2026/canControl.py
/home/dog/Downloads/Can2026/client_UDP.py
```

## 1. 原 UDP 链路

旧链路是:

```text
Nav2 / pure_pursuit
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> UDP 发送到下位机
  -> 下位机再转 CAN 控制轮椅
```

`/wheelchair_control_command` 类型:

```text
std_msgs/msg/Float32MultiArray
```

数组含义:

```text
data[0] = radius    # 转弯半径, mm
data[1] = velocity  # 速度
data[2] = distance  # 距离, 当前 CAN 版默认不使用
```

旧 UDP 发送格式在 `wheelchair_controller_node.cpp` 的 `SendUdpMsg()` 中:

```text
16 字节 UDP payload
前 8 字节: radius, big-endian double
后 8 字节: velocity, big-endian double
默认目标: 10.42.0.1:9999
```

旧下位机逻辑可参考 `/home/dog/Downloads/Can2026/client_UDP.py` 的 `canSendRadiusThre()`:

1. UDP 收到 `radius, velocity`。
2. 若 `radius != 10000`，先对半径取反。
3. 非直行半径限幅到至少 `1000 mm`。
4. 调用 `canMotor.cansend(velocity, radius, distance, 0.1, canMotor.bus)`。

这个“半径取反 + 最小转弯半径 1000mm”的行为必须保留，否则左右转方向会和旧系统不一致。

## 2. 当前 CAN 目标链路

改造后的链路是:

```text
Nav2 / pure_pursuit
  -> /wheelchair_control_command
  -> wheelchair_controller_node
  -> SocketCAN can0
  -> 轮椅控制器
```

核心原则:

- 不改变 `/wheelchair_control_command` 的消息接口。
- 保留 UDP 发送能力作为回退选项。
- 增加 `output_transport` 参数，默认设为 `can`。
- C++ 节点直接使用 Linux SocketCAN raw socket，不依赖 `python-can`。

## 3. 必须迁移的文件

从当前项目复制或按当前项目修改以下文件:

```text
src/wheelchair_controller/src/wheelchair_controller_node.cpp
src/wheelchair_controller/config/wheelchair_controller_param.yaml
src/wheelchair_controller/launch/wheelchair_controller.launch.py
scripts/keyboard_can_control.py
```

`CMakeLists.txt` 和 `package.xml` 通常不需要额外 CAN 依赖，因为 SocketCAN 使用系统头文件:

```cpp
#include <linux/can.h>
#include <linux/can/raw.h>
```

但要确保 `wheelchair_controller_node` 仍被编译安装。

## 4. wheelchair_controller_node.cpp 改造点

### 4.1 新增参数

在节点构造函数中声明并读取这些参数:

```yaml
output_transport: can
can_interface: can0
can_frame_id: 8393728          # 0x801400
can_velocity_limit: 16380
can_wheel_half_track_mm: 300.0
can_straight_radius_threshold_mm: 10000.0
can_min_turn_radius_mm: 1000.0
can_invert_radius: true
can_send_period_ms: 20.0
command_timeout_ms: 500.0
can_use_command_distance: false
can_default_distance: 0
```

保留旧 UDP 参数:

```yaml
target_ip: 10.42.0.1
target_port: 9999
```

### 4.2 初始化分流

构造函数按 `output_transport` 分流:

```cpp
if (output_transport_ == "udp") {
    InitUdp();
} else if (output_transport_ == "can") {
    InitCan();
}
```

### 4.3 SocketCAN 初始化

`InitCan()` 需要做这些事:

```cpp
can_sockfd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
ioctl(can_sockfd_, SIOCGIFINDEX, &ifr);
bind(can_sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
```

接口默认 `can0`，要求系统已提前配置为 500K。

### 4.4 CAN 保持发送和超时停车

当前实现不是只在收到 ROS topic 时发一次，而是:

1. 收到 `/wheelchair_control_command` 后立即发一次 CAN。
2. `can_send_period_ms` 定时器继续保持发送最近一条指令。
3. 超过 `command_timeout_ms` 没有新指令后，发送一次停止帧。

因此执行一次:

```bash
ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [-100.0, 1000.0, 0.0]}"
```

接收端看到多条 CAN 帧是正常现象。当前 `20ms` 周期和 `500ms` 超时下，通常会看到约 25 到 27 条。

## 5. CAN 帧协议

CAN 参数:

```text
interface : can0
bitrate   : 500000
CAN ID    : 0x801400
frame     : extended frame
DLC       : 8
```

8 字节 payload:

```text
byte 0-1: left velocity,  int16 little-endian
byte 2-3: right velocity, int16 little-endian
byte 4-5: left distance,  int16 little-endian
byte 6-7: right distance, int16 little-endian
```

当前默认:

```text
left distance  = 0
right distance = 0
```

对应 `canControl.py`:

```python
sendData = speed_byte1 + speed_byte2 + dis_byte1 + dis_byte2
msg = can.Message(arbitration_id=self.frameID, data=sendData, is_extended_id=True)
```

`canControl.py` 的默认 ID 计算:

```python
sendID=0x01
recvID=0x0
ctlInfo=0x02
endFlag=0x1
index=0x0

frameID = sendID<<23 | recvID<<17 | ctlInfo<<11 | endFlag<<10 | index
        = 0x801400
```

用 `can-utils` 手动发扩展帧时，ID 必须写满 8 位:

```bash
cansend can0 00801400#0000000000000000
```

不要写成:

```bash
cansend can0 801400#0000000000000000
```

否则会报 `Wrong CAN-frame format!`。

## 6. 左右轮速度计算

从 ROS topic 读取:

```text
radius   = data[0]
velocity = data[1]
distance = data[2]
```

先处理半径:

```text
如果 abs(radius) < 很小阈值: 原地转
如果 radius >= 10000: 直行
否则:
  如果 can_invert_radius=true，radius = -radius
  若 0 < radius < 1000，改为 1000
  若 -1000 < radius < 0，改为 -1000
```

再计算左右轮:

```text
vel == 0:
  left = 0
  right = 0

radius == 0:
  left = vel
  right = -vel

radius >= 10000:
  left = -vel
  right = -vel

其他转弯:
  left  = -int((radius + 300) * vel / radius + 0.5)
  right = -int((radius - 300) * vel / radius + 0.5)
```

最后把左右轮速度限幅到:

```text
[-16380, 16380]
```

这一逻辑必须与 `/home/dog/Downloads/Can2026/canControl.py` 的 `cansend()` 保持一致。

## 7. 配置文件默认值

`src/wheelchair_controller/config/wheelchair_controller_param.yaml` 应改为:

```yaml
wheelchair_controller_node:
  ros__parameters:
    'output_transport': 'can'
    'target_ip': '10.42.0.1'
    'target_port': 9999
    'auto_start': false
    'min_command_interval_ms': 20.0
    'command_timeout_ms': 500.0

    'can_interface': 'can0'
    'can_frame_id': 8393728
    'can_velocity_limit': 16380
    'can_wheel_half_track_mm': 300.0
    'can_straight_radius_threshold_mm': 10000.0
    'can_min_turn_radius_mm': 1000.0
    'can_invert_radius': true
    'can_send_period_ms': 20.0
    'can_use_command_distance': false
    'can_default_distance': 0
```

`auto_start: false` 时，需要在 `wheelchair_controller_node` 所在终端按一次空格才开始控制。测试时可临时用:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

## 8. 键盘 CAN 控制脚本

当前项目新增:

```text
scripts/keyboard_can_control.py
```

用途:

- 不经过 ROS。
- 不启动 `wheelchair_controller_node`。
- 直接打开 SocketCAN `can0`。
- 发送扩展帧 `0x801400`。
- 用来验证 Jetson CAN 口、物理接线、Windows USB-CAN 工具和真机底盘。

默认按键:

```text
W  前进
S  后退
A  慢左转
D  慢右转
Q  快左转
E  快右转
Z  原地左转
C  原地右转
X/Space/P 停止
Esc/Ctrl+C 退出
```

启动:

```bash
cd /home/dog/7.11backup
chmod +x scripts/keyboard_can_control.py
./scripts/keyboard_can_control.py --interface can0 --can-id 0x801400 --period-ms 20
```

脚本默认是 deadman 模式，松开按键超过 `--hold-timeout-ms` 会自动停车。若要恢复旧的“按一次持续运动”行为，使用:

```bash
./scripts/keyboard_can_control.py --latch
```

## 9. pure_pursuit 与导航链路

`pure_pursuit_controller_node` 发布:

```text
/wheelchair_control_command
```

其输出仍是:

```text
data[0] = R * 1000.0
data[1] = base_vel * 1000.0
data[2] = dist_to_lookahead * 1000.0
```

CAN 改造不要求改 `/plan`、Nav2、RTAB-Map 或 pure pursuit 的话题接口。只要旧项目已有:

```text
/plan -> pure_pursuit_controller_node -> /wheelchair_control_command
```

接手 Agent 只需要把最终执行层从 UDP 发送替换为 CAN 发送。

当前导航启动文件中，`bringup_2d.launch.py` 和 `bringup_2d_infra.launch.py` 会启动 `pure_pursuit_controller_node`，但底盘 CAN 控制节点通常单独启动:

```bash
ros2 launch wheelchair_controller wheelchair_controller.launch.py
```

或测试时:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

## 10. 构建

```bash
cd /home/dog/7.11backup
source /opt/ros/humble/setup.bash
colcon build --packages-select wheelchair_controller --symlink-install
source install/setup.bash
```

确认可执行文件:

```bash
ros2 pkg executables wheelchair_controller
```

应包含:

```text
wheelchair_controller wheelchair_controller_node
wheelchair_controller pure_pursuit_controller_node
```

## 11. CAN 环境准备

安装调试工具:

```bash
sudo apt-get install -y can-utils
```

启动 `can0`:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

检查:

```bash
ip -details link show can0
```

正常应看到:

```text
can0: <NOARP,UP,LOWER_UP,ECHO>
bitrate 500000
```

如果出现 `ERROR-WARNING` 或 `BUS-OFF`，优先检查:

- CAN_H/CAN_L 是否接反。
- 波特率是否 500K。
- 总线是否有 120 欧终端电阻。
- 对端是否在线并 ACK。

## 12. 最小测试用例

### 12.1 手动 CAN 发帧

```bash
candump -L can0
```

另一个终端:

```bash
cansend can0 00801400#0000000000000000
cansend can0 00801400#9CFF9CFF00000000
```

### 12.2 ROS topic 触发项目节点发 CAN

终端 A:

```bash
cd /home/dog/7.11backup
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

终端 B:

```bash
candump -L can0
```

终端 C:

```bash
cd /home/dog/7.11backup
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [-100.0, 1000.0, 0.0]}"
```

这条指令等价于 `canControl.py` 中:

```python
canMotor.cansend(1000, 100, 0, 0.1, canMotor.bus)
```

原因:

- ROS topic 顺序是 `[radius, velocity, distance]`。
- `canControl.py` 函数顺序是 `(vel, radius, dis, ...)`。
- 当前 C++ 节点默认 `can_invert_radius=true`，所以 `radius=-100` 会先变成 `100`。

停止:

```bash
ros2 topic pub --once /wheelchair_control_command std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0]}"
```

### 12.3 键盘控制测试

```bash
cd /home/dog/7.11backup
./scripts/keyboard_can_control.py
```

同时用:

```bash
candump -L can0
```

确认持续出现:

```text
00801400#...
```

### 12.4 导航触发 CAN

只播放 bag 不会自动发 CAN。必须有导航目标，链路才会产生:

```text
/plan -> pure_pursuit_controller_node -> /wheelchair_control_command -> wheelchair_controller_node -> can0
```

检查关键话题:

```bash
ros2 topic echo /wheelchair_control_command
candump -L can0
```

## 13. Windows USB-CAN 验证

Windows 端工具:

```text
/home/dog/Downloads/can_lin_tool_V3.0/CAN_LIN_Tool_V3.0.exe
```

接线:

```text
Jetson can0 CAN_H <-> USB-CAN CAN_H
Jetson can0 CAN_L <-> USB-CAN CAN_L
GND 建议共地
```

Windows 工具设置:

```text
bitrate: 500K
frame type: Extended
filter: 先不过滤，或过滤 0x801400
```

注意: 蓝色 USB-CAN/CAN-LIN 工具插在 Jetson 上时枚举为 `/dev/ttyUSB0`，不是 SocketCAN 设备，不能直接 `candump can1`。当前项目默认使用 Jetson 本机 `can0`。

## 14. 常见问题

1. Windows 能看到 `Can2026/canControl.py`，但看不到项目节点。

检查项目节点是否启动且参数正确:

```bash
ros2 run wheelchair_controller wheelchair_controller_node --ros-args \
  -p output_transport:=can \
  -p can_interface:=can0 \
  -p auto_start:=true
```

再检查是否真的有控制话题:

```bash
ros2 topic echo /wheelchair_control_command --once
```

2. 执行一次 ROS topic pub 后接收端看到多帧。

这是正常的。节点会按 `can_send_period_ms` 保持发送，直到 `command_timeout_ms` 超时后停车。

3. 轮椅按键控制后持续转动。

使用当前 `scripts/keyboard_can_control.py` 默认 deadman 模式，松开按键会自动停止。如果使用了 `--latch`，则会保持运动，必须按 `X`/空格停车。

4. 真机不动但 `candump` 有帧。

检查:

- 帧 ID 是否为扩展帧 `0x801400`。
- Windows/控制器是否配置为 500K。
- 轮椅控制器是否要求使能或安全状态。
- 左右轮速度是否过小。
- CAN 总线是否有 ACK，`ip -details link show can0` 是否报错。

## 15. 当前已验证功能

已验证过的功能包括:

- `wheelchair_controller_node` 从 `/wheelchair_control_command` 转换并发送 CAN。
- `scripts/keyboard_can_control.py` 直接键盘控制 CAN。
- 播放数据集并设置导航目标后，导航链路可以产生 `/wheelchair_control_command`，并触发 CAN 输出。
- Windows USB-CAN 工具可作为接收端观察 Jetson 发出的 `0x801400` 扩展帧。

接手 Agent 优先按本文档迁移 `wheelchair_controller_node.cpp` 和参数文件，不要先改导航、建图、RTAB-Map 或 TF。
