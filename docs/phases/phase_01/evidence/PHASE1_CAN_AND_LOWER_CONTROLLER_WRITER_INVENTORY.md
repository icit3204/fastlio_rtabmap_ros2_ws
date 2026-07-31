# Phase 1 CAN and Lower-Controller Writer Inventory

Date: 2026-07-31
Machine: Jetson 2

## Summary

| Item | Count |
|---|---|
| Total CAN writers identified | 1 |
| UDP command writers identified | 1 (alternative transport in same node) |
| Other lower-controller writers | 0 |
| Direct hardware writers (not via ROS) | 0 |

---

## Writer 1: wheelchair_controller_node

### Identification

| Attribute | Value |
|---|---|
| Package | `wheelchair_controller` |
| Source file | `src/wheelchair_controller/src/wheelchair_controller_node.cpp` |
| Executable | `wheelchair_controller_node` |
| Class | `WheelchairController` |

### Transport configuration

| Attribute | Value |
|---|---|
| Primary transport | SocketCAN (`PF_CAN`, `SOCK_RAW`) |
| CAN interface | `can0` (configurable via `can_interface` parameter) |
| CAN frame ID | `0x801400` (configurable via `can_frame_id` parameter, uses `CAN_EFF_FLAG`) |
| Alternative transport | UDP (`AF_INET`, `SOCK_DGRAM`) |
| Transport selection | `output_transport` parameter (`"can"` or `"udp"`) |
| Watchdog | 20ms timer (`can_send_period_ms`) — sends repeated zero when no command |

### CAN message format

```
Radius (mm), velocity (mm/s) → differential wheel speeds (left/right)
+ distance command (optional, can_use_command_distance)
```

### Input

| Attribute | Value |
|---|---|
| Input topic | `/wheelchair_control_command` |
| Message type | `std_msgs/Float32MultiArray` |
| Expected format | [radius_mm, velocity_mm_per_s] or [radius_mm, velocity_mm_per_s, distance_mm] |

### Auto-start / default state

| Attribute | Value |
|---|---|
| Launched by any bringup | NO — must be started manually |
| Default state | Requires explicit `ros2 run wheelchair_controller wheelchair_controller_node` |
| Control paused on start | YES (`control_paused_ = true`) — requires keyboard 'c' to arm |

### Watchdog behavior

- Sends repeated zero-velocity CAN frame every 20ms when no command received
- `control_paused_` state blocks all CAN output

### Authority

| Attribute | Value |
|---|---|
| Authoritative CAN writer | YES — sole writer to `can0` for wheelchair control |
| Legacy | YES — part of legacy plan_nav/Pure Pursuit chain |
| Fallback | UDP transport available as alternative |
| Unused | CAN transport is the default (`output_transport: "can"`) |

### Distinction: ROS subscriber vs CAN writer

`wheelchair_controller_node` is:
- A ROS **SUBSCRIBER** to `/wheelchair_control_command`
- A CAN **WRITER** to `can0` (SocketCAN)

It does NOT publish any ROS command topic.

---

## Search completeness

### Topics searched

- `can0`, `CAN`, `SocketCAN`, `socketcan`, `can_interface`, `can_socket`, `cansend`
- `PF_CAN`, `AF_CAN`, `SOCK_RAW`, `CAN_RAW`, `CAN_EFF_FLAG`
- `can_frame_id`, `can_send`, `can_write`, `can_bind`, `can_sockfd`
- `output_transport`, `udp_sockfd`, `sendto`

### Directories searched

- `src/wheelchair_controller/`
- `src/robot_bringup/`
- `src/FAST_LIO_ROS2/`
- `src/livox_ros_driver2/`
- `src/rtabmap_ros/`
- `src/ydlidar_ros2_driver/`
- `src/YDLidar-SDK/`
- `plan_nav/`

### Excluded from CAN writer scope

- YDLIDAR `MAX_SCAN_NODES` constants (unrelated to CAN bus)
- RapidJSON `_SIDD_LEAST_SIGNIFICANT` (unrelated)
- RTAB-Map `DbPlayerNode` `ICANON` (terminal input flag, not CAN bus)
- RTAB-Map `CANCELED` action result (ROS action, not CAN bus)

---

## Conclusion

One CAN writer exists: `wheelchair_controller_node`. It is a subscriber to `/wheelchair_control_command` and is NOT auto-launched by any bringup. It supports both SocketCAN (can0, CAN ID 0x801400) and UDP transports. It is the sole physical authority for wheelchair motion through the lower controller. No other source can write to CAN or any lower-controller transport.
