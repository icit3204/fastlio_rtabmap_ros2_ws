# Phase 2 Minimum Implementation Plan

Date: 2026-08-01
Phase: 2 — Core Simple Nav2 Navigation Baseline

## Stage overview

| Stage | Description | Files | Validation | Pass gate |
|---|---|---|---|---|
| P2-A | Create isolated package and configuration | 6 new | Build + parse check | Package builds; launch parses without node execution |
| P2-B | Static launch inspection | 0 changes | Launch parse + topic list | No hardware node auto-starts; all topics are Phase 2 scope |
| P2-C | Map + fake TF/odom + Nav2, no goal | 0 changes | Runtime check | /map, /Odometry, TF published; Nav2 lifecycle active |
| P2-D | One simple fake closed-loop goal | 1 new (test runner) | Goal result | NavigateToPose SUCCEEDED; sensible path and velocity |
| P2-E | Sequential goals and cancellation | 0 changes | Goal sequence | 3 goals succeed; cancel returns CANCELLED |
| P2-F | Failure tests | 0 changes | Failure results | Planner/controller/TF failures abort cleanly; zero velocity |
| P2-G | Repetition, metrics, freeze | 0 changes | Reproducibility | 5/5 closed-loop successes; metrics recorded |

---

## P2-A: Create isolated package and configuration

### Files to create

**`parking_robot_bringup/CMakeLists.txt`**
```cmake
cmake_minimum_required(VERSION 3.8)
project(parking_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config maps scripts
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/phase2_goal_test_runner.py scripts/phase2_fake_base.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
```

**`parking_robot_bringup/package.xml`**
```xml
<package format="3">
  <name>parking_robot_bringup</name>
  <version>0.1.0</version>
  <description>Phase 2 isolated Nav2 navigation baseline</description>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

**`parking_robot_bringup/config/phase2_nav2_params.yaml`**
Copy from `nav2_common.yaml` with modifications:
- Remove all observation sources from costmaps
- Set `cmd_vel_out_topic: /cmd_vel_phase2_mock` (or remap in launch)
- Disable inflation layer (optional, for pure-static operation)
- Keep NavFn A* + MPPI configuration

**`parking_robot_bringup/launch/phase2_core_nav2.launch.py`**
Minimal launch:
1. `map_server` with `clean_map.yaml`
2. `static_transform_publisher map → odom` (identity)
3. `phase2_fake_base` node (odom_from_cmd_vel wrapper)
4. Nav2 bringup (navigation_launch.py with phase2 params)
5. Nav2 lifecycle manager
6. RViz (optional, conditioned on launch arg)

**`parking_robot_bringup/maps/`**
Symbolic links to `scripts/offline_nav_maps/clean_map.yaml` and
`clean_map.pgm`.

### Validation
- `colcon build --packages-select parking_robot_bringup` succeeds
- `ros2 launch parking_robot_bringup phase2_core_nav2.launch.py` parses
  without error (launch file syntax check)

### Rollback
Delete `parking_robot_bringup/` directory.

---

## P2-B: Static launch inspection

### Checks
- Parse launch with `ros2 launch --show-arguments`
- Verify no `wheelchair_controller`, `pure_pursuit`, `collision_monitor`,
  `plan_nav`, `livox`, `fast_lio` nodes present
- Verify no sensor drivers (camera, YDLIDAR, MID-360)
- Verify `/cmd_vel_phase2_mock` is the only command topic
- Verify no remapping to `/cmd_vel`, `/cmd_vel_nav`, `/wheelchair_control_command`

### Validation
- All nodes in launch are Phase 2 scope
- No excluded node, topic, or parameter present

### Pass gate
Static inspection passes. No node execution required.

---

## P2-C: Map + fake TF/odom + Nav2, no goal

### Runtime test
```bash
env -i HOME=/home/dog ... \
  ros2 launch parking_robot_bringup phase2_core_nav2.launch.py
```

### Checks
- `/map` published by map_server (check with `ros2 topic echo /map --once`)
- `/Odometry` published by fake_base (check stamp, frame_id)
- `map → odom` TF exists (static identity)
- `odom → base_footprint` TF exists (dynamic from integration)
- Nav2 lifecycle nodes active (`ros2 lifecycle list`)
- No crash, no missing library, no infinite wait for sensor

### Pass gate
All topics, TFs, and lifecycle nodes confirmed active. Robot at origin
with zero velocity.

---

## P2-D: One simple fake closed-loop goal

### Test runner (phase2_goal_test_runner.py)
```python
# Key behavior:
# 1. Wait for navigate_to_pose action server
# 2. Publish /initialpose at (0, 0, 0)
# 3. Send NavigateToPose goal to (5.0, 0.0, 0.0) in map frame
# 4. Wait for result
# 5. Log SUCCEEDED/ABORTED, elapsed time, final pose
# 6. Exit
```

### Checks
- Action result: SUCCEEDED
- Final pose within 1.0m of goal (xy_tolerance)
- `/Odometry` shows velocity history consistent with path
- `/cmd_vel_phase2_mock` shows reasonable MPPI output (non-zero, decaying near goal)
- No oscillation or NaN in velocity

### Pass gate
Goal SUCCEEDED; path and velocity sensible.

---

## P2-E: Sequential goals and cancellation

### Sequential test
Send 3 goals: (5,0), (5,5), (0,5). Each must succeed before next.

### Cancellation test
Start goal to (10,0). After 3 seconds, send cancel. Verify:
- Action result: CANCELLED (code 5)
- `/cmd_vel_phase2_mock` goes to zero within 0.5s
- Fake robot velocity decays to zero

### Pass gate
All 3 sequential goals succeed in order. Cancel produces CANCELLED with
timely zero-velocity.

---

## P2-F: Failure tests

### Planner failure
Send goal to coordinates inside known occupied region (or beyond map
boundary at x=200). Verify:
- Action result: ABORTED
- `/cmd_vel_phase2_mock` is zero

### Controller timeout
Use slow MPPI settings or obstructed goal. Verify:
- Action result: ABORTED after timeout
- No continued nonzero command

### TF failure
Send goal without odom running (pause fake_base). Verify:
- Action rejected or times out
- No command continuation

### Pass gate
All failures produce clean abort. Zero velocity after failure.

---

## P2-G: Repetition, metrics, documentation

### Repeated closed-loop
Run P2-D (simple goal) 5 times consecutively. Record:
- Success count (expect 5/5)
- Elapsed time per run
- Final pose error per run
- Velocity profile consistency

### Documentation
Create `PHASE2_CORE_NAV2_BASELINE_RESULT.md`

### Pass gate
5/5 successes. Metrics within tolerance. Documentation frozen.

---

## Environment and isolation

All Phase 2 runtime must use:
```bash
ROS_LOCALHOST_ONLY=1
ROS_DOMAIN_ID=195  # Dedicated Phase 2 domain
use_sim_time=false  # Wall-clock deterministic integration
```

No sourced legacy runtime. No `can0` configuration. No sensor drivers.

---

## Total new code estimate

| File | Lines (est.) |
|---|---|
| `CMakeLists.txt` + `package.xml` | ~30 |
| `phase2_core_nav2.launch.py` | ~80 |
| `phase2_nav2_params.yaml` | ~100 (copy + edit) |
| `phase2_goal_test_runner.py` | ~150 |
| `phase2_fake_base.py` | ~50 |

**Total**: ~410 lines, mostly configuration and adaptation of existing
patterns. No new algorithmic code required.
