# Phase 2 P2-C No-Goal Runtime Result

Date: 2026-08-01  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Final decision

`PHASE2_P2C_NO_GOAL_RUNTIME_NEEDS_REVIEW`

P2-C did not pass. No source repair was performed in this task. No Phase 2 documentation/evidence commit was created.

## Source revision

| Item | Value |
|---|---|
| Branch | `main` |
| HEAD | `a88b22689fc6da4a1d2f16f23c3336d4aa572356` |
| `origin/main` before runtime | `a88b22689fc6da4a1d2f16f23c3336d4aa572356` |
| Working tree before runtime | clean |

Tags were verified before runtime and were not modified:

- `phase0_verified_baseline`: `ae7517e3fe981bfb3fa148bb48ecc206cf1cdaa0`
- `phase1_native_build_verified` object: `a1dd9aac316d5981aee13904002dfeb6b4052894`
- `phase1_native_build_verified^{}`: `7556f15fb0b8e893ac799330bbf26a62ae19e439`
- `phase1_authority_closed` object: `0a62598884351c0ad3bbe5b4264c62c55c4e8cc0`
- `phase1_authority_closed^{}`: `2dc53a117ead6bac5802bdd318b64337ebada930`

## Runtime environment

Runtime directory:

`/home/dog/phase2_runtime/p2c_no_goal_20260801_030415`

Environment:

- `ROS_LOCALHOST_ONLY=1`
- `ROS_DOMAIN_ID=195`
- `ROS_LOG_DIR=/home/dog/phase2_runtime/p2c_no_goal_20260801_030415/ros_log`
- sourced `/opt/ros/humble/setup.bash`
- sourced `/home/dog/phase2_builds/p2ab_scaffolding_validation_20260801_025103/install/setup.bash`

The legacy workspace install was not sourced.

## Launch command

```bash
ros2 launch parking_robot_bringup phase2_core_nav2.launch.py \
  use_sim_time:=false \
  use_rviz:=false \
  autostart:=true \
  initial_x:=5.425 \
  initial_y:=-53.725 \
  initial_yaw:=0.0
```

No `/initialpose` was published. No NavigateToPose goal was sent.

## Validated install check

Before runtime, the P2-A/P2-B install existed at:

`/home/dog/phase2_builds/p2ab_scaffolding_validation_20260801_025103/install`

Installed launch/config/map/Python file checksums matched the current source revision. The installed map YAML resolved to:

`/home/dog/phase2_builds/p2ab_scaffolding_validation_20260801_025103/install/share/parking_robot_bringup/maps/phase2_clean_map.pgm`

## Node set

Runtime nodes observed:

- `/behavior_server`
- `/bt_navigator`
- `/bt_navigator_navigate_through_poses_rclcpp_node`
- `/bt_navigator_navigate_to_pose_rclcpp_node`
- `/controller_server`
- `/global_costmap/global_costmap`
- `/lifecycle_manager_navigation`
- `/local_costmap/local_costmap`
- `/map_server`
- `/phase2_fake_base`
- `/phase2_map_to_odom_static_tf`
- `/planner_server`
- `/transform_listener_impl_*` internal transform listeners

No excluded legacy/hardware nodes were observed.

See:

`/home/dog/phase2_reports/PHASE2_P2C_RUNTIME_NODE_TOPIC_TF_MATRIX.tsv`

## Lifecycle state

Observed active:

- `/map_server`
- `/planner_server`
- `/controller_server`
- `/behavior_server`

Observed inactive:

- `/bt_navigator`

Blocking evidence:

```text
[bt_navigator]: Exception when loading BT: Error at line 12: -> Node not recognized: RemovePassedGoals
[bt_navigator]: Error loading XML file: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
[lifecycle_manager_navigation]: Failed to change state for node: bt_navigator
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

This fails the P2-C requirement that all required lifecycle nodes, including `bt_navigator`, become active.

## Map verification

`/map`:

- type: `nav_msgs/msg/OccupancyGrid`
- publisher: `map_server`
- frame: `map`
- resolution: `0.05000000074505806`
- width: `1744`
- height: `2683`
- origin: `x=-39.1`, `y=-85.15`, `z=0.0`, identity orientation
- map load timestamp finite
- QoS durability: transient local

`/map_server` `yaml_filename`:

`/home/dog/phase2_builds/p2ab_scaffolding_validation_20260801_025103/install/share/parking_robot_bringup/maps/phase2_clean_map.yaml`

No `/data/...` path or original workspace map was used.

## Odometry verification

Observed `/Odometry` for 10 seconds:

- type: `nav_msgs/msg/Odometry`
- publisher: `phase2_fake_base`
- frame_id: `odom`
- child_frame_id: `base_footprint`
- samples: `501`
- measured rate: `50.0017 Hz`
- first pose: `x=5.425`, `y=-53.725`, `z=0.0`, `yaw=0.0`
- last pose: `x=5.425`, `y=-53.725`, `z=0.0`, `yaw=0.0`
- translation drift: `0.0 m`
- yaw drift: `0.0 rad`
- quaternion norm: `1.0`
- nonzero velocity samples: `0`
- timestamps increased normally

Stationary behavior passed during the no-command observation interval.

## TF verification

Observed:

- `map -> odom`: static identity transform;
- `odom -> base_footprint`: dynamic transform at `x=5.425`, `y=-53.725`, `z=0.0`, yaw `0.0`;
- `map -> base_footprint`: composed transform at `x=5.425`, `y=-53.725`, `z=0.0`, yaw `0.0`.

No `body` frame was introduced by the Phase 2 stack. The disputed physical `body -> base_footprint` calibration was not involved.

## Command-topic isolation

`/cmd_vel_phase2_mock`:

- type: `geometry_msgs/msg/Twist`
- publishers: controller_server and behavior_server endpoints only;
- subscriber: `phase2_fake_base` only;
- 10 second idle observation: `0` messages, `0` nonzero commands.

Confirmed absent:

- `/wheelchair_control_command`
- `/wheelchair_control_command_raw`
- `/vehicle_cmd_safe`
- `/cmd_vel_nav`
- `/active_plan`
- `/plan_nav`
- `/scan`
- `/cloud_registered`
- `/livox/lidar`

No legacy or physical command topic was created.

## Nav2 interfaces

Observed action servers:

- `/backup`
- `/compute_path_through_poses`
- `/compute_path_to_pose`
- `/follow_path`
- `/navigate_through_poses`
- `/navigate_to_pose`
- `/spin`
- `/wait`

Observed costmap/map/lifecycle services included:

- `/map_server/map`
- `/map_server/load_map`
- `/global_costmap/get_costmap`
- `/local_costmap/get_costmap`
- `/global_costmap/clear_entirely_global_costmap`
- `/local_costmap/clear_entirely_local_costmap`
- lifecycle services for managed nodes

Action/server discovery only was performed. No NavigateToPose or planner/controller action was invoked.

## Costmap state

Global costmap parameters:

- global frame: `map`
- robot base frame: `base_footprint`
- plugins include static layer and inflation layer
- no obstacle layer
- no voxel layer
- `observation_sources: ''`

Local costmap parameters:

- global frame: `odom`
- robot base frame: `base_footprint`
- rolling window: `true`
- plugins include static layer and inflation layer
- no obstacle layer
- no voxel layer
- `observation_sources: ''`

Startup emitted two transient transform wait messages before the fake-base transform became available. No repeated sensor-topic wait loop was observed.

## Idle stability

The stack remained running for the idle observation window. `/Odometry` remained stationary and `/cmd_vel_phase2_mock` produced no messages.

CPU/RSS samples were collected in `idle_stability_samples.txt`. No process crash occurred during idle. However, lifecycle state remained invalid because `bt_navigator` stayed inactive.

## Shutdown result

SIGINT was sent once to the launch process. Nav2 C++ nodes and static transform publisher exited cleanly. The launch process exited after 2 seconds.

Blocking shutdown failure:

`phase2_fake_base` exited with code `1`.

The traceback shows its timer callback attempted to publish after the rclpy context became invalid, then `rclpy.shutdown()` was called after shutdown had already occurred.

Post-run cleanup:

- ROS CLI daemon created by discovery was stopped with `ros2 daemon stop`;
- no P2-C ROS/Nav2 process remained after cleanup;
- repository working tree was restored to clean after moving generated TF evidence into the runtime directory.

## Warning/error disposition

See:

`/home/dog/phase2_reports/PHASE2_P2C_STARTUP_LOG_CLASSIFICATION.md`

Blocking:

1. `bt_navigator` failed activation due BT/plugin configuration.
2. `phase2_fake_base` did not shut down cleanly.

Non-blocking:

- static transform publisher deprecated argument warning;
- transient startup transform wait;
- optional `controller_server.verbose` parameter warning;
- helper-only `ros2 topic hz` shutdown artifact.

## Excluded-component verification

No excluded nodes/topics were observed:

- no wheelchair controller;
- no CAN/can0 topic/process;
- no plan_nav;
- no Pure Pursuit;
- no Collision Monitor;
- no Livox/FAST-LIO/RTAB-Map live localization;
- no camera/YDLIDAR/ultrasonic sensor topics;
- no semantic processing;
- no command adapter;
- no `/vehicle_cmd_safe`;
- no `/wheelchair_control_command`.

## Limitations

This run is no-goal only. It does not validate:

- initial pose publication;
- NavigateToPose action execution;
- planner success/failure behavior;
- MPPI tracking;
- runtime navigation completion;
- product performance;
- physical safety.

## P2-D readiness decision

`P2-D_NOT_READY`

P2-D must not start until P2-C is rerun and passes after source remediation for:

1. BT navigator activation/configuration;
2. fake-base clean SIGINT shutdown.

## Integrity confirmation

- No source/configuration was modified.
- No `/initialpose` was published.
- No Nav2 goal was sent.
- No hardware, sensor, CAN, or UDP access occurred.
- No system package was installed or removed.
- No tag was created, moved, or recreated.
- Tracked Phase 2 documentation was not updated because P2-C did not pass.

## Final status

`PHASE2_P2C_NO_GOAL_RUNTIME_NEEDS_REVIEW`
