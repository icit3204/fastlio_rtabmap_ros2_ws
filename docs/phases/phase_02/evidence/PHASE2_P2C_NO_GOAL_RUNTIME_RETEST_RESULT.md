# Phase 2 P2-C No-Goal Runtime Retest Result

Date: 2026-08-01

Decision: `PHASE2_P2C_NO_GOAL_RUNTIME_RETEST_PASS`

## Source revision and environment

- Source revision before remediation: `a88b22689fc6da4a1d2f16f23c3336d4aa572356`.
- Fresh remediation install: `/home/dog/phase2_builds/p2c_blocker_remediation_20260801_034600/install`.
- Runtime directory: `/home/dog/phase2_runtime/p2c_no_goal_retest_20260801_035100`.
- ROS setup order: `/opt/ros/humble/setup.bash`, then `/home/dog/phase2_builds/p2c_blocker_remediation_20260801_034600/install/setup.bash`.
- `ROS_LOCALHOST_ONLY=1`.
- `ROS_DOMAIN_ID=195`.
- `use_sim_time:=false`.
- `use_rviz:=false`.
- `autostart:=true`.

Launch command:

```text
ros2 launch parking_robot_bringup phase2_core_nav2.launch.py use_sim_time:=false use_rviz:=false autostart:=true initial_x:=5.425 initial_y:=-53.725 initial_yaw:=0.0
```

No `/initialpose` was published. No navigation goal was sent.

## Lifecycle result

All managed lifecycle nodes reached ACTIVE:

```text
active [3]
active [3]
active [3]
active [3]
active [3]
```

`bt_navigator` reached ACTIVE. The previous `RemovePassedGoals`/BT XML missing-node error did not recur.

## Runtime node set

```text
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/controller_server
/global_costmap/global_costmap
/lifecycle_manager_navigation
/local_costmap/local_costmap
/map_server
/phase2_fake_base
/phase2_map_to_odom_static_tf
/planner_server
/transform_listener_impl_aaaae8897020
/transform_listener_impl_aaaaf3b17030
/transform_listener_impl_aaab101c6070
```

The `transform_listener_impl_*` nodes are standard internal Nav2 transform listeners. No excluded legacy, sensor, CAN, semantic, Collision Monitor, plan_nav, Pure Pursuit, FAST-LIO, RTAB-Map, camera, YDLIDAR, Gazebo or Phase 3 node was present.

## Map verification

- `/map` type: `nav_msgs/msg/OccupancyGrid`.
- Publisher: `map_server`.
- Frame: `map`.
- Resolution: `0.05000000074505806`.
- Dimensions: `1744 x 2683`.
- Origin: x=-39.1, y=-85.15, z=0.0.
- Map-server `yaml_filename`: `String value is: /home/dog/phase2_builds/p2c_blocker_remediation_20260801_034600/install/share/parking_robot_bringup/maps/phase2_clean_map.yaml`.
- QoS from topic info: transient-local compatible.

## Odometry verification

- `/Odometry` type: `nav_msgs/msg/Odometry`.
- Publisher: `phase2_fake_base`.
- Frame: `odom`.
- Child frame: `base_footprint`.
- Initial pose: x=5.425, y=-53.725, yaw=0.0.
- Last pose: x=5.425, y=-53.725, yaw=0.0.
- Drift: translation=0.0 m, yaw=0.0 rad.
- Velocity remained zero: linear x=0.0, angular z=0.0.
- Quaternion norm: `1.0`.
- Topic-rate samples: see `odom_rate_samples.txt`; observed rates were approximately 50 Hz in all samples.

## TF verification

- `/tf_static` publisher count: 1, `phase2_map_to_odom_static_tf`.
- `map -> odom`: static identity transform.
- `/tf` publisher count: 1, `phase2_fake_base`.
- `odom -> base_footprint`: dynamic transform matching odometry.
- `map -> base_footprint`: x=5.425, y=-53.725, z=0, yaw=0.
- No `body` frame or physical calibration transform was involved.

## Command-topic isolation

`/cmd_vel_phase2_mock`:

- Type: `geometry_msgs/msg/Twist`.
- Publishers: Nav2 controller/behavior server endpoints only.
- Subscriber: `phase2_fake_base` only.
- Observed command messages over the 10-second probe: `0`.
- Observed nonzero command messages: `0`.

Excluded command/path topics absent: `/wheelchair_control_command, /wheelchair_control_command_raw, /vehicle_cmd_safe, /cmd_vel_nav, /active_plan, /plan_nav, /scan, /cloud_registered, /livox/lidar`.

## Nav2 action/service discovery

Actions discovered:

```text
/backup [nav2_msgs/action/BackUp]
/compute_path_through_poses [nav2_msgs/action/ComputePathThroughPoses]
/compute_path_to_pose [nav2_msgs/action/ComputePathToPose]
/follow_path [nav2_msgs/action/FollowPath]
/navigate_through_poses [nav2_msgs/action/NavigateThroughPoses]
/navigate_to_pose [nav2_msgs/action/NavigateToPose]
/spin [nav2_msgs/action/Spin]
/wait [nav2_msgs/action/Wait]
```

`/navigate_to_pose` action server was available. `/navigate_through_poses` was also available because the installed Humble BT navigator constructs both standard navigators; it was not invoked and is not the Phase 2 sequential-goal path.

## Costmap state

Global costmap:

- Frame: `map`.
- Base frame: `base_footprint`.
- Plugins: static layer and inflation layer.
- No obstacle layer, voxel layer or observation sources.

Local costmap:

- Frame: `odom`.
- Base frame: `base_footprint`.
- Rolling window: true.
- Plugins: static layer and inflation layer.
- No obstacle layer, voxel layer or observation sources.

No repeating absent-sensor warning/error was found.

## Idle stability

- Minimum active idle period: exceeded 30 seconds after lifecycle activation.
- Three resource/lifecycle/rate samples were collected.
- Lifecycle stayed ACTIVE in all samples.
- Odometry pose drift was zero within the acceptance threshold.
- No nonzero mock command appeared.
- No crash or increasing error loop was observed.

## Warning/error disposition

Blocking errors: none.

Accepted warnings:

- Static transform publisher old-style positional-argument deprecation.
- One-time `controller_server.verbose` parameter warning.

No `Node not recognized`, missing BT plugin, XML-load error, invalid-context publish traceback, or double-shutdown traceback appeared.

## Shutdown result

- Initial direct SIGINT to the orphaned launch PID did not shut down the tree because the wrapper shell had exited.
- A single SIGINT to the isolated launch process group (`PGID 108927`) produced normal lifecycle shutdown.
- Shutdown status: `launch_exited_after_group_sigint`.
- Processes remaining after shutdown: `0`.
- Runtime log shows every launched process finished cleanly, including `phase2_fake_base`.

## Limitations

P2-C did not send `/initialpose`, did not send a NavigateToPose goal, did not test planning, did not test path following, did not start RViz, and did not access hardware, sensors, CAN or UDP.

## P2-D readiness decision

P2-D one-goal runtime is ready to begin as a later task. P2-D was not started here.

## Integrity confirmation

No source/configuration was changed during the runtime retest. No `/initialpose` was published. No Nav2 goal was sent. No hardware, sensor, CAN or UDP transport was accessed. No system package was installed or removed.
