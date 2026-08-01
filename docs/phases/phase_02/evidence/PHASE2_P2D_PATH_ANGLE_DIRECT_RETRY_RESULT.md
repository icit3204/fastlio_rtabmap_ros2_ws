# PHASE2 P2D PathAngleCritic Direct FollowPath Retry Result

## Decision

P2D_PATH_ANGLE_DIRECT_FOLLOW_NEEDS_REVIEW

The clean direct FollowPath experiment did not meet the direct-test acceptance gate. Lifecycle startup succeeded and `PathAngleCritic` loaded, but the single `/follow_path` action returned `ABORTED` (`status=6`) after the controller reported `Failed to make progress`.

## Source and dirty-state preservation

- Repository: `/home/dog/fastlio_rtabmap_ros2_ws`
- HEAD: `985cda0f20859fb6144362248e17b1fa42efd5e4`
- origin/main: `985cda0f20859fb6144362248e17b1fa42efd5e4`
- Dirty state remained limited to the expected Phase 2 runner files:

```text
M src/parking_robot_bringup/package.xml
 M src/parking_robot_bringup/setup.py
?? src/parking_robot_bringup/parking_robot_bringup/phase2_goal_test_runner.py
?? src/parking_robot_bringup/test/test_phase2_goal_test_runner.py
```

- Fresh dirty-state backup patch: `/home/dog/phase2_reports/PHASE2_P2D_UNCOMMITTED_RUNNER_20260801_060036.patch`
- Fresh dirty-state checksum/state record: `/home/dog/phase2_reports/PHASE2_P2D_UNCOMMITTED_STATE_20260801_060036.txt`

## Build/test validation

- Build root: `/home/dog/phase2_builds/p2d_path_angle_direct_retry_20260801_060036`
- Built package: `parking_robot_bringup` only
- Test result: `43 passed`
- Runner installed: yes
- Installed tracked Nav2 configuration was not changed; candidate YAML remained external.

## Candidate

- Candidate file: `/home/dog/phase2_runtime/p2d_path_angle_candidate_20260801_050532/phase2_nav2_params_path_angle_candidate.yaml`
- Candidate SHA256: `1adc0f865fefb5f6d42e297ad59edc5e89056367870e37316336301729c42791`
- Semantic change: appended `PathAngleCritic` to `FollowPath.critics` and added only the recorded `PathAngleCritic` parameter block.
- Candidate was not applied to tracked source.

## Runtime environment

- Runtime root: `/home/dog/phase2_runtime/p2d_path_angle_direct_retry_20260801_060036`
- ROS_DOMAIN_ID: `210`
- ROS_LOCALHOST_ONLY: `1`
- use_sim_time: `false`
- use_rviz: `false`
- autostart: `false`
- Startup method: explicit delayed `/lifecycle_manager_navigation/manage_nodes` `STARTUP` after lifecycle services were discoverable and a 2.0 s quiet interval.

## Lifecycle and service-discovery evidence

- Service discovery gate: `SERVICE_DISCOVERY_GATE_PASS`
- Services visible at gate: `91`
- ManageLifecycleNodes response:

```text
waiting for service to become available...
requester: making request: nav2_msgs.srv.ManageLifecycleNodes_Request(command=0)

response:
nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)
```

- STARTUP service call elapsed time: `5.182744979858398` seconds
- Final startup summary: `startup_rc=0`
- Lifecycle log confirmed `Managed nodes are active`.
- `PathAngleCritic` log: `PathAngleCritic instantiated with 1 power and 2.200000 weight. Reversing not allowed.`

## Runtime node/topic scope

Nodes after startup:

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
/transform_listener_impl_aaaac34f7930
/transform_listener_impl_aaaaf2b4b640
/transform_listener_impl_aaab03b6fde0
```

Topics after startup:

```text
/Odometry [nav_msgs/msg/Odometry]
/behavior_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/behavior_tree_log [nav2_msgs/msg/BehaviorTreeLog]
/bond [bond/msg/Status]
/bt_navigator/transition_event [lifecycle_msgs/msg/TransitionEvent]
/cmd_vel_phase2_mock [geometry_msgs/msg/Twist]
/controller_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
/global_costmap/costmap [nav_msgs/msg/OccupancyGrid]
/global_costmap/costmap_raw [nav2_msgs/msg/Costmap]
/global_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]
/global_costmap/footprint [geometry_msgs/msg/Polygon]
/global_costmap/global_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/global_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]
/goal_pose [geometry_msgs/msg/PoseStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/local_costmap/costmap [nav_msgs/msg/OccupancyGrid]
/local_costmap/costmap_raw [nav2_msgs/msg/Costmap]
/local_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]
/local_costmap/footprint [geometry_msgs/msg/Polygon]
/local_costmap/local_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/local_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]
/map [nav_msgs/msg/OccupancyGrid]
/map_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/plan [nav_msgs/msg/Path]
/planner_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/rosout [rcl_interfaces/msg/Log]
/speed_limit [nav2_msgs/msg/SpeedLimit]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/trajectories [visualization_msgs/msg/MarkerArray]
/transformed_global_plan [nav_msgs/msg/Path]
```

Excluded node/topic finding: `none found in startup node/topic/log evidence`

## Direct FollowPath action

- Action sent: exactly one `/follow_path` goal
- NavigateToPose sent: no
- Path frame: `map`
- Path start: `(5.425, -53.725, yaw 0.0)`
- Path end: `(7.425, -53.725, yaw 0.0)`
- Path samples: approximately 21 poses at 0.10 m spacing
- controller_id: `FollowPath`
- goal_checker_id: `general_goal_checker`
- Timeout: 90 s

## Initial pose verification

- `/initialpose` was published once by the temporary direct-test client.
- Reset verified: `True`
- Initial odometry sample: `{'cmd_angular_z': 0.0, 'cmd_linear_x': 0.0, 'distance_to_endpoint': 2.0, 'elapsed': -0.002257929001643788, 'goal_bearing': 0.0, 'heading_error': 0.0, 'odom_twist_angular_z': 0.0, 'odom_twist_linear_x': 0.0, 'path_heading': 0.0, 'path_heading_error': 0.0, 't': 52287.607668565, 'tf_odom_translation_error': 0.0, 'tf_odom_yaw_error': 0.0, 'tf_x': 5.425, 'tf_y': -53.725, 'tf_yaw': 0.0, 'x': 5.425, 'y': -53.725, 'yaw': 0.0}`

## Measurements

| Metric | Value |
|---|---:|
| goal accepted | True |
| result status | ABORTED (6) |
| elapsed sec | 79.8576446770021 |
| total translation m | 2.085651840979229 |
| final position error m | 0.44730536973715584 |
| final yaw error rad | 0.006440924432327577 |
| max lateral deviation m | 0.45433838620517975 |
| max absolute yaw error rad | 0.4929180548488392 |
| max abs linear.x command | 0.06738881766796112 |
| max abs angular.z command | 0.08867331594228745 |
| command messages | 1598 |
| nonzero command messages | 1597 |
| invalid command messages | 0 |
| unsupported command field count | 0 |
| command-limit violations | 0 |
| corrective angular-command samples | 1209 |
| divergent angular-command samples | 2783 |
| post-result command-stop latency sec | 0.0 |

Final pose:

```json
{
  "cmd_angular_z": 0.0,
  "cmd_linear_x": 0.0,
  "distance_to_endpoint": 0.44730536973715584,
  "elapsed": 82.85682046000147,
  "goal_bearing": 1.5906727855470753,
  "heading_error": 1.5971137099794026,
  "odom_twist_angular_z": 0.0,
  "odom_twist_linear_x": 0.0,
  "path_heading": 0.0,
  "path_heading_error": 0.006440924432327577,
  "t": 52370.466746954,
  "tf_odom_translation_error": 0.0,
  "tf_odom_yaw_error": 0.0,
  "tf_x": 7.4338902613196165,
  "tf_y": -54.17221701337199,
  "tf_yaw": -0.006440924432327579,
  "x": 7.4338902613196165,
  "y": -54.17221701337199,
  "yaw": -0.006440924432327579
}
```

## Warning/error disposition

Key startup/runtime messages:

```text
[lifecycle_manager-8] [INFO] [1785535408.717235390] [lifecycle_manager_navigation]: Configuring map_server
[controller_server-5] [INFO] [1785535409.712983585] [controller_server]: PathAngleCritic instantiated with 1 power and 2.200000 weight. Reversing not allowed.
[controller_server-5] [INFO] [1785535409.713027843] [controller_server]: Critic loaded : mppi::critics::PathAngleCritic
[lifecycle_manager-8] [INFO] [1785535409.903136847] [lifecycle_manager_navigation]: Activating map_server
[controller_server-5] [WARN] [1785535410.287432810] [controller_server]: Parameter controller_server.verbose not found
[lifecycle_manager-8] [INFO] [1785535410.740742154] [lifecycle_manager_navigation]: Managed nodes are active
[controller_server-5] [ERROR] [1785535504.680827326] [controller_server]: Failed to make progress
[controller_server-5] [WARN] [1785535504.681831327] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[behavior_server-6] [INFO] [1785535510.105641593] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[lifecycle_manager-8] [INFO] [1785535510.105644697] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[controller_server-5] [INFO] [1785535510.105684283] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[planner_server-4] [INFO] [1785535510.106124521] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[map_server-1] [INFO] [1785535510.106164202] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[static_transform_publisher-2] [INFO] [1785535510.106219500] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[bt_navigator-7] [INFO] [1785535510.106228012] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[INFO] [static_transform_publisher-2]: process has finished cleanly [pid 158303]
[INFO] [map_server-1]: process has finished cleanly [pid 158301]
[INFO] [behavior_server-6]: process has finished cleanly [pid 158311]
[INFO] [lifecycle_manager-8]: process has finished cleanly [pid 158371]
[INFO] [phase2_fake_base-3]: process has finished cleanly [pid 158305]
[INFO] [controller_server-5]: process has finished cleanly [pid 158309]
[INFO] [bt_navigator-7]: process has finished cleanly [pid 158332]
[INFO] [planner_server-4]: process has finished cleanly [pid 158307]
```

Blocking runtime issue: controller-server progress failure during the direct FollowPath action. This caused the action server to abort the handle.

Non-blocking startup warning: `Parameter controller_server.verbose not found` appeared once after controller activation and did not prevent lifecycle activation or action execution.

## Shutdown

- One group SIGINT was sent by the runtime harness.
- Launch log shows all Phase 2 processes finished cleanly, including `phase2_fake_base`.
- No residual Phase 2 runtime process was intentionally left running.

## Interpretation

The lifecycle/autostart race was controlled by fresh domain `210`, autostart disabled, service-discovery gate, 2.0 s quiet interval, and explicit STARTUP. That gate passed.

The candidate improved plugin loading state but did not satisfy direct FollowPath behavior. The robot moved forward and commands were finite, but lateral deviation reached `0.45433838620517975` m, final position error was `0.44730536973715584` m, and the controller aborted for lack of progress. Because the direct-test acceptance threshold is `<= 0.25 m`, this run is not acceptable.

## Exact next recommendation

Do not apply this candidate to tracked source as authoritative. The next investigation should remain in the MPPI profile/feedback area and should use the captured synchronized timeline to determine why the controller considers progress insufficient despite finite forward motion. Per this task scope, no additional critic or parameter search was performed.

## Integrity

- Repository files remained unchanged during the runtime experiment.
- Candidate was not applied to tracked config.
- Exactly one direct FollowPath goal was sent.
- No NavigateToPose goal was sent.
- No hardware, sensor, CAN, or UDP access was performed.
- No system package operation was performed.
- P2-E/P2-F/P2-G were not started.

## Final process check update

An exact process-name check after report generation found no running `ros2`, `rviz2`, `map_server`, `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `phase2_fake_base`, `static_transform_publisher`, `colcon`, or `wheelchair_controller_node` processes. `ros2 daemon status` for domain 210 reported that the daemon was not running.
