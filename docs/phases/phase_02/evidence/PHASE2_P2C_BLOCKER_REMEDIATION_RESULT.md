# Phase 2 P2-C Blocker Remediation Result

Date: 2026-08-01

Source revision before remediation: `a88b22689fc6da4a1d2f16f23c3336d4aa572356`

## Failed-run root causes

1. `bt_navigator` failed activation because the installed Humble `navigate_through_poses_w_replanning_and_recovery.xml` references `RemovePassedGoals`, while the Phase 2 BT plugin list did not include the installed library `nav2_remove_passed_goals_action_bt_node`.
2. `phase2_fake_base` exited with code 1 on shutdown because the timer could publish after the `rclpy` context became invalid and the node then called `rclpy.shutdown()` after shutdown had already occurred.

## Installed Humble evidence

- `nav2_behavior_tree` package version inspected: `1.1.20`.
- NavigateToPose navigator name from installed header: `navigate_to_pose`.
- NavigateThroughPoses navigator name from installed header: `navigate_through_poses`.
- Installed NavigateToPose BT XML: `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`.
- Installed NavigateThroughPoses BT XML: `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`.
- Installed behavior-tree library export includes `nav2_compute_path_through_poses_action_bt_node`, `nav2_navigate_through_poses_action_bt_node`, and `nav2_remove_passed_goals_action_bt_node`.

## Selected BT correction

Correction used: fallback supported by installed Humble.

`bt_navigator` in this Humble install constructs both NavigateToPose and NavigateThroughPoses navigators. Phase 2 still uses sequential individual NavigateToPose goals for future P2-D/P2-E, but the unused through-poses navigator must load cleanly at lifecycle configuration time. The configuration now explicitly sets the installed through-poses default BT XML and includes its required installed BT node libraries.

No custom BT XML was created. No sensor, hardware, command adapter, Collision Monitor, or later-phase dependency was introduced.

## Fake-base shutdown correction

`phase2_fake_base` now:

- cancels timer publication before node teardown;
- checks ROS context validity before timer publication;
- catches only `RCLError` caused after context invalidation and re-raises `RCLError` while the context is still valid;
- handles `KeyboardInterrupt` and `ExternalShutdownException` as normal shutdown paths;
- calls `rclpy.shutdown()` only when `rclpy.ok()` is still true.

Normal command staleness, integration, odometry and TF behavior are unchanged.

## Files changed

- `src/parking_robot_bringup/config/phase2_nav2_params.yaml`
- `src/parking_robot_bringup/parking_robot_bringup/phase2_fake_base.py`
- `src/parking_robot_bringup/test/test_phase2_config_scope.py`
- `src/parking_robot_bringup/test/test_phase2_fake_base_shutdown.py`

## Validation

- Python syntax compilation: PASS.
- YAML/package XML parse: PASS.
- Unit/static tests: PASS, 30 tests passed.
- LaunchDescription generation: PASS, 17 launch actions including 8 node actions.
- `ros2 launch ... --show-args`: PASS against fresh install.
- Build root: `/home/dog/phase2_builds/p2c_blocker_remediation_20260801_034600`.
- Package build: PASS, only `parking_robot_bringup` built.
- Package tests: PASS, 30 tests, 0 failures.
- Installed source/resource checksums: MATCH for launch, config, map and Python files.

## Behavior before and after

| Blocker | Before | After |
|---|---|---|
| BT Navigator | `bt_navigator` failed activation with `Node not recognized: RemovePassedGoals`. | `bt_navigator` reaches ACTIVE; no missing-node/plugin/XML-load error appears. |
| Fake base shutdown | `phase2_fake_base` exited code 1 with publish-after-invalid-context and double-shutdown traceback. | `phase2_fake_base` finished cleanly after group SIGINT; no traceback or invalid-context publish appears. |

## Scope integrity

No Phase 0/1 files were modified. No legacy command topic, CAN, UDP, sensor, Collision Monitor, plan_nav, Pure Pursuit, FAST-LIO, RTAB-Map, camera, YDLIDAR, Gazebo or Phase 3 component was added or activated.
