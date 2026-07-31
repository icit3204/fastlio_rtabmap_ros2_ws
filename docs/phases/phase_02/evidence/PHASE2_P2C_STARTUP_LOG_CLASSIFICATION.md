# Phase 2 P2-C Startup Log Classification

Date: 2026-08-01  
Runtime directory: `/home/dog/phase2_runtime/p2c_no_goal_20260801_030415`

## Decision

`PHASE2_P2C_NO_GOAL_RUNTIME_NEEDS_REVIEW`

## Warning/error table

| Timestamp/source | Node | Message summary | Repeated | Cause | Impact | Disposition | Next action |
|---|---|---|---|---|---|---|---|
| `1785524807.435` | `phase2_map_to_odom_static_tf` | Old-style `static_transform_publisher` arguments are deprecated | one-time | Launch uses positional static transform arguments | Non-blocking for P2-C behavior; should be modernized later | ACCEPTED_NON_BLOCKING | Use new-style arguments in a future source remediation task |
| `1785524809.019` and `1785524809.519` | `global_costmap.global_costmap` | Timed out waiting for transform `base_footprint` to `map` during activation | two startup messages | Costmap checked before fake-base TF became discoverable | Non-blocking in this run; transform became available and planner_server activated | ACCEPTED_STARTUP_TRANSIENT | Monitor after source remediation; no repeated runtime loop observed |
| `1785524811.292` | `controller_server` | Parameter `controller_server.verbose` not found | one-time | MPPI/controller optional parameter lookup | Non-blocking; controller activated | ACCEPTED_NON_BLOCKING | Optional cleanup only |
| `1785524811.514` | `bt_navigator` | Exception loading BT: node not recognized `RemovePassedGoals` | one-time blocking | `default_nav_through_poses_bt_xml` defaulted to installed NavigateThroughPoses BT, but configured plugin list lacks `nav2_remove_passed_goals_action_bt_node` | Blocking: `bt_navigator` remained inactive and lifecycle manager aborted bringup | BLOCKING | P2-D must not start; fix BT/plugin configuration in a separate source task |
| `1785524811.515` | `lifecycle_manager_navigation` | Failed to change state for `bt_navigator`; failed to bring up all requested nodes | one-time blocking | Consequence of BT load failure | Blocking: required lifecycle activation criterion not met | BLOCKING | Fix Nav2 BT configuration before rerunning P2-C |
| `1785524984.535` | `bt_navigator.rclcpp` | Failed to send response to `/bt_navigator/get_state` timeout | one-time observed during introspection | CLI lifecycle polling while node was in failed/inactive state | Diagnostic symptom of failed bt_navigator activation | BLOCKING_CONTEXT | Recheck after BT configuration fix |
| idle sample 1 | ROS CLI observer | `failed to shutdown: rcl_shutdown already called` from `ros2 topic hz` | one-time in evidence helper output | ROS CLI shutdown behavior during `timeout` termination | Does not affect launched stack; evidence-helper artifact | ACCEPTED_HELPER_ARTIFACT | Avoid using `timeout` around `ros2 topic hz` or ignore as helper-only |
| shutdown | `phase2_fake_base` | `RCLError: Failed to publish: publisher's context is invalid` followed by shutdown already called | one-time blocking on SIGINT | Timer callback attempted publish while rclpy context was invalid during SIGINT shutdown | Blocking: fake base exited with code 1; clean-shutdown criterion not met | BLOCKING | Add shutdown-safe spin/timer handling in a separate source task |

## Benign lifecycle/status messages

The following categories were expected and are not blocking by themselves:

- lifecycle node launched / waiting on lifecycle transitions;
- map loading and costmap static-layer resizing;
- planner/controller/behavior plugin creation/configuration;
- deactivate/cleanup/destroy messages during SIGINT shutdown for Nav2 C++ nodes.

## Blocking summary

P2-C failed two required criteria:

1. `bt_navigator` did not reach `active`.
2. `phase2_fake_base` did not exit cleanly on SIGINT.

Therefore P2-C evidence is preserved, but tracked Phase 2 documentation was not updated and no documentation commit was created.
