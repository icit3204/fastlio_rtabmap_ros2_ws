# Phase 2 P2-D One-Goal Runtime Result

Date: 2026-08-01

Decision: `PHASE2_P2D_ONE_GOAL_RUNTIME_NEEDS_REVIEW`

## Runtime roots

- Install root: `/home/dog/phase2_builds/p2d_one_goal_validation_20260801_042200/install`
- Runtime root: `/home/dog/phase2_runtime/p2d_one_goal_20260801_042900`
- ROS setup order: `/opt/ros/humble/setup.bash`, then fresh P2-D install.
- `ROS_LOCALHOST_ONLY=1`
- `ROS_DOMAIN_ID=195`
- `use_sim_time:=false`
- `use_rviz:=false`
- `autostart:=true`

## Launch command

```text
ros2 launch parking_robot_bringup phase2_core_nav2.launch.py use_sim_time:=false use_rviz:=false autostart:=true initial_x:=5.425 initial_y:=-53.725 initial_yaw:=0.0
```

## Test command

```text
ros2 run parking_robot_bringup phase2_goal_test_runner --ros-args -p start_x:=5.425 -p start_y:=-53.725 -p start_yaw:=0.0 -p goal_x:=0.325 -p goal_y:=-55.175 -p goal_yaw:=0.0 -p frame_id:=map -p goal_timeout_sec:=180.0 -p output_result_path:=/home/dog/phase2_runtime/p2d_one_goal_20260801_042900/PHASE2_P2D_TEST_RESULT.json -p test_name:=p2d_one_goal -p source_revision:=985cda0f20859fb6144362248e17b1fa42efd5e4
```

## Lifecycle result

Lifecycle nodes reached ACTIVE before the runner started. Active poll iteration: `1`.

## Initial pose reset

- Initialpose verified: `True`
- Position error: `0.0` m
- Yaw error: `0.0` rad
- Settle duration: `0.00235291200078791` sec
- First odometry after reset: `{'x': 5.425, 'y': -53.725, 'yaw': 0.0}`
- Settled odometry: `{'x': 5.425, 'y': -53.725, 'yaw': 0.0}`

## Action result

- Goal send count: `1`
- Goal accepted: `True`
- Action result: `None`
- Failure reasons: `['NavigateToPose result timed out']`
- Elapsed: `181.08709147000627` sec

The action did not return SUCCEEDED within the configured 180 second timeout.

## Global path analysis

- Path received: `True`
- Topic: `/plan`
- Frame: `map`
- Pose count: `201`
- Path length: `5.031111085448034` m
- Max gap: `0.04575428989299877` m
- Endpoint error: `0.0` m
- Finite: `True`
- Occupied hits: `0`
- Unknown hits: `0`
- Out-of-bounds hits: `0`

## Command analysis

- Command messages: `3405`
- Nonzero command messages: `3395`
- First nonzero command elapsed: `1.1632455840008333` sec
- Last nonzero command elapsed: `181.0382276330056` sec
- Max abs linear.x: `0.05000000074505806` m/s
- Max abs angular.z: `0.8` rad/s
- Invalid command count: `0`
- Unsupported command count: `0`
- Limit violation count: `0`
- Configured command limits: `{'max_abs_angular_z': 0.8, 'max_abs_linear_x': 0.25, 'tolerance': 1e-06}`
- Post-result stop latency: not available because the action timed out before a successful result.

## Fake odometry analysis

- Sample count: `9007`
- Total translation: `0.39563587622402063` m
- Max linear velocity: `0.05000000074505806` m/s
- Max angular velocity: `0.8` rad/s
- Non-finite samples: `0`
- Timestamp nonmonotonic count: `0`
- Quaternion norm range: `1.0` to `1.0`

The fake base moved in closed loop, but only about `0.39563587622402063` m before timeout.

## TF and command isolation

- `/cmd_vel_phase2_mock` remained the isolated command topic in monitor samples.
- No `/wheelchair_control_command`, `/vehicle_cmd_safe`, `/cmd_vel_nav`, CAN, sensor, semantic, plan_nav, Pure Pursuit, FAST-LIO, RTAB-Map, camera, YDLIDAR, Gazebo, or later-phase topic/process was found in evidence scans.
- `/Odometry` and `/tf` ownership were inspected before and after the run.

## Warnings/errors

Blocking runtime issue: repeated controller `Failed to make progress` events followed by action timeout.

See `PHASE2_P2D_STARTUP_AND_RUNTIME_LOG_CLASSIFICATION.md` for warning/error classification.

## Shutdown

- Shutdown status: `launch_exited_after_group_sigint`
- Residual Phase 2 processes after shutdown: `0`
- ROS daemon stopped after evidence collection.

## Limitations

P2-D did not complete. Sequential goals, cancellation, planner-failure testing, controller-timeout testing, TF-failure testing, repetition, P2-E, P2-F, and P2-G were not started.

## P2-E readiness decision

P2-E is not ready. P2-D requires review before later Phase 2 runtime stages.
