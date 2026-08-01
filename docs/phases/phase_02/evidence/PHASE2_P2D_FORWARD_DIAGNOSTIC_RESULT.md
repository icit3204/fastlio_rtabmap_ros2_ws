# Phase 2 P2-D Forward Diagnostic Result

Date: 2026-08-01

Decision: `PHASE2_P2D_FORWARD_DIAGNOSTIC_NEEDS_REVIEW`

## Scope

This diagnostic tested whether the previous P2-D failure was primarily caused by the original goal being nearly opposite the initial heading. No MPPI, progress-checker, goal-checker, controller-frequency, costmap, inflation, BT recovery, fake-base, command-limit, start-pose, or initial-yaw parameter was changed.

## Effective parameter truth

Progress checker:

- Plugin: `nav2_controller::SimpleProgressChecker`
- required_movement_radius: `0.1`
- movement_time_allowance: `20.0`

Goal checker:

- xy_goal_tolerance: `0.25`
- yaw_goal_tolerance: `0.5`
- stateful: `True`

Controller server:

- controller_frequency: `20.0`
- odom_topic: `/Odometry`
- failure_tolerance: `0.3`

MPPI:

- vx_min: `0.0`
- vx_max: `0.25`
- vx_std: `0.1`
- wz_max: `0.8`
- wz_std: `0.2`
- ax_max: `NOT_SET`
- ax_min: `NOT_SET`
- az_max: `NOT_SET`
- time_steps: `40`
- model_dt: `0.05`
- batch_size: `500`
- iteration_count: `1`
- motion_model: `DiffDrive`
- prune_distance: `1.0`
- transform_tolerance: `0.5`

Fake base remained on `/cmd_vel_phase2_mock`, `/Odometry`, 50 Hz publish rate, 0.5 s command timeout, 0.1 s integration timestep limit, and start pose `(5.425, -53.725, 0.0)`.

## Failed original-goal geometry

Original goal: `(0.325, -55.175, 0.0)`.

- Straight-line distance: `5.302` m
- Bearing: `-164.129` deg
- Heading difference from initial yaw: `-164.129` deg
- Failed-run max angular command: `0.8` rad/s
- Failed-run max linear command: `0.05000000074505806` m/s
- Failed-run total translation: `0.39563587622402063` m

The original goal was nearly behind the initial heading, but this remained a hypothesis before the forward diagnostic.

## Forward diagnostic goal

Forward goal: `(8.425, -53.725, 0.0)`. See `PHASE2_P2D_FORWARD_GOAL_SELECTION.md`.

- Straight-line distance: `3.000` m
- Bearing: `0.000` deg
- Heading difference from initial yaw: `0.000` deg

## Runtime result

- Initial pose verified: `True`
- Goal send count: `1`
- Goal accepted: `True`
- Action result: `None`
- Failure reasons: `['NavigateToPose result timed out']`
- Cancel requested: `True`
- Cancel response received: `True`
- Elapsed: `127.6800190569993` sec

The forward diagnostic did not return SUCCEEDED. The runner timed out after the configured 120 s goal timeout and cancelled the goal once.

## Path result

- Path received: `True`
- Path frame: `map`
- Pose count: `59`
- Path length: `1.508668738828552` m
- Endpoint error: `0.0` m
- Occupied hits: `0`
- Unknown hits: `0`
- Out-of-bounds hits: `0`

## Command result

- Command messages: `2394`
- Nonzero command messages: `2390`
- Max abs linear.x: `0.06743407249450684` m/s
- Max abs angular.z: `0.028292642906308174` rad/s
- Invalid command count: `0`
- Unsupported command count: `0`
- Command limit violations: `0`
- Linear nonzero count: `recorded_in_json`

Commands were finite and within limits.

## Odometry result

- Final pose: `{'x': 7.130008824540141, 'y': -54.479052371807235, 'yaw': -2.3920425480049348}`
- Final position error: `1.4985316559709452` m
- Total translation: `1.9435515172335305` m
- Max linear velocity: `0.06743407249450684` m/s
- Max angular velocity: `0.028292642906308174` rad/s
- Non-finite odometry samples: `0`
- Quaternion norm range: `1.0` to `1.0`

The fake base moved farther than the original reverse-facing run, but it still timed out with final position error above the 1.0 m acceptance threshold.

## Progress failures

Repeated progress failures occurred during the forward diagnostic: `3` occurrences.

This falsifies the hypothesis that the original failure was primarily caused by the goal being nearly opposite the initial heading. Heading contributed to the first failure geometry, but forward alignment alone did not satisfy P2-D acceptance with the current unchanged controller/fake-base settings.

## Shutdown and isolation

- Shutdown status: `launch_exited_after_group_sigint`
- Residual Phase 2 processes: `0`
- No excluded node/topic/process was found in monitor scans.

## Stop condition

Per task instruction, no MPPI tuning, progress-checker relaxation, costmap change, recovery change, fake-base change, second diagnostic goal, scenario promotion, authoritative confirmation, commit, or push was performed.
