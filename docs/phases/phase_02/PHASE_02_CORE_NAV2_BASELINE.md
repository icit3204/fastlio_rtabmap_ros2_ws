# Phase 2 — Core Simple Nav2 Navigation Baseline

Date: 2026-08-01  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Current status

Runtime status: `IN_PROGRESS`

Completed scope:

- readiness audit;
- P2-A isolated package/configuration scaffolding;
- P2-B static validation and package build/install validation;
- initial P2-C no-goal runtime attempt retained as failed evidence;
- source remediation for BT Navigator activation and fake-base shutdown;
- P2-C isolated no-goal runtime retest completed successfully.

P2-C no-goal runtime is complete after remediation and passing retest. No navigation goal completion, path-following success, or runtime safety claim beyond stationary no-goal bringup has been made.

## Implemented package

`src/parking_robot_bringup/`

This package is isolated from legacy `robot_bringup` runtime launch paths and does not include or activate:

- semantic grid tools/topics;
- Collision Monitor;
- Generic Command Safety Gate;
- Wheelchair Command Adapter;
- wheelchair controller/CAN;
- plan_nav;
- Pure Pursuit;
- Livox/FAST-LIO/RTAB-Map live localization;
- camera/YDLIDAR/ultrasonic drivers;
- keepout/speed filters;
- Gazebo;
- Phase 3 Mission Manager.

## Intended future runtime chain

```text
Nav2 controller_server or behavior_server
  -> /cmd_vel_phase2_mock
  -> phase2_fake_base
  -> /Odometry
  -> odom -> base_footprint

static_transform_publisher
  -> map -> odom identity
```

The chain has now been run only for P2-C stationary no-goal validation. No goal has been sent.

## Map

Packaged isolated test asset:

- `src/parking_robot_bringup/maps/phase2_clean_map.yaml`
- `src/parking_robot_bringup/maps/phase2_clean_map.pgm`

The PGM is byte-identical to `scripts/offline_nav_maps/clean_map.pgm`. The copied YAML image field resolves to the packaged PGM.

This is an isolated Phase 2 test asset, not a new site-map authority.

## Static scenarios

Selected from direct PGM analysis:

- start: `(5.425, -53.725, 0.0)`
- goal A: `(23.625, -81.125, 0.0)`
- goal B: `(0.325, -55.175, 0.0)`
- goal C: `(4.725, -58.375, 0.0)`
- planner-failure goal: `(-8.025, -8.725, 0.0)` in confirmed occupied space

These are static selections only and remain unvalidated at runtime.

## Validation completed

- Python syntax compilation passed.
- YAML and package XML parsing passed.
- Source-level pytest after remediation: 30 tests passed.
- Direct LaunchDescription generation passed.
- Installed `ros2 launch ... --show-args` passed without launching nodes.
- External colcon build/test of only `parking_robot_bringup` passed.
- Installed file checksums matched source.
- Installed map YAML resolved to installed PGM.
- P2-C no-goal runtime retest passed: all required lifecycle nodes ACTIVE, map/odometry/TF verified, `/cmd_vel_phase2_mock` isolated, no nonzero idle command, and clean shutdown after group SIGINT.


## P2-C remediation note

The first P2-C no-goal runtime attempt failed for two scoped reasons:

1. `bt_navigator` failed lifecycle activation because the installed Humble default NavigateThroughPoses BT referenced `RemovePassedGoals` without the required configured BT plugin library.
2. `phase2_fake_base` exited with code 1 on shutdown due publish-after-invalid-context and double shutdown handling.

The remediation commit keeps the installed Humble standard through-poses navigator loadable by adding the exact installed BT node libraries, and updates fake-base shutdown handling so the timer stops before teardown and context shutdown is called only when still valid. The passing retest is the authoritative P2-C runtime evidence.

Phase 2 overall: `IN_PROGRESS`

## Remaining Phase 2 stages

Status:

- P2-C runtime launch in isolated localhost/domain mode: COMPLETE.
- P2-D initial pose and goal execution: NOT_STARTED.

Open:

- P2-D initial pose and goal execution;
- P2-E planner failure evidence;
- P2-F command-topic isolation runtime evidence;
- P2-G final Phase 2 baseline freeze decision.

Do not treat this document as evidence of goal execution, MPPI path-following success, planner failure behavior, or full Phase 2 closure.

## Evidence

See `PHASE_02_EVIDENCE_INDEX.md`.

