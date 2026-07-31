# Phase 2 — Core Simple Nav2 Navigation Baseline

Date: 2026-08-01  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Current status

Runtime status: `NOT_STARTED`

Completed scope:

- readiness audit;
- P2-A isolated package/configuration scaffolding;
- P2-B static validation and package build/install validation.

No Phase 2 runtime success, navigation completion, or runtime safety claim has been made.

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

The chain has been statically configured only. It has not been run.

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
- Source-level pytest: 22 tests passed.
- Direct LaunchDescription generation passed.
- Installed `ros2 launch ... --show-args` passed without launching nodes.
- External colcon build/test of only `parking_robot_bringup` passed.
- Installed file checksums matched source.
- Installed map YAML resolved to installed PGM.

## Remaining Phase 2 stages

Open:

- P2-C runtime launch in isolated localhost/domain mode;
- P2-D initial pose and goal execution;
- P2-E planner failure evidence;
- P2-F command-topic isolation runtime evidence;
- P2-G final Phase 2 baseline freeze decision.

Do not treat this document as Phase 2 runtime completion evidence.

## Evidence

See `PHASE_02_EVIDENCE_INDEX.md`.

