# parking_robot_bringup

Isolated Phase 2 P2-A/P2-B package for the Core Simple Nav2 Navigation Baseline.

This package is deliberately independent of legacy `robot_bringup`, `plan_nav`,
Pure Pursuit, wheelchair controller/CAN code, live perception, and hardware
drivers.

## Implemented scope

- Packaged copy of `clean_map.yaml` / `clean_map.pgm`
- Static scenario selection from the copied map
- `phase2_fake_base` node:
  - subscribes to `/cmd_vel_phase2_mock`
  - publishes `/Odometry`
  - broadcasts `odom -> base_footprint`
  - accepts `/initialpose` resets
- Explicit minimal Nav2 launch description for static inspection
- Unit and static-scope tests

## Runtime status

`NOT_STARTED`

This package has not yet been used to run Nav2, send a goal, validate navigation
success, or make any runtime safety claim.

