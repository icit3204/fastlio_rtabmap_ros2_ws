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

Phase 2 package validation has progressed through the isolated fake-base Nav2
stack. The P2-F failure-handling contract is closed for the Phase 2 scope:

- planner failure: expected BT recovery motion was attributed to Nav2 by the
  C++ publisher-GID monitor;
- controller no-progress: NavigateToPose ended naturally with ABORTED status 6
  before cleanup cancellation;
- cold-start missing TF: `start_fake_base:=false` starts no fake base, publishes
  no `/Odometry`, creates no dynamic `odom -> base_footprint`, and produces a
  safe no-command failure disposition.

Phase 2 does not claim runtime localization-freshness command suppression after
a previously valid transform has been cached. That remains mandatory Phase 4/5
work: the Generic Command Safety Gate must force repeated zero
`/vehicle_cmd_safe` when localization/freshness permission is invalid, and the
localization-valid monitor must independently check `/Odometry`, `map -> odom`,
complete `map -> base_footprint`, finite pose/quaternion, jump policy, and
stability before physical Nav2 arming.
