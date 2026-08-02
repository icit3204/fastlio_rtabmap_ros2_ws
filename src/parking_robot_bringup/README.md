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

## Phase 2 Closure

Phase 2 proves the isolated fake/mock core Nav2 baseline only:

`static map` -> `fake localization` -> `NavFn A*` -> `MPPI` ->
`/cmd_vel_phase2_mock` -> `phase2_fake_base` -> `/Odometry` and
`odom -> base_footprint`.

Frozen baseline identifiers:

- Candidate C Nav2 parameters:
  `9d435d52681bbf841e32de3c9a9071d5d3fad8228c44667726b3e91be700540d`
- Phase 2 map PGM:
  `69428c94a8032fd54939492afe848156ef1687faf7a8023609a3e26d9604beb0`
- Loaded Humble NavigateToPose BT:
  `b7a7d847f5b6a9f1f29a58b109a6f8838281672dd22ae10e822b17b3f9cc6127`

Verified gates:

| Gate | Result |
| --- | --- |
| P2-D straight NavigateToPose | PASS |
| P2-E sequential goals | PASS |
| P2-E cancellation | PASS |
| P2-F planner failure | PASS |
| P2-F controller no-progress | PASS |
| P2-F cold-start missing TF | PASS |
| P2-G five-run repeatability | PASS |

P2-G repeatability used five independent fresh-stack trials. All five sent one
NavigateToPose goal, received SUCCEEDED status, used fake closed-loop odometry,
stopped cleanly, and produced zero unknown nonzero command publishers.

| Trial | Domain | Result | Final XY Error (m) | Duration (s) | Command Hz |
| --- | ---: | --- | ---: | ---: | ---: |
| 1 | 211 | PASS | 0.248885 | 48.230971 | 20.024422 |
| 2 | 207 | PASS | 0.247340 | 49.493110 | 20.023958 |
| 3 | 208 | PASS | 0.248716 | 47.923727 | 20.022988 |
| 4 | 209 | PASS | 0.247219 | 47.066354 | 20.023301 |
| 5 | 210 | PASS | 0.247747 | 47.734939 | 20.026240 |

P2-G aggregate:

- success rate: 5/5;
- unique goal IDs: 5;
- cancellations: 0;
- unknown nonzero command publishers: 0;
- post-terminal nonzero commands: 0;
- mean final XY error: 0.247981 m;
- mean action duration: 48.089820 s;
- action-duration coefficient of variation: 1.857%;
- command-count coefficient of variation: 0.054%;
- total-translation coefficient of variation: 0.028%;
- consistency flags: none.

Known Phase 2 exclusions:

- physical navigation;
- obstacle avoidance;
- semantic perception;
- runtime localization-freshness command suppression;
- Mission Manager;
- Collision Monitor;
- Generic Safety Gate;
- CAN integration.

Phase 3 entry requires this Phase 2 closure commit, preserved raw evidence, and
continued enforcement that the Phase 4 command gate and Phase 5
localization-valid monitor are mandatory before physical Nav2 arming.

Closure commit history:

- `aa1841be0a186f5d1b7bb8e0e0b952c6ad6bed95`
  `test(phase2): validate clean navigation failure handling`
