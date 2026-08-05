# Phase 4 P4-C Generic Command Safety Gate Design

## Final Status

P4-C adds a software-only generic command safety boundary after the installed
Collision Monitor:

`/cmd_vel_nav_raw -> Collision Monitor -> /cmd_vel_nav_safe -> guarded_vehicle_cmd_gate -> /vehicle_cmd_safe`

P4-C also adds an independent observation-health permission:

`synthetic LaserScan or PointCloud2 observation + Collision Monitor lifecycle/config -> collision_monitor_validity_monitor -> /system/collision_monitor_valid`

Final closure status: `P4C_FINAL_CLOSURE_PRECOMMIT_PASS` before commit, with
post-commit handoff expected to carry `PHASE4_P4C_COMPLETED_AND_PUSHED`.

## Topic And Authority Chain

| Topic | Type | Sole intended publisher in P4-C | Intended consumers |
| --- | --- | --- | --- |
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | Nav2 or synthetic test fixture | installed Collision Monitor |
| `/cmd_vel_nav_safe` | `geometry_msgs/msg/Twist` | installed Collision Monitor | `guarded_vehicle_cmd_gate` |
| `/system/localization_valid` | `std_msgs/msg/Bool` | Phase 4 test permission fixture | `guarded_vehicle_cmd_gate` |
| `/system/controller_valid` | `std_msgs/msg/Bool` | Phase 4 test permission fixture | `guarded_vehicle_cmd_gate` |
| `/system/collision_monitor_valid` | `std_msgs/msg/Bool` | `collision_monitor_validity_monitor` | `guarded_vehicle_cmd_gate` |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` | `guarded_vehicle_cmd_gate` | future P4-D adapter only |

`/vehicle_cmd_safe.header.frame_id` is frozen as `base_footprint`.
`guarded_vehicle_cmd_gate` does not subscribe to `/cmd_vel_nav_raw`.
No P4-C adapter subscribes to `/vehicle_cmd_safe`.

## Gate Contract

The gate is split into a pure deterministic core and a ROS wrapper. The core
accepts injected monotonic timestamps, command samples, permission samples,
publisher authority counts, and arm requests. It returns state, fault reason,
diagnostic fields, and the `TwistStamped` payload published by the wrapper.

Gate states are `STARTUP`, `DISARMED`, `ARMED_COMMAND`, and `FAULT`.
Startup, disarmed, and fault states publish repeated zero output at the gate
heartbeat. While armed, stale commands, false or stale permissions, invalid
motion, or authority conflicts immediately latch FAULT and force zero.

Fault latching is first-cause preserving. Recovery requires cause removal,
`arm=false`, a fresh stable prerequisite window, and a new `arm=true`.
`arm=true` is rejected while faulted.

Rejected command classes are:

- reverse linear command;
- unsupported linear or angular axes;
- nonfinite values;
- in-place rotation;
- values over the frozen generic limits.

Frozen generic limits:

| Limit | Value |
| --- | ---: |
| maximum forward velocity | `0.20 m/s` |
| maximum angular velocity | `0.50 rad/s` |
| maximum linear increase rate | `0.50 m/s^2` |
| maximum angular increase rate | `1.00 rad/s^2` |

Slew limiting applies only to increases in command magnitude. Reductions,
upstream zero, disarm zero, and fault zero are immediate.

## Timing Contract

The gate heartbeat is 20 Hz. Permission timeout is 0.50 s. Accepted stale
faulting requires FAULT within 0.60 s of the final permission receipt,
first gate-owned zero within 0.10 s after authoritative FAULT, and repeated
zero output at 18-22 Hz for at least 2.0 s.

All freshness, authority stability, recovery stability, and slew calculations
use steady or monotonic time. ROS time is allowed only for output/header
timestamps and traceability.

The final steady-clock blocker was classified as
`P4C_GATE_MULTIPLE_SAFETY_TIMERS_ROS_TIME_DEFECT`. The correction retained
explicit `STEADY_TIME` clocks for the heartbeat/watchdog/diagnostic timer and
the graph-authority timer. The retained clock objects live for the node
lifetime. Header ROS timestamps may remain frozen when `/clock` is frozen.

Accepted steady-clock runtime proof:

- status: `P4C_GATE_FROZEN_ROS_TIME_LIVENESS_PREFLIGHT_PASS`;
- status: `P4C_GATE_STEADY_CLOCK_RUNTIME_PASS`;
- frozen-clock nonzero rate: `20.035967601 Hz`;
- permission stale latency: `0.540634166 s`;
- fault-to-first-zero: `0.045113268 s`;
- zero heartbeat: `20.006348881 Hz`;
- zero duration: `2.099333579 s`;
- later gate-owned nonzero count: `0`;
- `arm=true` while faulted: rejected;
- raw archive SHA256: `d4b6fbb9a1908299bddc8971ae131fb5c568704bdc71f9bbce515f31977a0f9a`;
- raw manifest SHA256: `3224be563d61755a6473c2ab165805878c0b7ba94b2f185ebc6f90abd750a969`;
- raw entry count: `237`.

## Collision Validity Contract

`collision_monitor_validity_monitor` is the sole intended publisher of
`/system/collision_monitor_valid`. It verifies the selected synthetic
LaserScan or PointCloud2 observation source, Collision Monitor lifecycle
availability/state, configured source topic/type/frame, publisher authority,
message validity, and steady-clock freshness. It publishes a Boolean validity
heartbeat and diagnostics.

The Phase 4 permission fixture was corrected so it creates no
`/system/collision_monitor_valid` publisher when `publish_collision=false`.
This preserves sole-publisher ownership during E2E stale-observation tests.

Installed Collision Monitor stale-source fail-open remains an upstream
behavior: after observation source loss, Collision Monitor can continue to
propagate nonzero `/cmd_vel_nav_safe`. P4-C closes propagation at
`/vehicle_cmd_safe` using the independent validity monitor and the Generic
Safety Gate.

Accepted validity results:

- `P4C_VALIDITY_FULL_STACK_LIVENESS_PREFLIGHT_PASS`;
- `P4C_LASERSCAN_VALIDITY_QUALIFICATION_PASS`;
- `P4C_POINTCLOUD_VALIDITY_QUALIFICATION_PASS`;
- `P4C_VALIDITY_STEADY_CLOCK_TIMEOUT_PASS`;
- `P4C_COLLISION_MONITOR_VALIDITY_MATRIX_ACCEPTED_AND_CLOSED`.

The validity matrix covered healthy scan and point-cloud recovery, malformed
observations, wrong frame, missing source, topic/type mismatch, lifecycle
inactive/unavailable, duplicate validity publisher attribution, and steady
clock timeout.

## Accepted Runtime Evidence

Generic gate:

- preflight: `P4C_GATE_PREFLIGHT_PASS`;
- fault matrix: `P4C_FAULT_MATRIX_CUMULATIVELY_ACCEPTED_AND_CLOSED`;
- duplicate-output classification: `P4C_DUPLICATE_OUTPUT_RUNNER_ATTRIBUTION_DEFECT`.

E2E stale-observation closure:

- readiness: `P4C_E2E_AUTHORITY_READINESS_PREFLIGHT_PASS`;
- stale closure: `P4C_E2E_STALE_OBSERVATION_GATE_CLOSURE_PASS`;
- raw archive SHA256: `30ff6abb84f3e462c0ab895a51edc081acd47dbdf25b4d660f42545c4891f04f`;
- raw manifest SHA256: `1f3bea93abccd30a01c5728203b0815e4d8d4008041ae178d0a99a2e1e0db51a`;
- raw entry count: `38`;
- last valid scan to collision-valid false: `0.544928737 s`;
- collision-valid false to first gate-owned zero: `0.039173842 s`;
- last valid scan to first gate-owned zero: `0.584102579 s`;
- gate-owned zero heartbeat: `20.003764388 Hz`;
- gate-owned zero duration: `2.249576586 s`;
- later gate-owned nonzero count: `0`;
- nonzero `/cmd_vel_nav_safe` samples after validity false: `46`;
- `/cmd_vel_nav_safe` rate after validity false: `19.998023591 Hz`;
- safe-input maximum age during closure: `0.009914 s`;
- post-rearm nonzero duration: `2.100501569 s`.

## Final Tests And Regression

Final focused tests and build:

- runtime-runner tests:
  `PYTHONPATH=src/vehicle_cmd_safety:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/vehicle_cmd_safety/test/test_phase4_p4c_runtime_runner.py`
  -> `49 passed`;
- full focused P4-C tests:
  `PYTHONPATH=src/vehicle_cmd_safety:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/vehicle_cmd_safety/test`
  -> `91 passed`;
- clean external `vehicle_cmd_safety` build in
  `/tmp/p4c_final_build_20260805T183805Z`: PASS;
- installed package tests from the same external root:
  `91 tests, 0 errors, 0 failures, 0 skipped`.

Cross-project regression:

- `parking_robot_bringup`: `149 passed`;
- `parking_robot_mission_manager`: `50 passed`;
- `plan_nav`: `20 passed`;
- dense plan_nav regression: `47/47 passed`;
- `RouteMission.msg` SHA256:
  `7524239b6aaf17809a63478a1608edf58759d04d1b3ba444641beedd34d8aea4`;
- `MissionState.msg` SHA256:
  `e2b7bf495ad7d92f9695f8b7905af0a4eae2773455afb446bbfd0e8855a4fd90`;
- no Phase 3 mission, topology, or TF authority source diff was present.

## Final Boundaries

P4-C is MOCK/software-only. It does not implement a chassis or wheelchair
adapter, and it does not create a real `/wheelchair_control_command`,
`/wheelchair_control_command_raw`, or `/wheelchair_control_command_mock`
topic.

P4-C did not exercise CAN, vcan, application UDP, hardware, live sensors, or
physical lower-controller paths. Synthetic LaserScan and PointCloud2 messages
were used only to qualify observation validity logic.

`/system/localization_valid` remains a Phase 4 test permission fixture. The
real localization-valid monitor belongs to Phase 5.

P4-D begins mock wheelchair-command conversion and must consume
`/vehicle_cmd_safe`. It must not bypass the Generic Safety Gate.

This document does not claim hardware validation, certified safety, or
physical safety certification.
