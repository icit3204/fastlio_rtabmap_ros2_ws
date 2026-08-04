# Phase 4 P4-A.1 Command-Safety Interface Decisions

Status: `P4_READY_AFTER_EXPLICIT_INTERFACE_DECISIONS`

This document freezes the approved Phase 4 command-safety interfaces and
defaults required before P4-B/P4-C implementation. It is documentation-only:
P4-B is not started by this decision record.

## Decision Authority And Baseline

| Item | Frozen value |
| --- | --- |
| Required branch | `main` |
| Required baseline HEAD | `e1675a757f15934c9612fab2a762a164b809c8b8` |
| Required baseline `origin/main` | `e1675a757f15934c9612fab2a762a164b809c8b8` |
| P4-A audit | `docs/phases/phase_04/PHASE_04_P4A_AUTHORITY_AND_INTERFACE_AUDIT.md` |
| P4-A audit SHA256 | `75a7e2eb79a1d7e71373caa4e9a9280b0dfb67eb7193938ce244abbf5a35cb26` |
| Primary authority SHA256 | `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b` |
| Phase 3 closure SHA256 | `169bb31a38319b7719b4c43fe0f712fafcc5a7475ea7db51b51eea578fd88103` |
| Frozen `RouteMission.msg` SHA256 | `7524239b6aaf17809a63478a1608edf58759d04d1b3ba444641beedd34d8aea4` |
| Frozen `MissionState.msg` SHA256 | `e2b7bf495ad7d92f9695f8b7905af0a4eae2773455afb446bbfd0e8855a4fd90` |

The frozen `RouteMission.msg` and `MissionState.msg` interfaces must not be
modified by Phase 4 command-safety work.

## Frozen Topic And Type Matrix

| Topic | Type | Sole approved publisher / ownership | Phase 4 default |
| --- | --- | --- | --- |
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | Nav2 controller output | Allowed |
| `/cmd_vel_nav_safe` | `geometry_msgs/msg/Twist` | Collision Monitor | Allowed |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` | `guarded_vehicle_cmd_gate` | Allowed |
| `/wheelchair_control_command_mock` | `std_msgs/msg/Float32MultiArray` | `wheelchair_cmd_adapter_node` | Allowed |
| `/wheelchair_control_command` | Not approved for default Phase 4 mode | No Phase 4 default publisher | Forbidden |
| `/system/localization_valid` | `std_msgs/msg/Bool` | Deterministic Phase 4 fixture publisher | Allowed |
| `/system/controller_valid` | `std_msgs/msg/Bool` | Deterministic Phase 4 fixture publisher | Allowed |
| `/vehicle_cmd_safety/state` | `diagnostic_msgs/msg/DiagnosticStatus` | `guarded_vehicle_cmd_gate` | Allowed |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard diagnostics publishers | Allowed |
| `/phase4/synthetic_scan` | `sensor_msgs/msg/LaserScan` | Synthetic Phase 4 source | Allowed |
| `/phase4/synthetic_points` | `sensor_msgs/msg/PointCloud2` | Synthetic Phase 4 source | Allowed |

No Phase 4 default launch may start or subscribe to live MID-360, YDLIDAR,
camera, or other physical sensor sources.

## Collision Monitor Target

Phase 4 targets the installed package
`ros-humble-nav2-collision-monitor` version
`1.1.20-1jammy.20260607.134559`.

The approved P4-B command path is:

`/cmd_vel_nav_raw` (`geometry_msgs/msg/Twist`)
-> Collision Monitor
-> `/cmd_vel_nav_safe` (`geometry_msgs/msg/Twist`)

P4-B must use the exact installed local Collision Monitor schema. The old live
`/cmd_vel_nav` -> `/cmd_vel` block is not command authority and must not be
reused as authority.

## Generic Gate APIs

Package: `vehicle_cmd_safety`

Node: `guarded_vehicle_cmd_gate`

Input:

| Topic | Type |
| --- | --- |
| `/cmd_vel_nav_safe` | `geometry_msgs/msg/Twist` |

Output:

| Topic | Type | Frame |
| --- | --- | --- |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` | `base_footprint` |

Arm service:

| Service | Type |
| --- | --- |
| `/vehicle_cmd_safety/arm` | `std_srvs/srv/SetBool` |

`data=true` requests arming. The request succeeds only when all prerequisites
are healthy and stable. A rejected arm request returns `success=false` with the
exact reason.

`data=false` immediately disarms the gate.

## Mode And State Model

Operating mode and gate state are separate concepts.

Phase 4 supports `MOCK` mode only.

Approved state values:

| State | Meaning |
| --- | --- |
| `DISARMED` | Gate is not armed and outputs repeated zero commands. |
| `ARMED` | Gate may forward bounded, valid safe commands. |
| `FAULT` | A latched fault is active; the gate outputs repeated zero commands. |
| `MANUAL_OVERRIDE` | Representable only; remains unknown/unasserted unless a real source exists. |

State topic:

| Topic | Type |
| --- | --- |
| `/vehicle_cmd_safety/state` | `diagnostic_msgs/msg/DiagnosticStatus` |

Standard diagnostics:

| Topic | Type |
| --- | --- |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

State reason, detail, command age, permission ages, permission values,
authority status, and configured mode are carried as `DiagnosticStatus`
key-value entries. No separate fault topic is approved.

## Permission APIs

| Topic | Type | Freshness clock | Phase 4 source |
| --- | --- | --- | --- |
| `/system/localization_valid` | `std_msgs/msg/Bool` | Steady/monotonic receipt time | Deterministic fixture publisher |
| `/system/controller_valid` | `std_msgs/msg/Bool` | Steady/monotonic receipt time | Deterministic fixture publisher |

The actual localization-valid monitor is Phase 5. The actual
health/controller supervisor remains later work.

## Timing Defaults

| Decision | Frozen value |
| --- | --- |
| Generic gate output heartbeat | `20 Hz` |
| Safe Twist receipt timeout | `0.25 s` |
| Localization permission timeout | `0.50 s` |
| Controller permission timeout | `0.50 s` |
| Authority stability before arming | `1.0 s` |
| ROS graph polling target | `2 Hz` |
| Mock adapter independent deadman | `0.25 s` |
| Mock stop heartbeat | `20 Hz` |

All watchdog and deadman decisions use steady/monotonic receipt time. ROS
message time is used for output stamping and traceability, not as the sole
deadman clock.

## Arming Semantics

Startup is `MOCK` mode, `DISARMED`, with repeated zero output.

An arm request (`data=true`) succeeds only when all prerequisites are healthy
and stable:

- finite and supported command stream;
- fresh safe command;
- fresh and true localization permission;
- fresh and true controller permission;
- no authority conflict;
- valid frame and configuration;
- no internal timing fault;
- explicit Phase 4 YAML has valid positive motion limits that are no greater
  than the frozen Candidate C source-defined limits;
- authority has been stable for `1.0 s`.

A disarm request (`data=false`) immediately disarms and forces repeated zero.

Fault recovery sequence:

1. `arm=false`
2. fault cause removed
3. prerequisites stable
4. `arm=true`

## Latched-Fault Semantics

The following conditions latch `FAULT` and force repeated zero output:

- NaN;
- infinity;
- unsupported nonzero axes;
- stale safe command;
- stale or false localization permission;
- stale or false controller permission;
- authority conflict;
- invalid frame/configuration;
- internal timing fault.

## Authority-Conflict Detection

No separate authority-valid input topic is approved.

The gate must internally inspect ROS graph publisher information. Before
arming and continuously while armed, the gate requires:

- exactly one publisher on `/cmd_vel_nav_safe`;
- exactly one publisher on `/vehicle_cmd_safe`, namely the gate.

An external independent evidence monitor must also record topic type,
publisher GID, node name, and publisher-count changes.

Authority must be stable for `1.0 s` before arming. The graph polling target
is `2 Hz`.

## Fail-Safe Motion-Limit Policy

Do not define permissive motion defaults. Node defaults must deny motion:

| Limit | Default |
| --- | --- |
| Maximum linear limit | `0` |
| Maximum angular limit | `0` |
| Slew/acceleration limits | `0` |

The gate refuses arming until an explicit Phase 4 YAML provides valid positive
limits.

Phase 4 configured limits must be equal to or stricter than the exact frozen
Candidate C values where Candidate C defines them. Candidate C must not be
modified or tuned by Phase 4 command-safety work.

Initial Phase 4 policy:

- reverse disabled;
- nonzero `linear.y` and `linear.z` rejected;
- nonzero `angular.x` and `angular.y` rejected;
- in-place rotation rejected;
- SI units only;
- no silent sign or unit reinterpretation.

## Frozen Candidate C Values

The frozen source inspected for Phase 4 command limits is
`src/parking_robot_bringup/config/phase2_nav2_params.yaml` at baseline
`e1675a757f15934c9612fab2a762a164b809c8b8`.

The current tracked source SHA256 is
`9d435d52681bbf841e32de3c9a9071d5d3fad8228c44667726b3e91be700540d`.
Earlier recovered Phase 2 evidence records Candidate C SHA256
`6e206a332f0a0d5dd61ca8691325267014f5455d6a9ab238f716fcbd9aa0f842`; this
document does not modify Candidate C and records the motion values present in
the current frozen baseline source.

Source-defined MPPI command/motion values:

| Candidate C key | Source-defined value | Phase 4 implication |
| --- | --- | --- |
| `controller_frequency` | `20.0` | Nav2 controller output cadence reference |
| `motion_model` | `DiffDrive` | No lateral command support |
| `vx_max` | `0.25` | Phase 4 positive linear.x limit must be `<= 0.25 m/s` |
| `vx_min` | `0.0` | Reverse disabled |
| `vy_max` | `0.0` | `linear.y` must remain zero |
| `wz_max` | `0.8` | Phase 4 angular.z limit must be `<= 0.8 rad/s` |
| `time_steps` | `40` | Controller profile value only; not a gate slew limit |
| `model_dt` | `0.05` | Controller profile value only; not a gate slew limit |
| `batch_size` | `500` | Controller profile value only |
| `iteration_count` | `1` | Controller profile value only |
| `prune_distance` | `1.0` | Controller profile value only |
| `transform_tolerance` | `0.5` | Controller profile value only |
| `vx_std` | `0.10` | Sampling value only; not a gate speed or slew limit |
| `vy_std` | `0.0` | Sampling value only; reinforces no lateral command |
| `wz_std` | `0.20` | Sampling value only; not a gate speed or slew limit |
| `PathAngleCritic.forward_preference` | `true` | Reinforces forward-only policy |

Candidate C does not explicitly define a required gate acceleration or slew
limit. No Candidate C acceleration/slew value is invented by this decision
record.

Candidate C also includes behavior-server rotational values
`max_rotational_vel: 0.8`, `min_rotational_vel: 0.2`, and
`rotational_acc_lim: 1.0`; these belong to Nav2 recovery behaviors, not the
Phase 4 generic command gate. They are not adopted as gate defaults.

## Mock Wheelchair Adapter Contract

Package: `wheelchair_cmd_adapter`

Node: `wheelchair_cmd_adapter_node`

Input:

| Topic | Type |
| --- | --- |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` |

Output:

| Topic | Type |
| --- | --- |
| `/wheelchair_control_command_mock` | `std_msgs/msg/Float32MultiArray` |

The Phase 4 adapter must not create a publisher for
`/wheelchair_control_command`.

Recovered converter material policy: `REUSE_TEST_VECTORS_ONLY`.

`VehicleState` is deferred as `P4_LATER_SUBTASK`.

## Independent Adapter Deadman

The mock adapter has an independent steady-clock deadman of `0.25 s`.
Startup and shutdown behavior is repeated mock stop. The mock stop heartbeat
is `20 Hz`.

## Synthetic Source Topics

Primary synthetic Collision Monitor source:

| Topic | Type |
| --- | --- |
| `/phase4/synthetic_scan` | `sensor_msgs/msg/LaserScan` |

Secondary synthetic Collision Monitor source:

| Topic | Type |
| --- | --- |
| `/phase4/synthetic_points` | `sensor_msgs/msg/PointCloud2` |

No Phase 4 default launch may start or subscribe to live MID-360, YDLIDAR,
camera, or other physical sensor sources.

## Phase 4 Boundaries

| Phase | Boundary |
| --- | --- |
| P4-B | Nav2 raw Twist -> Collision Monitor -> collision-safe Twist with synthetic LaserScan and PointCloud2 cross-check. |
| P4-C | Generic Safety Gate and independent authority monitor. |
| P4-D | Mock Wheelchair Command Adapter and independent deadman. |
| P4-E | Full fake closed-loop integration, stop/slow, no-bypass, and bounded blocked/progress behavior. |
| P4-F | Regression and Phase 4 closure. |

P4-A.1 does not implement P4-B and does not add packages, nodes, launch files,
YAML, interfaces, or tests.

## Supersession And Change Control

These decisions are frozen for Phase 4.

Any later change to topic names, message types, timing defaults, state
semantics, or ownership requires:

- explicit Main ChatGPT review;
- documented reason;
- regression impact assessment;
- a separate decision update before implementation.
