# Phase 4 P4-D Mock Wheelchair Command Adapter

## Scope and ownership

P4-D adds the `wheelchair_cmd_adapter` ROS 2 package and its production node,
`mock_wheelchair_cmd_adapter`. This is a mock, software-only chassis boundary.
The generic safety boundary remains owned by `vehicle_cmd_safety`:

```
guarded_vehicle_cmd_gate
  -> /vehicle_cmd_safe (geometry_msgs/msg/TwistStamped, base_footprint)
  -> mock_wheelchair_cmd_adapter
  -> /wheelchair_control_command_mock (std_msgs/msg/Float32MultiArray)
```

The adapter subscribes only to `/vehicle_cmd_safe`. Its only wheelchair-command
publication is `/wheelchair_control_command_mock`; diagnostics are published on
`/wheelchair_cmd_adapter/diagnostics`. Topic parameters are validated before ROS
endpoints are created, so the input and mock output cannot be redirected. The
Generic Safety Gate remains unchanged and is the sole intended publisher of
`/vehicle_cmd_safe`; there is no adapter bypass from navigation command topics.

The package launch file starts only `mock_wheelchair_cmd_adapter`. Qualification
runners are explicitly named `phase4_p4d*`; the integrated runner is under
`test/`. They are not production launch entry points.

## Legacy contract and conversion

P4-D.0 traced the legacy `std_msgs/msg/Float32MultiArray` producer contract as:

```
[radius_mm, velocity_mm_s, distance_mm]
```

The P4-D adapter always emits exactly three floating-point elements and always
sets `array[2]` to exactly `0.0`. This reproduces the legacy producer convention;
it does not validate physical chassis behavior.

| Input | Mock output |
|---|---|
| stop | `[0.0, 0.0, 0.0]` |
| straight | `[10000.0, linear.x * 1000.0, 0.0]` |
| supported curve | `[-(linear.x / angular.z) * 1000.0, linear.x * 1000.0, 0.0]` |

At the legacy array boundary positive radius denotes right and negative radius
denotes left. Therefore positive ROS `angular.z` produces a negative array
radius and negative `angular.z` produces a positive radius. This sign is only a
reproduction of the legacy producer convention. Physical chassis turn direction
is unverified.

The minimum supported absolute radius is `1000 mm`. An absolute radius of
`10000 mm` or greater is represented canonically as straight (`10000 mm`). The
accepted limits are `linear.x <= 0.20 m/s` and `abs(angular.z) <= 0.50 rad/s`.
Reverse and in-place rotation are rejected to repeated zero.

## Validation, deadman, heartbeat, and authority

Input must be a finite `TwistStamped` in `base_footprint`. Non-finite values,
unsupported linear or angular axes, reverse, in-place motion, an unsupported
tight radius, or an over-limit command evaluate to the stop array. Startup,
missing input, stale input, invalid input authority, and invalid output authority
also evaluate to the stop array.

Receipt freshness uses an explicitly retained `STEADY_TIME` clock and a `0.25 s`
timeout. The `20 Hz` heartbeat and graph polling timers use their own retained,
explicit `STEADY_TIME` clocks. Thus ROS time freeze does not freeze safety
scheduling or input ageing. Every heartbeat publishes the current conversion or
another mock zero. When a fault disappears, a fresh valid input recovers
automatically; there is no hidden adapter latch.

Input authority requires exactly one publisher on `/vehicle_cmd_safe`. Output
authority requires aggregate publisher count one on
`/wheelchair_control_command_mock`. Duplicate-output analysis distinguishes the
adapter-owned publisher from aggregate topic publishers: the adapter owns one,
while an aggregate count other than one forces repeated zero.

## Standalone qualification

P4-D.1 basic preflight completed as
`P4D1_MOCK_ADAPTER_BASIC_PREFLIGHT_PASS`. P4-D.2 accepted the startup/missing
input, stale/deadman, frame, NaN, Inf, unsupported-axis, reverse, in-place,
tight-radius, over-limit, input-authority, output-authority, recovery, and
process-exit matrix as `P4D2_MOCK_ADAPTER_FAULT_MATRIX_ACCEPTED`.

Frozen-time qualification passed independently for liveness, deadman, and
authority:

- `P4D2_MOCK_ADAPTER_FROZEN_TIME_LIVENESS_PASS`
- `P4D2_MOCK_ADAPTER_FROZEN_TIME_DEADMAN_PASS`
- `P4D2_MOCK_ADAPTER_FROZEN_TIME_AUTHORITY_PASS`

## Gate-to-adapter qualification

P4-D.3 exercised the complete chain from `/cmd_vel_nav_safe`, through the
Generic Safety Gate and `/vehicle_cmd_safe`, to the mock array topic. It passed
readiness, straight/left/right conversion, collision-permission fault
propagation to mock zero, the gate's explicit fault-clear and re-arm sequence,
automatic adapter recovery, and the adapter's independent deadman after gate
loss:

- `P4D3_GATE_ADAPTER_CHAIN_READINESS_PASS`
- `P4D3_GATE_ADAPTER_VALID_CONVERSION_PASS`
- `P4D3_GATE_FAULT_TO_MOCK_ZERO_PASS`
- `P4D3_EXPLICIT_RECOVERY_AND_REARM_PASS`
- `P4D3_GATE_LOSS_ADAPTER_DEADMAN_PASS`

Accepted measurements were: disarmed gate/mock zero `20.008753/19.995135 Hz`;
stable straight gate/mock `20.001942/19.989542 Hz`; left/right gate-to-adapter
latency `0.014266954/0.014670647 s`; final collision-permission receipt to mock
zero `0.551347560 s`; gate zero to mock zero `0.020688102 s`; fault mock-zero
heartbeat `20.035960 Hz` for `2.295872 s`; recovered mock output `20.000549 Hz`
for `2.349936 s`; gate-loss final receipt to adapter zero `0.111473169 s`; and
gate-loss zero `19.995917 Hz` for `2.700551 s`.

## Final verification

P4-D final closure passed the source adapter suite (`27 passed`), installed
adapter suite (`27 passed`, zero failures/errors), P4-C runtime runner (`49
passed`), P4-C focused suite (`91 passed`), external builds of
`vehicle_cmd_safety` and `wheelchair_cmd_adapter`, bringup regression (`149
passed`), Mission Manager (`50 passed`), plan_nav (`20 passed`), and the dense
plan_nav regression (`47/47 passed`). Frozen interface checksums remained:

- RouteMission: `7524239b6aaf17809a63478a1608edf58759d04d1b3ba444641beedd34d8aea4`
- MissionState: `e2b7bf495ad7d92f9695f8b7905af0a4eae2773455afb446bbfd0e8855a4fd90`

These results preserve Phase-3 mission behavior, topology behavior, TF
authority behavior, and accepted P4-C behavior.

## Cumulative evidence ledger

| Stage | Result | Raw archive SHA256 | Raw manifest SHA256 | Entries |
|---|---|---|---|---:|
| P4-D.0 contract audit | `P4D0_WHEELCHAIR_COMMAND_CONTRACT_AUDIT_ACCEPTED` | `19c3605a4566ca1a7300bce222ae8c6cb5d343ff82a72f52b09dd26efd5a685c` | `b39ed2a2ccb122024b8d208830ebed361e1734d29b9f6c3310b379afccb5b6c7` | 21 |
| P4-D.1 basic adapter | `P4D1_MOCK_ADAPTER_BASIC_PREFLIGHT_PASS` | `556f7afeabd7ba63aeb3f74718204b463f2daa5b6ced7ccf0473394b2fb7c5fa` | `ed7d81254c84039ef1e046b1e07af55d40bf5d828612a5f78f84326af2f24983` | 207 |
| P4-D.2 fault matrix | `P4D2_MOCK_ADAPTER_FAULT_MATRIX_ACCEPTED` | `40d25a9b98b93b2f0f66bf58211623650f39d9c857e5b9d7cfa5b13c87484805` | `0382fda500db75f1addfbeafb4bb7f72097e8795911fa0b2c4a990176544e2bb` | 132 |
| P4-D.3 integrated chain | accepted five-status chain above | `b0ca288c767a184a3ca3f3c9566e2352463086f08c8bf8b0999aaee279ca71e9` | `4636ae2009a236db0d4c6ab88430e02b7712d3162f4aed003c76ca984bcd96d7` | 117 |

The authoritative Markdown and JSON handoffs remain in
`/home/dog/phase4_reports/`.

## Limitations

1. P4-D is mock/software-only.
2. `/wheelchair_control_command_mock` is not a hardware topic.
3. `/wheelchair_control_command` is not created.
4. The existing wheelchair controller is not launched.
5. CAN, vcan, and UDP are not accessed.
6. No physical motion or chassis direction is proven.
7. The radius sign reproduces the legacy producer convention only.
8. In-place rotation and reverse remain blocked.
9. `/system/localization_valid` remains a Phase-4 fixture.
10. A future real adapter must remain downstream of `/vehicle_cmd_safe` and must not bypass the Generic Safety Gate.
11. This work is not physical safety certification.
