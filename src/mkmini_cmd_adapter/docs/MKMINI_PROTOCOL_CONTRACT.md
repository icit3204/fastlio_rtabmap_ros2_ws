# MK-mini offline protocol contract

Authority is the exact manufacturer PDF:

`/home/dog/Downloads/MK_mini阿克曼线控底盘使用手册V3.0.0(1).pdf`

SHA-256: `fe07cffba9ae10f3bc6cf89a92737089231118a296c32444f0bb2fbbc64e72da`

The cover identifies V3.0.0 while internal headers retain V2.1.2. This is
recorded as an internal document-label inconsistency, not silently resolved.

## ctrl_cmd

Extended CAN ID `0x18C4D2D0`, DLC 8, expected period 10 ms, Intel layout:

| Bits | Field | Encoding |
|---|---|---|
| 0..3 | gear | unsigned: 0 disable, 1 P, 2 R, 3 N, 4 D |
| 4..19 | speed | unsigned, 0.001 m/s/bit |
| 20..35 | inner-wheel steering | signed int16, 0.01 degree/bit |
| 36..51 | reserved | zero |
| 52..55 | alive | unsigned 0..15 |
| 56..63 | checksum | Byte0 XOR ... XOR Byte6 |

The speed field is magnitude-only. Reverse uses gear R plus positive speed
magnitude. Left steering is positive and right steering is negative. The
wire steering range is -327.68..327.67 degrees; the documented chassis soft
limit is -34..+34 degrees. The documented running speed is approximately
9.7 km/h; project autonomy limits are not defined here.

Software policy: finite values are quantized to nearest representable step,
with exact half steps rounded away from zero (`ROUND_HALF_UP`). NaN, infinity,
negative speed, invalid gear, and wire overflow are rejected; the codec does
not silently clamp.

Golden payloads include the manufacturer D-gear, 1.0 m/s, and -25 degree
examples in `test/`.

## Feedback implemented

Pure decoders cover `ctrl_fb` (`0x18C4D2EF`), `lr_wheel_fb`
(`0x18C4D7EF`), `rr_wheel_fb` (`0x18C4D8EF`), `Veh_fb_Diag`
(`0x18C4EAEF`), `bms_Infor` (`0x18C4E1EF`), `bms_flag_Infor`
(`0x18C4E2EF`), `ultrasonic_1_fb` (`0x18C4E8EF`),
`ultrasonic_2_fb` (`0x18C4E9EF`), and the documented cumulative-distance
field of `odo_fb` (`0x18C4DEEF`). Wrong ID, non-extended frames, and wrong
DLC are rejected; checksum status is exposed separately where the manual
defines checksum. The odometer table documents only bits 0..31, so its
trailing bytes remain raw and uninterpreted.

`FeedbackFreshnessState`, `AliveTracker`, `DiagnosticState`, and mode/health
assessment helpers are timestamp-injected and transport-neutral. They expose
VALID/INVALID/UNKNOWN and provenance-bearing reasons without publishing
`controller_valid`. Unknown mode or diagnostic semantics are not eligible.

## Explicit non-scope

D0 introduced no ROS node, no `TwistStamped` conversion, and no transport;
D1 now adds only the mock-only ROS wrapper documented below. There is still no
SocketCAN/python-can transport and no hardware authorization. The
professor's `v,w` vehicle-center reference remains
`PENDING_KINEMATIC_REFERENCE_AUTHORITY`.

The manual narrative mentions a parking request, but the `ctrl_cmd` table has
no separate parking field. No synthetic parking bit is implemented; gear P is
the documented ctrl_cmd parking representation.

## Ackermann kinematic contract

`MkminiKinematicsCore` accepts body-frame longitudinal velocity `v` in m/s
(the future `TwistStamped.twist.linear.x`) and body-frame yaw rate `w` in
rad/s (the future `TwistStamped.twist.angular.z`). `v` is not an arbitrary
point's scalar speed. The future base reference remains on the longitudinal
centerline; its exact physical `base_link` location is still pending operator
measurement.

For nonzero longitudinal motion:

`kappa = w / v`, `R = 1 / kappa = v / w`.

For `D = abs(R) - T/2 > 0`, with manufacturer `L=0.600 m` and `T=0.518 m`:

`alpha = sign(kappa) * atan(L / D)`.

`alpha` is the manufacturer inner-wheel angle in degrees: left positive and
right negative. Forward is `v>0`, reverse is `v<0`, and the speed magnitude
is `abs(v)`. Thus forward-left is positive, forward-right negative,
reverse with positive yaw negative, and reverse with negative yaw positive.
`v=0,w=0` is valid stationary; `v=0,w!=0` is invalid because an Ackermann
platform cannot spin in place. The only numerical epsilon is `1e-12` for
zero/division handling; no project command deadband is selected.

The core rejects nonfinite inputs, invalid geometric denominators, and angles
outside the documented -34..+34 degree chassis soft limit without clamping.
The manufacturer minimum-radius consistency check gives rear-axle radius
approximately 1.149 m and outside-front trajectory radius approximately
1.53 m, consistent with the documented approximately 1.5 m specification.

The core returns `Direction` and never selects a stationary CAN gear. Moving
direction mapping (`FORWARD -> D`, `REVERSE -> R`) is tested only as an
offline composition with `MkminiCanCodec` and `MockTransport`; the stationary
gear policy remains deferred. Final `v,w` reference authority and any ROS
adapter remain out of scope.

## Adapter stop/stationary contract (D0)

`MkminiAdapterCore` is a pure, timestamp-injected state machine. It accepts
body-frame `v,w`, a caller-supplied `HealthAssessment`, optional decoded
feedback, and optional `GateContext`; it has no ROS, CAN, wall-clock, or
controller side effects. Moving commands require valid kinematics, explicit
eligible feedback health, and an explicit non-quiescent `ARMED` Gate context.
Forward maps to gear D and reverse maps to gear R, with speed `abs(v)`.

The zero command is always zero speed. After motion it enters
`STOPPING_FORWARD` or `STOPPING_REVERSE`, retains the last D/R direction and
last steering angle, and does not recenter steering immediately. Standstill
requires caller-configured feedback speed evidence and a caller-configured
stability duration. These values default to unset and are not project
authority. Pulse progression is not treated as evidence by this core.

After standstill, production stationary gear remains
`StationaryGearPolicy.UNRESOLVED`; the result is zero-speed with no fabricated
gear/frame and reason `STATIONARY_POLICY_UNRESOLVED`. P, N, DISABLE, and
HOLD_LAST_DIRECTION are explicit test/configuration candidates only. The
physical stopping policy requires live qualification; P is not selected merely
because the manufacturer defines it, and N/DISABLE behavior is not assumed
safe. Feedback faults, stale/deadman input, missing/non-armed Gate context,
and invalid kinematics force zero and never continue a nonzero command.
Recovery does not auto-resume motion: a fresh eligible nonzero command is
required. A future 100 Hz adapter scheduler is separate from Gate input
cadence; each emitted manufacturer frame would advance the alive counter and
recompute checksum. No scheduler or transport is implemented here.

The audited Gate context is `/vehicle_cmd_safety/state`,
`diagnostic_msgs/msg/DiagnosticStatus`: its public core states are represented
by diagnostic level/message/values for `DISARMED`, `ARMED`, and `FAULT`, with
`arm_pending`, `fault_latched`, and `safe_zero_quiescent` diagnostics. A future
ROS adapter should retain `/vehicle_cmd_safe` as its sole motion-command
source and additionally require a fresh, explicit `ARMED`/non-quiescent Gate
context for nonzero output; missing Gate context must fail closed. Gate state
may only further restrict motion or select stationary handling and must never
bypass the Generic Gate.

Project adapter input timeout, standstill threshold, stability duration,
motion limits, stationary gear, acceleration/deceleration, and steering-rate
limits remain `NOT_SELECTED`. Manufacturer CAN cadence is 10 ms (100 Hz),
distinct from future Gate command-input cadence.

## Controller-validity shadow authority (D2A)

`MkminiControllerValidityCore` is a pure, timestamp-injected permission
analysis layer. It requires normalized health evidence for both manufacturer
`ctrl_fb` and `Veh_fb_Diag`; left/right wheel feedback is not required merely
to assert communication/controller health and remains available for
standstill and future plausibility analysis. Missing or unknown feedback,
invalid checksum/alive continuity, stale or reversed timestamps, REMOTE/STOP
or unknown mode, E-stop, and explicit diagnostic faults remain ineligible.

The ROS wrapper `mkmini_controller_validity_node` publishes only the
qualification shadow `/phase5a/mkmini/controller_valid` at a 20 Hz heartbeat,
plus `/phase5a/mkmini/controller_valid/state`. Its default
`NoValidityFeedbackProvider` publishes `false`. Source-freshness and recovery
stability parameters use unresolved sentinels by default, so `true` requires
explicit caller/test configuration and continuous healthy evidence. The
canonical project controller-valid authority is deliberately not migrated.

Generic Gate integration is qualified only with a task-local runtime remap of
its controller-valid subscription to the shadow topic. Gate state remains
restrictive context, Gate remains the sole `/vehicle_cmd_safe` publisher, and
the MK-mini adapter remains below that safe-command boundary. The shadow
producer does not reset or re-arm Gate and does not publish the canonical
controller-valid topic.

## ROS mock adapter integration (D1)

`mkmini_cmd_adapter_node` is the first ROS-facing wrapper and is deliberately
mock-only. It subscribes to `/vehicle_cmd_safe` as
`geometry_msgs/msg/TwistStamped` and to `/vehicle_cmd_safety/state` as
`diagnostic_msgs/msg/DiagnosticStatus`. Both subscriptions use explicit
`RELIABLE`, `VOLATILE`, `KEEP_LAST(depth=1)` QoS. It has no subscriptions to
`/cmd_vel_nav_raw`, `/cmd_vel_nav_safe`, Nav2, or controller topics.

The safe Twist is the sole motion-command input: `linear.x` and `angular.z`
feed the existing core. Nonfinite values and nonzero unsupported components
are rejected fail-closed. Gate identity, state, `arm_pending`,
`fault_latched`, and `safe_zero_quiescent` are parsed explicitly; missing,
malformed, stale, non-ARMED, pending, fault-latched, or quiescent context
cannot enable motion. Gate state is restrictive context only and never
generates movement.

The default `NoFeedbackProvider` is UNKNOWN/ineligible. Tests inject an
in-memory provider; there is no assume-healthy production bypass. Four
safety-critical parameters use `-1.0` unresolved sentinels by default:
`command_timeout_sec`, `gate_state_timeout_sec`,
`standstill_speed_threshold_mps`, and `standstill_stability_sec`. The node
does not emit frames until explicit test/runtime values are supplied.

The node runs a 10 ms timer that synthesizes frames from the latest accepted
physical command and sends them only to the existing `MockTransport`. The
timer rate is independent of safe-command update rate. `AliveCounter` advances
only when a frame is emitted and the established checksum is recomputed by the
codec. Stopping frames retain D/R and steering while speed raw is zero; once
standstill is proven with unresolved stationary gear, frame emission ceases.

Diagnostics are published observationally on `/mkmini_cmd_adapter/state` as
`diagnostic_msgs/msg/DiagnosticStatus`, including state, reason, freshness,
Gate fields, kinematic result, physical command, stationary policy, and mock
frame/alive information. No `controller_valid` authority or full-stack launch
composition is changed. Local ROS integration tests use only in-process
publishers, an injected feedback provider, and MockTransport.
