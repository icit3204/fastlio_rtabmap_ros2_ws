# Phase 4 P4-A Authority And Interface Audit

## Status

P4-A is a read-only architecture and interface audit. It does not implement, launch, or validate the Phase 4 runtime chain.

Implementation-readiness classification:

`P4_READY_AFTER_EXPLICIT_INTERFACE_DECISIONS`

Installed Collision Monitor schema classification:

`INSTALLED_HUMBLE_COLLISION_MONITOR_SCHEMA_CONFIRMED`

## Authority Hierarchy

| Priority | Source | Scope | Status | Phase 4 facts | Limitations |
| --- | --- | --- | --- | --- | --- |
| 1 | V3.1 Phase-0 authority, `/home/dog/Downloads/FINAL_REVISED_ARCHITECTURE_AND_IMPLEMENTATION_STRATEGY_AFTER_DAY0_V3_1_FINAL_PHASE0_AUTHORITY.md` | Target architecture and phase acceptance | Current primary authority | Defines `MPPI -> Collision Monitor -> Generic Command Safety Gate -> Wheelchair Command Adapter`, strict topic separation, mock-first behavior, arming/freshness gates, and later Phase 5 localization freshness | Some public ROS API names/types for the generic gate are conceptual, not fully frozen |
| 2 | Current repository source | What software presently does | Current implementation authority | Shows Phase 2 Nav2 remaps, Phase 3 Mission Manager, plan_nav mission mode, legacy wheelchair controller and Pure Pursuit code | Does not yet contain Phase 4 gate or adapter packages |
| 3 | Phase 3 closure, `docs/phases/phase_03/PHASE_03_MISSION_AND_PLAN_NAV_CLOSURE.md` | Validated mission integration | Current validation authority | Confirms typed mission route reaches actual Nav2 in fake/mock mode and that Phase 3 excludes Collision Monitor, Generic Gate, VehicleState, CAN and wheelchair actuation | Scenario scope is fake/mock; no obstacle or safety gate validation |
| 4 | Phase 2 closure/evidence | Frozen Nav2 fake/mock baseline | Current validation authority | Confirms Candidate C NavFn/MPPI fake-base baseline and the cached-transform freshness limitation deferred to Phase 4/5 | Does not validate command safety chain |
| 5 | Phase 1 module registry and command-authority reconciliation | Existing module inventory and recovered evidence classification | Current inventory authority | Records Collision Monitor package version and identifies Generic Gate, adapter, `/vehicle_cmd_safe`, and `VehicleState` as not implemented | Phase 1 did not close Phase 4 behavior |
| 6 | Recovered semantic/guarded bridge references | Design and test-vector evidence | Reference only | May inform conversion and safety-gate tests | Not authoritative implementation source unless explicitly reintroduced later |
| 7 | Historical live Collision Monitor blocks | Legacy launch evidence | Superseded for Phase 4 target chain | Show older `/cmd_vel_nav -> /cmd_vel` construction | Not the target chain and does not protect wheelchair commands |

Contradictions and reconciliations:

- Old `bringup_2d` constructs a Collision Monitor around `/cmd_vel_nav -> /cmd_vel`, but that block is commented out and is not the Phase 4 target. The V3.1 chain supersedes it with `/cmd_vel_nav_raw -> /cmd_vel_nav_safe -> /vehicle_cmd_safe`.
- Historical P3-A topic names under `/mission_manager/*` are superseded by P3-A.1/P3-A.2 `/mission/*` APIs.
- Recovered/live wheelchair conversion code is reference evidence only. Phase 4 must keep generic SI safety above `/vehicle_cmd_safe` and chassis-specific conversion below it.

## Installed Collision Monitor Schema

Local installed package:

- Package: `nav2_collision_monitor`
- Version: `1.1.20`
- Prefix: `/opt/ros/humble`
- Executable: `nav2_collision_monitor collision_monitor`
- Launch file: `/opt/ros/humble/share/nav2_collision_monitor/launch/collision_monitor_node.launch.py`
- Default params: `/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml`

Installed package evidence:

- `package.xml` declares `nav2_collision_monitor` version `1.1.20` and dependencies on `rclcpp`, `rclcpp_components`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `sensor_msgs`, `geometry_msgs`, `nav2_common`, `nav2_util`, and `nav2_costmap_2d`.
- `collision_monitor_node.hpp` defines a `nav2_util::LifecycleNode` with lifecycle configure/activate/deactivate/cleanup/shutdown callbacks.
- `cmdVelInCallback` consumes `geometry_msgs::msg::Twist`; the lifecycle publisher also emits `geometry_msgs::msg::Twist`. No installed header evidence supports TwistStamped command I/O for this Humble package.
- Default parameters include `base_frame_id`, `odom_frame_id`, `cmd_vel_in_topic`, `cmd_vel_out_topic`, `transform_tolerance`, `source_timeout`, `base_shift_correction`, and `stop_pub_timeout`.
- The installed comment for `publishVelocity` states that if the robot has been stopped longer than `stop_pub_timeout_`, Collision Monitor quits publishing zero velocity. Therefore Phase 4 must not rely on Collision Monitor alone for repeated zero output; that belongs in the Generic Command Safety Gate and adapter deadman.

Installed parameter support:

| Parameter or feature | Local evidence | P4-A conclusion |
| --- | --- | --- |
| Input command topic | `cmd_vel_in_topic`, default `cmd_vel_raw` | Supported, target `/cmd_vel_nav_raw` |
| Output command topic | `cmd_vel_out_topic`, default `cmd_vel` | Supported, target `/cmd_vel_nav_safe` |
| Command type | `geometry_msgs/msg/Twist` | Confirmed; gate converts later to `TwistStamped` |
| Base frame | `base_frame_id`, default `base_footprint` | Supported |
| Odom frame | `odom_frame_id`, default `odom` | Supported |
| Transform tolerance | `transform_tolerance`, default `0.5` | Supported |
| Source timeout | `source_timeout`, default `5.0` | Supported |
| Stop publishing timeout | `stop_pub_timeout`, default `2.0` | Supported; repeated zeros are not guaranteed forever |
| Source types | `scan`, `pointcloud`, `range` installed headers | LaserScan, PointCloud2, Range supported locally |
| Polygon/action types | `STOP`, `SLOWDOWN`, `APPROACH` installed enum | Stop, slowdown and approach supported locally |
| Minimum points | `max_points` polygon parameter | Supported as maximum points allowed before action |
| Slowdown ratio | `slowdown_ratio` polygon parameter | Supported for slowdown |
| Time before collision | `time_before_collision` and `simulation_time_step` | Supported for approach |
| Source enabled | `enabled` source and polygon parameters | Supported |
| Lifecycle behavior | Installed launch runs Collision Monitor with a lifecycle manager | Lifecycle-managed |

The installed Debian package provides public headers and YAML/launch files, but not the full `.cpp` source. P4-B should treat the local schema as confirmed while validating runtime details with isolated fake tests before using it as safety evidence.

## Current Command-Topic Inventory

| Topic | Message type | Publisher source | Subscriber source | Launch condition | Current phase use | Classification | Physical risk | Target Phase 4 disposition |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 default before remap; legacy inputs | Pure Pursuit legacy subscribes in `pure_pursuit_controller.cpp` | Legacy bringup | Legacy Nav2/Pure Pursuit coupling | Legacy/new boundary | Medium if bridged to Pure Pursuit | Do not expose as Phase 4 authority |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 in legacy `bringup_2d` remap | Old commented Collision Monitor input; Pure Pursuit may consume `/cmd_vel` separately | Legacy bringup | Legacy Nav2 command | Legacy | Medium | Superseded by `/cmd_vel_nav_raw` |
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | None currently | None currently | Not implemented | Target raw MPPI output | New | Low until wired | P4-B target controller output |
| `/cmd_vel_nav_safe` | `geometry_msgs/msg/Twist` | None currently | None currently | Not implemented | Target Collision Monitor output | New | Low until wired | P4-B/P4-C input to Generic Gate |
| `/cmd_vel_phase2_mock` | `geometry_msgs/msg/Twist` | Phase 2 controller and behavior servers via remap | `phase2_fake_base` | Phase 2 launch | Frozen fake/mock Nav2 baseline | Mock | Low | Keep for Phase 2 regression only |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` | None currently | None currently | Not implemented | Target generic-safe command | New | Low until wired | P4-C gate output and P4-D adapter input |
| `/wheelchair_control_command_mock` | `std_msgs/msg/Float32MultiArray` | None currently | None currently | Not implemented | Target mock chassis-formatted output | Mock | Low | P4-D output; no CAN subscriber allowed |
| `/wheelchair_control_command_raw` | `std_msgs/msg/Float32MultiArray` | Pure Pursuit in laser-avoidance mode | `laser_command_safety_filter` | Legacy bringup | Legacy app-side safety filter | Legacy | High if real controller active | Forbidden in new Phase 4 mode |
| `/wheelchair_control_command` | `std_msgs/msg/Float32MultiArray` | Pure Pursuit no-avoidance, laser filter, future real adapter | `wheelchair_controller_node` | Legacy bringup / real controller | Real chassis command input | Real | High | Must not exist in default Phase 4 mock mode |
| `/plan_nav` | `nav_msgs/msg/Path` | plan_nav dense display/legacy output | Pure Pursuit legacy subscriber | plan_nav legacy/display | Dense path visualization and legacy authority | Legacy/display | Medium if Pure Pursuit active | Display-only in mission_nav2 |

Current Phase 2 command behavior:

- `phase2_core_nav2.launch.py` declares `cmd_vel_topic` default `/cmd_vel_phase2_mock`.
- It remaps `controller_server` and `behavior_server` `/cmd_vel` to that topic.
- `phase2_fake_base.py` subscribes `/cmd_vel_phase2_mock`, publishes `/Odometry`, rejects non-finite commands, and is conditioned by `start_fake_base`.
- Candidate C sets `controller_frequency: 20.0`.

Minimum safe P4 remap to obtain `/cmd_vel_nav_raw` without tuning Candidate C:

- Use a Phase-4-only launch wrapper or launch argument to remap Nav2 `/cmd_vel` to `/cmd_vel_nav_raw`.
- Do not change Candidate C parameters.
- Keep behavior-server command output accounted for in publisher attribution. If behavior-server publishes zero commands on the same raw topic, P4-B must decide whether raw-topic sole ownership means one nonzero authority or exactly one ROS publisher.

## Current Versus Target Chain

Current frozen fake/mock Phase 2 chain:

`Nav2 controller_server / behavior_server -> /cmd_vel_phase2_mock -> phase2_fake_base -> /Odometry`

Target Phase 4 chain:

`MPPI /cmd_vel_nav_raw -> Collision Monitor -> /cmd_vel_nav_safe -> guarded_vehicle_cmd_gate -> /vehicle_cmd_safe -> mock wheelchair_cmd_adapter -> /wheelchair_control_command_mock`

Non-negotiable target properties:

- Raw, Collision-Monitor-safe, generic-safe, and chassis-mock topics stay separate.
- No adapter subscribes to raw MPPI or Collision-Monitor-safe Twist directly.
- Wheelchair-specific units and radius semantics exist only below `/vehicle_cmd_safe`.
- Default Phase 4 launches must make real `/wheelchair_control_command`, CAN, and application UDP impossible.

## Topic, Type And Ownership Matrix

| Target topic | Type | Sole allowed publisher | Allowed subscribers | Forbidden publishers | Mock/real | Lifecycle owner | Required runtime check |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | Nav2 controller path, with explicit treatment of behavior-server zero publisher | Collision Monitor, monitors only | plan_nav, Pure Pursuit, adapters, tests after readiness | Fake/mock Nav2 raw | Nav2 lifecycle | Continuous publisher GID/node/type check |
| `/cmd_vel_nav_safe` | `geometry_msgs/msg/Twist` | Collision Monitor | Generic Gate, monitors only | Nav2, plan_nav, adapter | Collision-safe command | Collision Monitor lifecycle | Continuous publisher GID/node/type check |
| `/vehicle_cmd_safe` | `geometry_msgs/msg/TwistStamped` | `guarded_vehicle_cmd_gate` | mock adapter, monitors only | Collision Monitor, Nav2, plan_nav, legacy controllers | Generic-safe command | Gate lifecycle | Continuous publisher GID/node/type check |
| `/wheelchair_control_command_mock` | `std_msgs/msg/Float32MultiArray` | mock wheelchair adapter | mock recorder/test sink only | Pure Pursuit, laser filter, real controller | Mock chassis format | Adapter lifecycle | Continuous publisher/subscriber and no-real-remap check |
| `/wheelchair_control_command` | `std_msgs/msg/Float32MultiArray` | No publisher in default Phase 4 | No subscriber in default Phase 4 | All Phase 4 fake-mode processes | Real chassis input | Future real adapter only | Must be absent in default Phase 4 |

ROS graph inspection alone is insufficient because Phase 2/P3 evidence required publisher GID/node attribution over time. Phase 4 should reuse the C++ publisher-GID monitor pattern and extend it with process authority, topic type, and continuous conflict detection.

## Collision Monitor Reuse Classification

| Path or source | Active now | Input | Output | Source | Known defects | Reuse classification |
| --- | --- | --- | --- | --- | --- | --- |
| `/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml` | Installed reference | `cmd_vel_raw` | `cmd_vel` | LaserScan default; PointCloud2 example | Generic sample topics, not robot-specific | `REUSE_AS_TEST_REFERENCE` |
| `/opt/ros/humble/share/nav2_collision_monitor/launch/collision_monitor_node.launch.py` | Installed reference | Configurable | Configurable | Configurable | Autostart default and sample params need P4 wrapper control | `REUSE_AS_TEST_REFERENCE` |
| `src/robot_bringup/launch/bringup_2d.launch.py` Collision Monitor block | Constructed but commented out | `/cmd_vel_nav` | `/cmd_vel` | `/cloud_registered_body` PointCloud2 | Not launched, old routing, does not protect `/wheelchair_control_command` | `LEGACY_ONLY` |
| Phase 1 Collision Monitor verification evidence | Evidence only | Installed schema | Installed schema | Installed schema | Not Phase 4 runtime target | `REUSE_AS_AUTHORITY` for package/version finding, `REUSE_AS_TEST_REFERENCE` for approach |
| Recovered/live Collision Monitor blocks | Historical | Varies | Varies | Varies | Old live-source assumptions | `REWRITE_FOR_PHASE4` |

## Synthetic Obstacle Source Recommendation

| Choice | Support | Determinism | TF needs | Stale-data test | Stop/slow test | Live-sensor risk | Recommendation |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Synthetic LaserScan | Installed `scan` source | High for 2D zones | Simple scanner frame to base | Straightforward by stopping scan publisher | Good for stop/slow polygons | Low | Primary P4-B source |
| Synthetic PointCloud2 | Installed `pointcloud` source | High with fixed cloud points | Moderate; height filters matter | Good | Good; closer to later 3D sensor path | Low if synthetic topic only | Secondary cross-check |
| Synthetic Range | Installed `range` source | High but narrow field | Simple | Good | Limited geometry coverage | Low | Supplemental only |
| Replayed observation source | Depends on bag | Medium | Bag-specific | Good | Good if curated | Medium if sourced from live sensor topics | Later regression, not primary |

Use synthetic LaserScan first. Use synthetic PointCloud2 as the secondary cross-check. Do not use `/cloud_registered_body` for fake Phase 4 just because it appears in legacy launch files; live MID-360 input remains Phase 5/6.

## Generic Command Safety Gate Contract

Recommended package and node:

- Package: `vehicle_cmd_safety`
- Node: `guarded_vehicle_cmd_gate`

Input:

- `/cmd_vel_nav_safe`
- `geometry_msgs/msg/Twist`

Output:

- `/vehicle_cmd_safe`
- `geometry_msgs/msg/TwistStamped`

Output contract:

- `header.stamp` is the gate acceptance/publication ROS time.
- `header.frame_id` is normally `base_footprint`.
- `linear.x` is metres/second.
- `angular.z` is radians/second.
- `linear.y`, `linear.z`, `angular.x`, and `angular.y` are zero.
- All accepted values are finite.
- No sign, unit, or radius reinterpretation occurs.
- Reverse and in-place rotation policy must be explicit.
- Watchdog and freshness decisions use steady/monotonic receipt time, not ROS time.

Requirement classification:

| Requirement | Classification | Notes |
| --- | --- | --- |
| MOCK/DISARMED/ARMED/FAULT/MANUAL_OVERRIDE state model | `NOT_IMPLEMENTED` | Authority concept exists; public API still open |
| Startup zero | `NOT_IMPLEMENTED` | Must repeat zero before arming |
| Shutdown zero | `NOT_IMPLEMENTED` | Must be bounded and observable |
| Repeated-zero heartbeat | `NOT_IMPLEMENTED` | Must not depend on Collision Monitor's stop timeout |
| Command freshness | `NOT_IMPLEMENTED` | Must use steady receipt time |
| Localization-valid permission | `INTERFACE_UNDEFINED` / `DEFERRED_TO_PHASE5` | P4 needs a fixture input; real monitor is Phase 5 |
| Nav2/controller-valid permission | `INTERFACE_UNDEFINED` | Needs P4 decision |
| Explicit arming | `INTERFACE_UNDEFINED` | Must be approved before P4-C |
| Numerical validation | `NOT_IMPLEMENTED` | Reject NaN, Inf, lateral/vertical/roll/pitch |
| Generic speed limits | `INTERFACE_UNDEFINED` | Defaults need approval |
| Acceleration/slew limits | `INTERFACE_UNDEFINED` | Defaults and calculation window need approval |
| Unique publisher ownership | `PARTIALLY_IMPLEMENTED` | Phase 2/3 monitors are reusable evidence; gate behavior not implemented |
| Diagnostics | `INTERFACE_UNDEFINED` | DiagnosticStatus likely acceptable but not frozen |

Phase boundary:

- Phase 4 must include a localization-permission test fixture and must suppress commands when that permission is false.
- Phase 5 implements the real localization-valid/freshness monitor for `/Odometry`, `map -> odom`, complete `map -> base_footprint`, finite pose/quaternion, jump policy, and stability policy.

## Open Interface Register

Every row marked `P4_INTERFACE_DECISION_REQUIRED` needs explicit approval before the relevant implementation subtask.

| Interface or parameter | Authority requirement | Existing definition | Missing decision | Safe options | Recommended option | Owner | Decide before |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Arm/disarm API | Explicit arming required | None | Name and type | `std_srvs/SetBool`, latched topic, custom service | `std_srvs/srv/SetBool` service `/vehicle_cmd_gate/armed` unless authority selects custom stateful API | Main ChatGPT | P4-C |
| Gate state topic | Gate state observable | None | Message type/name | DiagnosticStatus, custom message, String enum | DiagnosticStatus for P4-C; custom message only if richer API approved | Main ChatGPT | P4-C |
| Fault diagnostics | Stable fault reason | Partial diagnostic convention | Topic and fields | DiagnosticStatus, mission state detail, custom msg | `/system/vehicle_cmd_gate_state` DiagnosticStatus plus stable reason codes | Main ChatGPT | P4-C |
| Localization-valid input | Invalid localization forces zero | Conceptual `/system/localization_valid` | Type/name and freshness timeout | BoolStamped equivalent, DiagnosticStatus, custom msg | P4 fixture input as `std_msgs/Bool` plus steady receipt timeout, then replace/augment in Phase 5 | Main ChatGPT | P4-C |
| Controller-valid input | Nav2/controller validity required | None | Source and type | Lifecycle state, action status, Bool input | Start with explicit fixture/controller-valid Bool plus runtime authority monitor | Main ChatGPT | P4-C |
| Authority-conflict detection | Exactly one publisher owns topics | Phase monitors reusable | In-process or external monitor contract | Gate self-check, external monitor, both | External continuous GID/process monitor first; gate subscribes to conflict state only after API approved | Main ChatGPT | P4-C/P4-E |
| Heartbeat rate | Repeated zero output required | None | Default rate | 10 Hz, 20 Hz, configurable | 20 Hz to match controller cadence unless resource test proves otherwise | Main ChatGPT | P4-C |
| Input timeout | Stale command forces zero | None | Default timeout | 0.1 s, 0.25 s, 0.5 s | 0.25 s initial fake-mode default | Main ChatGPT | P4-C |
| Speed limits | Generic limits required | Candidate C limits exist but gate limits not defined | Values | Candidate C-derived, lower conservative, configurable | Configurable defaults no greater than Candidate C | Main ChatGPT | P4-C |
| Acceleration/slew limits | Generic slew limits required | None | Values and state reset policy | Per-axis delta/sec | Configurable conservative fake-mode defaults, reset on disarm/fault | Main ChatGPT | P4-C |
| Command frame | TwistStamped frame required | None | Default frame | `base_footprint`, configurable | `base_footprint` configurable | Main ChatGPT | P4-C |
| Startup mode | Real output impossible by default | None | Initial state | MOCK, DISARMED, FAULT | MOCK plus DISARMED until armed fixture succeeds | Main ChatGPT | P4-C |
| Mock vs future real mode | Phase 4 must be mock-only | None | Parameter and enforcement | Enum parameter, launch-only selection | `output_mode:=mock` hard default; real mode absent or fails closed in Phase 4 | Main ChatGPT | P4-D |

## Wheelchair Command Adapter Contract

Current real controller input:

- Topic: `/wheelchair_control_command`
- Type: `std_msgs/msg/Float32MultiArray`
- Required array length: at least 3.
- `data[0]`: radius, interpreted in millimetres by current controller paths.
- `data[1]`: velocity, interpreted in millimetres/second by current controller paths.
- `data[2]`: distance, interpreted in millimetres by legacy Pure Pursuit output; current wheelchair controller stores it but CAN conversion mainly uses radius and velocity.
- Stop vector: `[0.0, 0.0, 0.0]`.
- Current CAN transport default can open `can0`; current UDP mode can send to `10.42.0.1:9999`; both are forbidden in default Phase 4.
- Current controller treats radius `0` with nonzero velocity as in-place differential rotation; Phase 4 must reject in-place rotation until chassis mapping is verified.
- Current controller clamps velocity and normalizes radius, including optional radius sign inversion. The Phase 4 generic gate must not rely on these lower-level clamps for safety.

Separation:

| Layer | Contract | Phase 4 status |
| --- | --- | --- |
| A. `/vehicle_cmd_safe` | Generic SI `TwistStamped`, base frame, finite `linear.x` and `angular.z`, no chassis semantics | P4-C target |
| B. Mock adapter output | `/wheelchair_control_command_mock` Float32MultiArray, no CAN/UDP subscriber, repeated mock stop heartbeat | P4-D target |
| C. Existing wheelchair controller input | `/wheelchair_control_command` Float32MultiArray length 3, real controller semantics | Read-only legacy evidence |
| D. CAN encoder behavior | Wheel speed conversion and can0 output | Forbidden in Phase 4 default mode |

Required adapter conversion tests:

- Stop.
- Straight forward.
- Gentle left.
- Gentle right.
- Maximum permitted command.
- Reverse blocked.
- Stale input.
- NaN/Inf.
- Unsupported lateral/vertical/roll/pitch.
- In-place rotation rejected.
- Near-zero angular velocity.
- Near-zero linear velocity with nonzero angular velocity.
- Command timeout.
- Shutdown.
- Repeated mock stop heartbeat.

Recovered converter reuse classification:

`REUSE_TEST_VECTORS_ONLY`

Reason: V3.1 permits recovered conversion logic as evidence, but current-source authority still requires a new Phase 4 adapter that cannot inherit unchecked real-output paths or wheelchair-controller transport side effects.

## Independent Adapter Deadman

The Generic Gate deadman protects `/vehicle_cmd_safe`.

The Wheelchair Command Adapter deadman protects chassis-formatted output if `/vehicle_cmd_safe` stops after a previously accepted nonzero command.

Required adapter behavior:

- Use steady/monotonic receipt time.
- Publish startup mock stop before any accepted command.
- Publish repeated mock stop at an approved heartbeat while stale, invalid, disarmed, or shutting down.
- Check both message header stamp validity policy and receipt freshness; receipt time is authoritative for deadman timing.
- Reject stale `TwistStamped` input even if the last data values are finite.
- Keep future real mode disabled or absent in Phase 4.
- On shutdown, emit bounded repeated mock stop and close without opening CAN or application UDP.

Missing decisions:

- Adapter heartbeat default.
- `/vehicle_cmd_safe` input timeout.
- Straight-radius sentinel value.
- Maximum allowed mock velocity/radius.
- Whether near-zero angular velocity maps to straight or is rejected.
- Whether reverse remains fully blocked through all Phase 4.

## Mission Manager Blocked And Progress Reconciliation

Phase 3 already implemented the generic Mission Manager and typed mission state machine. Do not create a second mission state machine in Phase 4.

| Item | Classification | Remaining Phase 4 work |
| --- | --- | --- |
| Typed mission reception/start/cancel/pause/resume | `COMPLETE_IN_PHASE3` | Reuse |
| Sequential Nav2 waypoint dispatch | `COMPLETE_IN_PHASE3` | Reuse |
| Failed waypoint not silently skipped | `COMPLETE_IN_PHASE3` | Reuse |
| `TEMPORARILY_BLOCKED`, `BLOCKED`, `HELP_REQUIRED` public states | `PARTIAL` | States exist; classification policy incomplete |
| Bounded no-progress detection | `DEFER_TO_LATER_PHASE4_SUBTASK` | P4-E should map sustained gate/Collision Monitor stop or Nav2 failure to bounded mission state |
| Planner failure mapping | `PARTIAL` | Phase 2/P3 covered failure evidence; mission blocked mapping not final |
| Controller failure mapping | `PARTIAL` | No-skip behavior exists; blocked/help mapping not final |
| Collision Monitor sustained-stop mapping | `NOT_IMPLEMENTED` | P4-E synthetic obstacle tests |
| Semantic obstacle classification | `DEFER_TO_SEMANTIC_OBSTACLE_PHASE` | Later live/semantic phase |

Synthetic Phase 4 obstacles can test sustained stop, slowdown, cancellation, progress timeout and generic blocked state wiring. Semantic obstacle class and human-help policy require later phases.

## VehicleState Classification

`parking_robot_interfaces/msg/VehicleState` does not exist in the current repository.

V3.1 conceptual requirements:

- Tri-state status values.
- Validity mask.
- No fabricated lower-controller acknowledgement.
- Unsupported fields remain UNKNOWN/invalid.

Classification:

`P4_LATER_SUBTASK`

Reason: VehicleState is not required to prove P4-B Collision Monitor isolation or P4-C generic Twist safety in mock mode, but it becomes important before future real adapter arming or any claim about lower-controller acknowledgement.

## No-Bypass Threat Model

| Failure | Effect | Preventive design | Runtime detection | Test | Required evidence | Phase owner |
| --- | --- | --- | --- | --- | --- | --- |
| MPPI raw bypasses Collision Monitor | Obstacle stop/slow ineffective | Adapter and gate subscribe only downstream | Topic/process authority monitor | Attempt unexpected raw subscriber/publisher fixture | GID/process table | P4-B/P4-E |
| Adapter subscribes raw or safe Twist directly | Gate bypass | Adapter input fixed to `/vehicle_cmd_safe` | Source scan and ROS graph | Static and runtime isolation | Subscriber inventory | P4-D |
| Gate bypass | Unsafe command reaches adapter | Mock adapter accepts only `/vehicle_cmd_safe` | Graph monitor | Start with wrong remap | No output on mock | P4-D/P4-E |
| Duplicate command publishers | Conflicting authority | One allowed publisher per topic | Continuous GID monitor | Inject duplicate publisher | Fault/zero evidence | P4-C/P4-E |
| Legacy Pure Pursuit active | Legacy physical path | Mission mode launch excludes it | Process monitor | Launch inspection | No process/topic | P4-E |
| `laser_command_safety_filter` active | Legacy real command path | Mission mode excludes it | Process monitor | Launch inspection | No node/topic | P4-E |
| `wheelchair_controller_node` active | CAN/UDP risk | Phase 4 fake launch forbids it | Process monitor | Launch inspection | No process/can0/UDP | P4-D/P4-E |
| Mock topic remapped to real | Real actuation risk | Hard-coded mock output and real topic absence check | Topic monitor | Remap-failure test | Real topic absent | P4-D |
| Application UDP active | External actuation risk | Mission mode disables UdpSender | Socket/process audit | Legacy-mode mock vs mission-mode test | No destination attempts | P4-D/P4-E |
| can0 access | CAN actuation risk | Do not launch controller; no SocketCAN open | Process/file/socket monitor | Runtime authority audit | No CAN access | P4-D/P4-E |
| Stale Collision Monitor source | Obstacle data invalid | Collision Monitor timeout plus gate freshness | Source heartbeat monitor | Stop synthetic scan | Repeated zero | P4-B/P4-C |
| Stale safe Twist | Old command repeats | Gate input watchdog | Steady-time timeout | Stop `/cmd_vel_nav_safe` | Repeated zero | P4-C |
| Stale localization-valid input | Movement with stale localization | Gate permission watchdog | Permission monitor | Drop fixture signal | Repeated zero | P4-C/P5 |
| ROS time pause/jump | Watchdogs fail | Use steady monotonic time | Clock audit | Pause sim time fixture | Timeout still fires | P4-C/P4-D |
| NaN/Inf | Invalid command propagation | Numeric validation | Command validator | Publish NaN/Inf | Rejection/zero | P4-C/P4-D |
| Unsupported lateral command | Chassis misuse | Reject nonzero unsupported fields | Command validator | Publish y/z/roll/pitch | Rejection/zero | P4-C |
| Reverse | Unverified chassis behavior | Explicit block policy | Command validator | Publish negative x | Rejection/zero | P4-C/P4-D |
| In-place rotation | Unverified radius mapping | Reject until verified | Command validator | Publish x=0,z!=0 | Rejection/zero | P4-D |
| Executor/node crash | Prior command may persist | Independent deadmen and lifecycle stop | Heartbeat/process monitor | Kill process fixture | Downstream repeated stop or fail evidence | P4-C/P4-D |
| Shutdown without repeated stop | Last nonzero persists | Bounded shutdown stop sequence | Command timeline | Shutdown after nonzero | Stop samples | P4-C/P4-D |
| Output silence after nonzero | Chassis keeps prior command | Repeated zero heartbeat | Command timeline | Stop upstream after nonzero | Repeated zeros | P4-C/P4-D |
| Authority conflict after arming | Bypass after initial validation | Continuous monitoring while armed | GID monitor | Inject late publisher | Fault/zero | P4-C/P4-E |

## Proposed Phase 4 Decomposition

| Subtask | Scope | Dependencies | Likely changed files | Forbidden paths | Runtime tests | Pass criteria | Stop conditions | Rollback point |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P4-B | Isolated MPPI raw to Collision Monitor safe Twist with synthetic obstacle source | P4-A decisions for CM config naming | Phase-4-only launch/config/tests | Phase 2 config, real sensors, CAN, UDP | Synthetic LaserScan stop/slow/stale source, publisher ownership | `/cmd_vel_nav_raw` and `/cmd_vel_nav_safe` separated, CM behavior proven | Schema conflict, real topic, source ambiguity | Revert P4-B launch/config only |
| P4-C | Generic Command Safety Gate core | Open gate APIs approved | New `vehicle_cmd_safety` package/tests | Adapter real output, Phase 2 tuning | Arming, disarm, stale, NaN, limits, slew, repeated zero, authority conflict | `/vehicle_cmd_safe` TwistStamped safe under all gates | Interface conflict, unrepeated stop, duplicate publisher | Revert gate package |
| P4-D | Mock Wheelchair Command Adapter | P4-C `/vehicle_cmd_safe` contract | New adapter package/config/tests | Real `/wheelchair_control_command`, CAN, application UDP | Conversion vectors, stale input, in-place/reverse rejection, mock repeated stop | Only `/wheelchair_control_command_mock` exists and is safe | Real topic appears, CAN/UDP access, bad conversion | Revert adapter package |
| P4-E | Full fake closed-loop integration | P4-B/C/D pass | Phase-4-only launch/orchestration/tests | Physical launch, live sensors | Obstacle stop/slow, mission cancel, blocked/progress mapping, no-bypass proof | Mission chain safe through fake base and mock adapter | Unknown publisher, post-stop motion, stale localization not suppressed | Revert integration launch/tests |
| P4-F | Phase 4 regression and closure | P4-E pass | Docs/evidence only | New implementation | Full regression campaign | Phase 4 closed for fake/synthetic safety chain | Evidence gap or regression | Documentation-only rollback |

Gazebo starts only after the fake closed-loop Phase 4 chain passes. Live sensors and MID-360 remain Phase 5 or later.

## Implementation Readiness

Classification:

`P4_READY_AFTER_EXPLICIT_INTERFACE_DECISIONS`

Required decisions in priority order:

1. Gate arm/disarm API name and type.
2. Gate state/fault diagnostic topic names and message types.
3. Localization-valid fixture input type/name and timeout for P4, with Phase 5 replacement path.
4. Controller-valid signal source/type.
5. Heartbeat, input timeout, speed limit, acceleration/slew defaults.
6. Authority-conflict monitor interface into the gate.
7. Adapter straight-radius sentinel and near-zero angular policy.
8. Reverse and in-place rotation policy duration for Phase 4.
9. Mock adapter heartbeat and stale stamped-command timeout.
10. VehicleState timing: P4 later subtask versus Phase 5.

## Limitations

- No Phase 4 source, launch, config, interface or test implementation was performed in P4-A.
- Nav2, Collision Monitor, Mission Manager, plan_nav and any command-producing ROS node were not launched.
- The installed Collision Monitor `.cpp` implementation source is not present in the Debian install; P4-B must validate runtime behavior locally.
- P4-A does not close safety behavior. It only defines the authority chain, reusable evidence, open decisions and recommended implementation sequence.
- The Phase 2 cached-transform freshness limitation remains mandatory Phase 4/5 work and is not waived.
