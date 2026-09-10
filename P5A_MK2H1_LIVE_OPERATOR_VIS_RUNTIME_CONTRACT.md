# P5A MK2H1 Live Operator Visualization Runtime Contract

## Disposition

`P5A_MK2H1_BLOCKED_ROUTE_LOCALIZATION_MISMATCH`

The live canonical stack is qualified through the pre-mission readiness gate.
No RouteMission was published and no mission control was invoked because the
accepted PlanNav topology was not spatially consistent with the live localized
robot pose.

## Canonical launch

```bash
ros2 launch robot_bringup mk2h1_live_operator_mock.launch.py
```

The launch includes the existing canonical navigation composition with:

- real MID-360 and T-mini drivers;
- FAST-LIO and RTAB-Map localization using a unique runtime database copy;
- real global and dual-sensor local costmaps;
- real Collision Monitor, planner, MPPI, BT Navigator, and Mission Manager;
- the Generic Safety Gate, bridge, vendored wheelchair controller, and
  hard-locked `MockTransport`;
- the standalone Operator GUI and canonical RViz configuration;
- the qualified real localization-validity monitor;
- the existing controller permission fixture only for
  `/system/controller_valid`.

The launch does not start any integrated demo fixture or physical transport.

## Validity authority

`/system/localization_valid` has one publisher:
`/localization_validity_monitor`. It observes `/Odometry` and the qualified TF
chain. The previous stationary localization permission publisher is disabled
only in this profile.

`/system/controller_valid` has one publisher:
`/stationary_gate_permission_fixture`. In MK2H1 this means mock command
permission. It is **not physical chassis/controller health**.

`/system/collision_monitor_valid` has one publisher:
`/required_perception_validity`, derived from the real dual-sensor freshness
authorities.

## Actuation containment

The backend parameters are fixed to:

- `output_transport: mock`
- `can_interface: CANONICAL_MOCK_FORBIDDEN`

The backend reported `MockTransport enabled: SocketCAN will not be opened`.
No SocketCAN endpoint or physical chassis writer was present. can0 remained at
zero RX and zero TX packets before and after the run.

## Route admission gate

Before `/mission/start`, the current metric route must be meaningful in the
same `map` frame as the localized robot. The live pose was approximately:

- x: -7.260 m
- y: 12.452 m
- yaw: -96.1 degrees

The current PlanNav workspace is `plan_nav/underGround_split1`. Its nearest
node was node 1 at `(0.8461, -2.0429)`, 16.608 m from the robot. No accepted
workspace node represented the current robot region. Therefore:

- no RouteMission was published;
- Operator START was not pressed;
- NavigateToPose, planning, MPPI command production, pause, resume, and cancel
  were not claimed in this run;
- topology was not edited or invented to force a pass.

## Shutdown and integrity

All launched nodes and GUI processes exited. The ROS daemon used for inspection
was stopped. The known Livox driver shutdown-only exit `-11` occurred after
SDK deinitialization; all other listed launch processes exited cleanly.

The canonical database remained:

`73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`

The runtime database was preserved at:

`/home/dog/phase5_runtime/rtabmap/rtabmap_2d_localization_20260910T210127_pid8929.db`
