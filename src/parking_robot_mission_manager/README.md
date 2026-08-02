# parking_robot_mission_manager

Phase 3 Mission Manager core aligned with the V3.1 architecture authority.

## ROS API

- Subscribes: `/mission/route`
- Publishes: `/mission/state`
- Diagnostics: `/mission/status`, `/mission/block_reason`
- Services:
  - `/mission/start` using `std_srvs/srv/Trigger`
  - `/mission/cancel` using `std_srvs/srv/Trigger`
  - `/mission/pause` using `std_srvs/srv/SetBool`
- Action client: standard `nav2_msgs/action/NavigateToPose`

`/mission/pause` SetBool semantics:

- `true`: pause;
- `false`: resume.

The Mission Manager never publishes Twist or any velocity-equivalent command.

## Route Contract

The manager accepts only typed `parking_robot_interfaces/RouteMission` input.
Required fields are:

```text
std_msgs/Header header
string mission_id
string route_id
string topology_version
string[] node_ids
string[] edge_ids
int8[] edge_directions
geometry_msgs/PoseStamped[] poses
```

The configured `expected_topology_version` must match the mission
`topology_version`. Array lengths must be consistent, directions must be `-1`,
`0`, or `1`, all IDs must be nonempty, all poses must resolve to `map`, and
quaternions must be finite, nonzero, and normalized.

Waypoint spacing defaults:

```text
goal_xy_tolerance_m = 0.25
waypoint_separation_margin_m = 0.05
min_waypoint_separation_m = 0.55
```

The default follows:

```text
min_waypoint_separation_m =
    2 * goal_xy_tolerance_m + waypoint_separation_margin_m
```

The value is configurable. Close consecutive poses are rejected with
`CONSECUTIVE_WAYPOINTS_TOO_CLOSE`; they are not silently removed because that
would also require changing node and edge identity arrays. P3-B must export
sparse topological waypoints rather than dense path samples.

Plain `nav_msgs/Path` geometry is not authoritative in P3-A. Any later
compatibility adapter must not invent route identity, edge identity, direction
or topology version.

## Mission State Policy

Receiving a mission stores it and publishes `RECEIVED`; it does not dispatch
Nav2. `/mission/start` validates the stored mission and then sends sequential
NavigateToPose goals:

```text
IDLE -> RECEIVED -> VALIDATING -> PLANNING -> NAVIGATING -> SUCCEEDED
```

Validation failure becomes `FAILED` with a stable reason code. Goal rejection or
ABORTED without later blocked classification also becomes `FAILED`. A failed
waypoint is never skipped.

`CANCELLED`, `SUCCEEDED`, `BLOCKED`, and `FAILED` remain observable terminal
states until a new mission or explicit reset transitions through `IDLE`.

## Pause And Cancel

The ROS node is a transport adapter around the ROS-independent
`MissionStateMachine`; node-visible mission state, legal transitions, progress,
active goal UUIDs, terminal reasons, and dispatch eligibility all come from that
single core. The node may retain ROS action goal handles and Futures, but it
does not maintain a second mission-state model.

Mission cancellation sends at most one active-goal cancel request. Service
success for `/mission/cancel` means the request was accepted for asynchronous
processing; it does not mean the Nav2 goal is already canceled. `CANCELLED` is
published only after both the cancel response is accepted and a terminal
`CANCELED` action result arrives.

Cancellation deadlines use steady monotonic time, not ROS simulation time:

```text
cancel_response_timeout_sec = 2.0
cancel_result_timeout_sec = 5.0
```

Reason codes distinguish immediate rejection from true timeout:

- `CANCEL_ACK_REJECTED`
- `CANCEL_ACK_TIMEOUT`
- `CANCEL_RESULT_TIMEOUT`

Pause from `NAVIGATING` sends at most one active-goal cancel request, preserves
the current waypoint index, and publishes `PAUSED` only after both cancellation
acknowledgement and a terminal `CANCELED` result. `/mission/pause` service
success means the pause or resume request was accepted for processing; the
authoritative completion appears on `/mission/state`.

Pause-specific failure reason codes are:

- `PAUSE_CANCEL_ACK_REJECTED`
- `PAUSE_CANCEL_ACK_TIMEOUT`
- `PAUSE_CANCEL_RESULT_TIMEOUT`

Resume transitions through `PLANNING` and re-sends the same current waypoint.
Late action results after cancellation or pause timeout are retained as
diagnostics and do not change `FAILED` into `SUCCEEDED`, `CANCELLED`, or
`PAUSED`.

`TEMPORARILY_BLOCKED`, `BLOCKED`, and `HELP_REQUIRED` are reserved mission
outcomes whose complete classification policy is added in later Phase 3/4
integration. P3-A.1 does not classify every Nav2 abort as blockage.

## P3-B Entry Note

A P3-B exporter must prove that the selected `plan_nav` route can provide
explicit topology version, node IDs, edge IDs, and edge directions without
deriving identity from geometry. If those identifiers are absent, P3-B requires
topology identity augmentation before publishing typed `RouteMission`.

## Scope

P3-A.2 does not modify `plan_nav`, Dijkstra, Phase 2 Nav2 parameters, fake-base
behavior, maps, scenarios, or Phase 2 runners. Dense `/plan_nav` remains
display-only and non-authoritative in the new mode.
