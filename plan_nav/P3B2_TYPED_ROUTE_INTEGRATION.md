# P3-B.2 Typed Route Integration

P3-B.2 connects the P3-B.1 sparse `RouteSpec` contract to the hardened
Mission Manager API without changing topology assets or publishing velocity.

## Authority Modes

`plan_nav` now has two explicit authority modes:

- `legacy`: existing dense Dijkstra path behavior remains authoritative for
  the legacy UDP integration point.
- `mission_nav2`: sparse typed missions are authoritative. Legacy UDP is
  disabled, no velocity or wheelchair command is published, and dense
  `/plan_nav` may continue only as display/comparison output.

The default mode remains `legacy`. `mission_nav2` must be selected explicitly.

## ROS Ownership

`core.ros_runtime` owns process-wide `rclpy` initialization and shutdown.
`PoseReceiver`, `PlanPublisher`, and the Mission bridge use that shared owner
instead of independently shutting down ROS. Qt widgets are updated only from Qt
slots; ROS callbacks emit Qt signals through `MissionBridgeThread`.

## RouteMission Publication

`RouteMission` is constructed only from the P3-B.1 `RouteSpec` returned by
`build_sparse_route_spec()`.

For each explicit publish request:

- a new unique `mission_id` is used;
- deterministic `route_id` is preserved;
- current manifest `topology_version` must still match the planned route;
- `node_ids`, `edge_ids`, `edge_directions`, and ordered poses are copied
  exactly from `RouteSpec`;
- one `/mission/route` message is published;
- `/mission/start` is not called automatically.

Stable rejection reasons include `NO_TOPOLOGICAL_ROUTE`,
`ROUTE_EDGE_NOT_FOUND`, `AMBIGUOUS_ROUTE_EDGE`,
`CONSECUTIVE_WAYPOINTS_TOO_CLOSE`, `TOPOLOGY_CHANGED_AFTER_PLANNING`, and
`MISSION_MODE_DISABLED`.

## Mission Controls

The required sequence is:

1. select a route;
2. publish/prepare the mission;
3. observe matching `MissionState.RECEIVED`;
4. explicitly call Start.

Start remains disabled until the displayed `mission_id` and `route_id` match a
received `/mission/state` message with state `RECEIVED`. Service responses mean
request acceptance only; `/mission/state` remains authoritative for completion.

Pause, resume, and cancel call the frozen Mission Manager services:

- `/mission/pause` with `true` pauses;
- `/mission/pause` with `false` resumes;
- `/mission/cancel` cancels.

## Route Identity Behavior

Disconnected selections produce no `RouteSpec`, no `RouteMission`, no start
request, and no NavigateToPose goal. Ambiguous topology fixtures are rejected
with `AMBIGUOUS_ROUTE_EDGE`; no collaborator-owned ambiguity UI is implemented
in P3-B.2.

Reverse traversal of a persisted `bi` edge uses the same `edge_id` with
`edge_direction = -1`. Forward traversal uses `edge_direction = +1`.

## Scope Limits

P3-B.2 does not launch real Nav2, does not close Phase 3, and does not
implement P3-C. It does not add a geometry-only Path adapter, Collision
Monitor, Generic Command Safety Gate, wheelchair adapter, sensors, CAN, UDP
transmission, or physical command path.
