# parking_robot_interfaces

Typed Phase 3 mission interfaces aligned with the V3.1 architecture authority.

## RouteMission

Authoritative mission input is `parking_robot_interfaces/msg/RouteMission` on
`/mission/route`:

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

Required invariants:

- `header.frame_id` is `map`;
- `mission_id`, `route_id`, and `topology_version` are nonempty;
- `topology_version` matches the Mission Manager configured topology version;
- `poses.size == node_ids.size`;
- `edge_ids.size == max(poses.size - 1, 0)`;
- `edge_directions.size == edge_ids.size`;
- `edge_directions` values are exactly `-1`, `0`, or `1`;
- every node ID and edge ID is nonempty;
- every pose frame is `map` or inherits the mission frame;
- pose coordinates and quaternions are finite;
- quaternions are nonzero and normalized;
- consecutive poses must satisfy the configured sparse waypoint separation.

Route identity, edge identity, and direction are never inferred from geometry.
Dense `nav_msgs/Path` geometry is not authoritative.

## MissionState

`MissionState` is published on `/mission/state` and uses typed constants:

```text
IDLE
RECEIVED
VALIDATING
PLANNING
NAVIGATING
PAUSED
CANCELLING
CANCELLED
SUCCEEDED
TEMPORARILY_BLOCKED
BLOCKED
FAILED
HELP_REQUIRED
```

`current_waypoint_index` is zero-based, progress is monotonic, and completed
count must never exceed total count.

`TEMPORARILY_BLOCKED`, `BLOCKED`, and `HELP_REQUIRED` are public mission states,
but their complete obstacle/fault classification policy is reserved for later
Phase 3/4 integration.

This package defines no velocity, hardware, charging, company-specific task, CAN
or UDP protocol.
