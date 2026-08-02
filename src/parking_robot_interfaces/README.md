# parking_robot_interfaces

Typed Phase 3 mission interfaces for route execution.

## Route Identity

`RouteMission` is authoritative only when it carries nonempty `mission_id`,
`route_id`, `route_version`, and `direction_id` values. Geometry alone is not a
mission because it cannot distinguish alternate route identities or directions.

`RouteMission.header.frame_id` must be `map`. Waypoint poses must either use
`map` or inherit the mission frame with an empty waypoint pose frame.

## Messages

- `RouteWaypoint`: `waypoint_id` plus a `geometry_msgs/PoseStamped`.
- `RouteMission`: map-frame route identity and ordered sparse waypoints.
- `MissionState`: typed state constants plus progress, active goal UUID, stable
  reason code, and detail text. `current_waypoint_index` is zero-based.

This package defines no velocity, hardware, charging, company-specific task, CAN
or UDP protocol.
