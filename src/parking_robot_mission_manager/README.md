# parking_robot_mission_manager

Phase 3 Mission Manager core for typed route missions.

## Interfaces

- Subscribes: `/mission_manager/route_mission`
- Publishes: `/mission_manager/state`
- Services: `/mission_manager/pause`, `/mission_manager/resume`,
  `/mission_manager/cancel`
- Action client: standard `nav2_msgs/action/NavigateToPose`

The Mission Manager never publishes Twist or any velocity-equivalent command.

## Route Contract

The manager accepts only typed `parking_robot_interfaces/RouteMission` input.
`mission_id`, `route_id`, `route_version`, and `direction_id` must be nonempty.
The mission frame must be `map`, waypoint IDs must be nonempty and unique, pose
values must be finite, quaternions must be valid and normalized, and consecutive
waypoints must not be effectively identical.

Plain `nav_msgs/Path` geometry is not authoritative in P3-A. Any compatibility
adapter belongs to later Phase 3 work and must still resolve route identity and
direction without ambiguity.

## State Policy

Legal nominal flow is:

`IDLE -> VALIDATING -> READY -> RUNNING -> SUCCEEDED -> IDLE`

Invalid routes become `REJECTED`. Goal rejection or a failed waypoint becomes
`FAILED`; later waypoints are not sent. Cancel from `READY`, `RUNNING`, or
`PAUSED` publishes `CANCELING` and returns to `IDLE`.

Pause policy for P3-A:

- cancel the active NavigateToPose goal;
- retain the current waypoint index;
- enter `PAUSED` after cancellation acknowledgement or bounded timeout;
- resume re-sends the current waypoint.

## Scope

P3-A does not modify `plan_nav`, Dijkstra, Phase 2 Nav2 parameters, fake-base
behavior, maps, scenarios, or Phase 2 runners. Dense `/plan_nav` remains outside
this package and is not accepted as authoritative mission input.
