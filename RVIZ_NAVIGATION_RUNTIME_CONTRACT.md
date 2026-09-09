# MK2G1A RViz Navigation Runtime Contract

## Scope

This package is observation-only. It renders existing navigation authority in
RViz and publishes two derived markers. It does not implement the Operator
GUI, modify PlanNav, publish any command, create any action/service client, or
access CAN.

## Package and launch

Package: `robot_visualization`

Launch: `robot_visualization/navigation_visualization.launch.py`

The launch starts the helper and optionally RViz. `use_fixture:=true` starts a
synthetic offline-only map/path/mission/TF fixture. It does not start sensors,
Nav2, PlanNav, Mission Manager, or any lower command node.

RViz configuration:

`robot_visualization/config/canonical_navigation.rviz`

Fixed frame: `map`.

## Existing displays

| Display | Source | Default |
|---|---|---|
| Global Map | `/map`, `nav_msgs/msg/OccupancyGrid` | ON |
| Robot | `robot_description` plus TF | ON |
| Robot Footprint | `/local_costmap/published_footprint` | ON |
| Global Plan | `/plan`, `nav_msgs/msg/Path` | ON |
| Selected Local Trajectory | `/trajectories`, `MarkerArray`, namespace `Optimal Trajectory` | ON |
| Local Costmap / Detected Obstacles | `/local_costmap/costmap` | ON |
| Controller Reference | `/transformed_global_plan` | OFF |
| Global Costmap | `/global_costmap/costmap` | OFF |
| CM Stop/Slow polygons | `/collision_monitor/phase5_stop`, `/collision_monitor/phase5_slow` | OFF |
| Raw MID-360 | `/cloud_registered_body` | OFF |
| Raw T-mini | `/scan` | OFF |
| MPPI candidates | `/trajectories`, namespace `Candidate Trajectories` | OFF |
| TF debug | TF | OFF |

MPPI visualization is enabled in the accepted `FollowPath` configuration by
setting only `visualize: true`. No controller limit, critic, model,
frequency, or optimization parameter was changed.

## Observation helper outputs

`navigation_visualization_helper` subscribes to:

* `/plan`, `nav_msgs/msg/Path`, expected frame `map`;
* TF `map -> base_footprint`;
* `/mission/route`, `RouteMission`;
* `/mission/state`, `MissionState`.

It publishes:

* `/visualization/nearest_path_point`, `visualization_msgs/msg/Marker`;
* `/visualization/current_navigation_goal`, `visualization_msgs/msg/Marker`.

Nearest Point is the nearest sampled planner-path pose by Euclidean distance
to the current `base_footprint` translation in `map`. No interpolation is
used; the sampled-pose definition is deterministic and tested.

Current Navigation Goal is the RouteMission pose indexed by
`MissionState.current_waypoint_index`. Empty, invalid-frame, or out-of-range
inputs publish a DELETE marker and cannot create a stale visual authority.

Aim Point is deliberately not implemented or displayed.

## Safety boundary

The package contains no publisher, client, or transport for:

`/cmd_vel`, `/cmd_vel_nav`, `/vehicle_cmd_safe`,
`/wheelchair_control_command`, NavigateToPose, Mission Manager services, or
CAN. Static tests enforce this boundary. The fixture publishes only
`/map`, `/plan`, `/mission/route`, `/mission/state`, and TF.
