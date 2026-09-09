# MK2G0A GUI/RViz Source Authority Contract

Status: source-authority freeze for `P5A-MK2G0A`.

## Scope

This document freezes the information and supervisory-control boundary for a
future Operator GUI and RViz configuration. It does not implement either UI
and it does not modify PlanNav.

PlanNav remains the existing topology/route-authoring interface, unchanged.
The Operator GUI observes mission/safety state and calls existing supervisory
services only. It never publishes velocity, owns `NavigateToPose`, or computes
an independent safety validity result. RViz displays spatial state and debug
observations only.

## Authority rules

* `parking_robot_interfaces/msg/MissionState` on `/mission/state` is the
  primary mission status authority.
* `RouteMission` on `/mission/route` is the route identity and topology
  metadata authority. Its geometry is map-frame mission waypoint geometry;
  route identity is not inferred from geometry.
* `/vehicle_cmd_safe` is the safety-approved commanded Twist, not measured
  chassis speed.
* `/system/collision_monitor_valid` is the required-perception/CM health
  Boolean in the current Phase-5 chain. It is not a CLEAR/SLOW/STOP action
  state.
* Generic Gate `DiagnosticStatus` on its configured state topic is the state
  and reason authority.
* Collision Monitor has no current authoritative CLEAR/SLOW/STOP state topic.
  Its configured polygon geometry and Twist output are observations, not a
  substitute for a state authority. A future read-only adapter may expose
  that distinction, but no GUI-side inference is authorized in MK2G0A.
* The Planner Server publishes `/plan` (`nav_msgs/msg/Path`) for valid plans.
* MPPI's built-in visualization publishes `/trajectories`
  (`visualization_msgs/msg/MarkerArray`) and `transformed_global_plan`
  (`nav_msgs/msg/Path`) only when `FollowPath.visualize` is enabled. The
  `Optimal Trajectory` marker namespace is the selected trajectory; the
  `Candidate Trajectories` namespace is debug-only.

## Safety reset disposition

`RESET_SAFETY_GUI_NOT_CURRENTLY_AUTHORIZED`.

The Gate has an existing `std_srvs/srv/SetBool` arm service. `false` performs
the existing disarm/fault-clear operation and `true` requests arm only after
all current validity, publisher-authority, freshness, and stability
prerequisites pass. In the accepted stationary mock launch this is
`/phase5/gate_test/arm`; the node default is `/vehicle_cmd_safety/arm`.
These are qualification/configuration interfaces, not yet a frozen final
operator reset authority. Gate reset does not restart or resume a terminal
Mission Manager state. The future GUI must not add a reset button until one
production service name and its operator authorization are separately frozen.

## Frames and rates

* Map and global plan: `map`.
* Global costmap: `map`; local costmap and MPPI transformed trajectory:
  `odom_chassis` under the accepted stationary/navigation configuration.
* Robot navigation reference: `base_footprint`.
* Mission progress pose observation: accepted TF chain, with
  `odom_chassis -> base_footprint` for the current Phase-5 navigation mode.
* Rates are configuration/implementation rates, not GUI polling deadlines:
  MPPI 20 Hz, local costmap update 10 Hz/publish 5 Hz, global costmap
  update 2 Hz/publish 1 Hz, Gate heartbeat 20 Hz, Collision Monitor command
  stream approximately 10 Hz in the accepted Humble configuration.

## Derived observations allowed

* Current metric goal is the retained `RouteMission.poses[index]` joined with
  `MissionState.current_waypoint_index`; this is an observational join.
* Start and goal node are `node_ids[0]` and `node_ids[-1]` from the same
  retained RouteMission.
* Current edge is `edge_ids[index]` only while `index < len(edge_ids)`;
  final-pose state is `ARRIVAL/NONE`. It must not be inferred from robot
  proximity.
* Nearest point may be computed by an observation-only helper from `/plan`
  and TF. It must not publish a controller input.
* Aim point is not currently authoritative. The existing `/lookahead_point`
  helper is a legacy observation candidate based on `/plan`, not an MPPI
target authority.
