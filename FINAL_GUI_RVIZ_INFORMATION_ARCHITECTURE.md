# Final GUI/RViz Information Architecture — MK2G0A

## Frozen boundary

PlanNav is preserved exactly as-is and remains the topology/route-authoring
GUI. No PlanNav source, launch, controls, visuals, or route-authoring behavior
is changed by MK2G0A.

The Operator GUI answers **what the system is doing** and invokes only the
existing Mission Manager supervisory services. RViz answers **where the robot
is and how navigation is behaving spatially**. RTAB-Map/PCL displays remain
engineering/debug tools, not new operator authority.

## Operator GUI information

Primary mission source: `/mission/state`,
`parking_robot_interfaces/msg/MissionState`, transient-local reliable. It
contains mission ID, route ID, state, current waypoint index, completed/total
counts, progress, active NavigateToPose UUID, reason code, and detail.

Route identity and node/edge metadata come from the retained
`/mission/route`, `parking_robot_interfaces/msg/RouteMission`. Current Goal is
an observational join of that message and the state index. Current Edge is an
observational index join and is not directly published by Mission Manager.

Safety information comes from existing sources: `/vehicle_cmd_safe` for
Safe Commanded Speed (`TwistStamped.twist.linear.x`),
`/system/collision_monitor_valid` for required perception/CM health,
`/system/localization_valid` for localization health, and the Gate's
`DiagnosticStatus` state topic for state/reason. The current mock chain's Gate
state is `/phase5/gate_test/state`; the node default is
`/vehicle_cmd_safety/state`.

`/system/controller_valid` is an existing Gate input and GUI display source,
but the accepted stationary mock launch currently obtains it from the
permission fixture. No dedicated production controller-health publisher is
frozen; this is a later authority decision, not a GUI-side calculation.

The recommended high-level Nav2 display is MissionState as the primary
operator state, with NavigateToPose action/lifecycle detail only as a
secondary diagnostic. Do not expose every lifecycle transition as the main
operator status.

## RViz information

Default spatial views are Map (`/map`), robot TF/RobotModel, configured
footprint, global plan (`/plan`), local costmap, and current metric goal. The
selected MPPI trajectory can be displayed from the built-in `Optimal
Trajectory` namespace on `/trajectories` after enabling the existing MPPI
visualization parameter. Candidate trajectories remain debug-only.

Local obstacles are represented by the qualified local costmap and its
published footprint; raw `/cloud_registered_body` and `/scan` are debug
inputs. The configured Collision Monitor zone polygons are optional/debug
geometry only. Current canonical Phase-5 polygon topics are
`/collision_monitor/phase5_stop` and `/collision_monitor/phase5_slow`;
older RViz references to `/collision_monitor/stop_zone` and
`/collision_monitor/slow_zone` are not the current Phase-5 authority.

Nearest Point is an observation-only nearest point on `/plan` relative to the
current TF pose if a dedicated helper is later enabled. Aim Point remains
`AIM_POINT_REQUIRES_VISUALIZATION_DEFINITION`: it must not be presented as an
MPPI target until a definition is selected and implemented observationally.

## Strict exclusions

No GUI component publishes `cmd_vel`, `cmd_vel_nav`, `/vehicle_cmd_safe`,
RouteMission, or NavigateToPose goals. No GUI component owns the safety Gate.
The GUI cannot infer a safe condition from stale data or replace the Gate's
validity logic.

## Implementation package plan

Future packages, following current repository conventions:

* `operator_gui`: ROS interface adapter, read-only state model, Qt widgets,
  existing Mission Manager service clients, and offline contract tests.
* `robot_visualization`: RViz configuration and observation-only marker
  helpers for nearest point/current goal/selected trajectory if needed.

Reuse `parking_robot_interfaces`, `parking_robot_mission_manager`, Nav2
topics/actions, existing TF/URDF, costmap displays, and the built-in MPPI
visualizer. Do not copy PlanNav or create a custom 3-D renderer.

## Unresolved by design

The following are explicitly identified rather than guessed: a clean stock
Collision Monitor CLEAR/SLOW/STOP status topic, a dedicated production
controller-valid publisher, a final operator Gate-reset authorization, and a
canonical MPPI aim-point semantic. These are future authority decisions;
they do not authorize GUI-side substitutes.
