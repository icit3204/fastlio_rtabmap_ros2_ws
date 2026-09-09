# Operator GUI Runtime Contract — MK2G1B

## Role

`operator_gui` is a standalone Qt supervisory display. It observes the frozen
Mission, navigation, and safety authorities and calls only the existing
Mission Manager services. It does not implement navigation, RViz, PlanNav,
velocity control, Gate reset, CAN, or chassis access.

## Executables and launch

* `operator_gui`: production standalone GUI.
* `operator_gui_mock_fixture`: offline/demo status fixture.
* `operator_gui.launch.py`: GUI only; does not start PlanNav or navigation.
* `operator_gui_demo.launch.py`: fixture plus GUI only.

PyQt5 is used with an rclpy worker object in a Qt worker thread. ROS callbacks
update the pure `GuiStateModel`; Qt widgets are updated only through signals.
Service calls use `call_async` and bounded readiness checks, so the GUI event
loop is not blocked.

## Display authorities

Mission fields come from `/mission/state` and `/mission/route`:

* Mission ID: MissionState mission_id.
* Start/Goal Node: RouteMission node_ids first/last.
* Current Edge: edge_ids[current_waypoint_index] only when valid.
* Current Goal: RouteMission poses[current_waypoint_index].
* Progress/state/reason: MissionState.

Navigation fields:

* Nav2 State: MissionState primary, passive NavigateToPose GoalStatus detail.
* Controller Valid: `/system/controller_valid` Bool.
* Safe Cmd Speed: `/vehicle_cmd_safe.twist.linear.x`, explicitly commanded
  speed rather than measured wheel speed.

Safety fields:

* Perception Valid and Collision Monitor Validity:
  `/system/collision_monitor_valid`, displayed as YES/NO/STALE/UNKNOWN.
* Localization Valid: `/system/localization_valid`.
* Gate State/Reason: configured `diagnostic_msgs/msg/DiagnosticStatus`,
  default `/vehicle_cmd_safety/state`.
* Fault Reason: Gate reason first, then MissionState reason code.

The GUI never labels the Collision Monitor Boolean CLEAR, SLOW, or STOP.

## Controls

* START -> `/mission/start`, `std_srvs/srv/Trigger`.
* PAUSE -> `/mission/pause`, `std_srvs/srv/SetBool(data=true)`.
* RESUME -> `/mission/pause`, `std_srvs/srv/SetBool(data=false)`.
* CANCEL -> `/mission/cancel`, `std_srvs/srv/Trigger`.

There is no RESET SAFETY control. Unknown state or route/state mismatch
disables all mission-changing buttons. Authoritative MissionState, rather than
service acceptance alone, confirms transitions.

## Presentation freshness

Mission state older than 2 seconds is shown as `STALE`; Boolean/diagnostic
sources older than 1 second are `STALE`; safe command speed older than 0.5
seconds is `STALE`. These are display-only labels and never feed the robot.
Transient-local mission and route values are retained but mismatched mission or
route identities produce `UNKNOWN`/waiting display values.

## Safety boundary

The package has no command publishers, NavigateToPose action client, CAN
transport, or Mission Manager route publisher. `/vehicle_cmd_safe` is received
only for display.
