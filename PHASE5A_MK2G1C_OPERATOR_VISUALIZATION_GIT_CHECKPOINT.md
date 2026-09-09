# Phase 5A MK2G1C Operator Visualization Git Checkpoint

Checkpoint: `phase5a_mk2g1c_operator_visualization_checkpoint`

Latest accepted milestone: `P5A_MK2G1C_OPERATOR_GUI_RVIZ_INTEGRATED_DEMO_PASS`

Predecessor checkpoint: `phase5a_mk2f4_mock_autonomy_checkpoint`

## Included

This checkpoint contains the accepted `robot_visualization` package, the
standalone `operator_gui` package, the canonical RViz configuration, the
observation-only helper, the integrated single-fixture GUI/RViz demo launch,
the GUI/RViz contracts and matrices, and their focused tests. It also contains
the visualization-only `FollowPath.visualize: true` setting, which enables the
existing MPPI visualization topics without changing control parameters.

The integrated demo was qualified with one synthetic authority. It showed the
same RouteMission/MissionState in GUI and RViz, the global plan, nearest point,
current goal, local costmap, robot/footprint, and selected Optimal Trajectory.
Candidate trajectory data existed but remained disabled by default.

## Explicit boundaries

PlanNav is preserved AS-IS and is not included in this checkpoint as a changed
component. The separate PlanNav visual and functional review is the next
planned area, but is not started here.

The checkpoint does not qualify physical CAN, physical motion, live
sensor-backed GUI/RViz navigation, final chassis behavior, or operator Safety
Reset authority. Aim Point remains deferred. A canonical Collision Monitor
CLEAR/SLOW/STOP state authority remains deferred; the GUI displays only the
accepted validity representation.

Canonical RTAB-Map database SHA-256 at checkpoint preparation:

`73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`
