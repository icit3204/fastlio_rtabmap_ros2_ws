# Phase 5A MK2H1 Live Foundation and Route Reconciliation Checkpoint

## Checkpoint boundary

This checkpoint records two accepted, separate results:

1. MK2H1 established the live dual-LiDAR, localization, Nav2, safety,
   Operator GUI, RViz, and MockTransport composition through the pre-mission
   admission gate.
2. MK2H1R reconciled PlanNav's historical raw-odometry topology coordinates
   to the canonical RTAB optimized `map` frame for one validated topology
   corridor.

`P5A_MK2H1_BLOCKED_ROUTE_LOCALIZATION_MISMATCH` remains the MK2H1 live-run
result. It was stopped before RouteMission publication and before Operator GUI
START. This checkpoint does **not** claim START, NavigateToPose, planner,
MPPI, PAUSE, RESUME, CANCEL, or a live mission pass.

## Preserved H1 foundation

- `stationary_real_localization_validity` is an opt-in real validity producer
  for the live profile; the predecessor mock profile remains available.
- `mk2h1_live_operator_mock.launch.py` composes real sensing/localization and
  operator visualization with `MockTransport` only.
- The controller-valid signal remains explicitly documented as mock command
  permission, not physical chassis health.
- Tests prohibit demo-fixture overlap, direct motion authority, and physical
  transport in this profile.

## Canonical map/topology authority

- Canonical RTAB DB: `map/rtabmap_2d.db`
- DB SHA-256: `73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`
- Canonical topology workspace: `map/rtabmap_2d/`
- Topology version:
  `sha256:f8bd2b688ce827446ea88f42f19777f97c918c2dc1832091fa9b379a85000d70`
- Historical source workspace: `map/采集/图书馆2/`, whose database is
  byte-identical to the canonical database and remains preserved.

The new workspace is a rigid SE(2) transformed copy of that source corridor:
yaw `+0.579629264°`, translation `(-0.060278951, -0.288028419) m`, scale
`1.0`. It is supported by nine node fit correspondences and 188 independent
trajectory validation correspondences. The transform is not general map
calibration and must not be used outside its documented corridor.

## Route and retry boundary

The static route `4 -> 1 -> 2` resolves to `edge-000002`, `edge-000001` in
frame `map` and passes Mission Manager route validation. PlanNav Start Node
is the first NavigateToPose metric goal; it is not a claim that the robot is
already at that node. The next live retry must require real planner
reachability from current TF to node 4 before command/Gate readiness.

MockTransport remains mandatory. Physical CAN, physical motion, physical
controller-health qualification, and live mission completion remain
unqualified.
