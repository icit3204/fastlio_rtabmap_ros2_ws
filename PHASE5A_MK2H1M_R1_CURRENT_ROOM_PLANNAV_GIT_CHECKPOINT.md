# Phase 5A MK2H1M-R1 current-room PlanNav Git checkpoint

This checkpoint records the accepted `P5A_MK2H1M_R1_OPTIMIZED_PLANNAV_IMPORT_AND_TOPOLOGY_PASS`.

## Scope

- A new current-room RTAB mapping session is preserved at `map/current_room_sessions/20260911_194712_current_room/`.
- The session database is `rtabmap_2d.db`, with SHA-256 `550227ccf65470946d78317f28f200f3dcc904dfa053291f69f5f06b67fdd127`.
- The old historical database `map/rtabmap_2d.db` remains unchanged at SHA-256 `73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`.
- PlanNav has an explicit `OPTIMIZED_GRAPH_MAP_FRAME` read-only importer for RTAB `Admin.opt_ids` and `Admin.opt_poses`.
- The legacy raw `Node.pose` importer remains available for historical workspaces and is not used for new map-frame authoring.
- The current-room workspace is `map/current_room_sessions/20260911_194712_current_room/plannav/` and is bound to the exact DB hash, session ID, `frame=map`, and `OPTIMIZED_RTAB_GRAPH` semantic.

## Approved topology

- Topology ID: `topology-sha256:47b58b2498a2ab9b43fb934561bc207cfe5d2ef0b79e26e2f00e70a3c4cfb1b8`
- Topology version: `sha256:948217a0129dbf785b4364cdff94fe5424563719495a6aeb1bfc2780814d1a2e`
- Nodes: 3
- Edges: 2 directed one-way edges
- `edge-000001`: 1 -> 2
- `edge-000002`: 2 -> 3
- Forward route: `1 -> 2 -> 3`, `6.644 m`, typed RouteMission `VALID`
- Reverse route: `3 -> 1`, `NO_TOPOLOGICAL_ROUTE` (no reverse edges were approved)

The topology was displayed in PlanNav and explicitly approved by the operator before persistence. No historical topology was overwritten.

## Qualification boundary

The importer was compared against the independent native RTAB `DBDriver::loadOptimizedPoses()` reference for all 38 optimized poses. Maximum translation residual was `6.15e-10 m`; maximum wrapped orientation residual was `1.57e-7 rad`.

This checkpoint does not qualify autonomous navigation, live planner reachability, physical CAN, physical controller health, drivetrain behavior, or robot motion. No sensors, ROS navigation, SocketCAN, or autonomous mission were started while producing this R1 work.

The next milestone is live localization plus planner-only preflight to WP-01, with physical CAN still disconnected and without Mission START.
