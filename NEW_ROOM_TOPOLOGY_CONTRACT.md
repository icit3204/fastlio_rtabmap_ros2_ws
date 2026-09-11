# New-room topology contract

Status: `OPERATOR_APPROVED` and persisted on 2026-09-11.

## Frozen map identity

- Mapping session: `20260911_194712_current_room`
- Database: `/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/20260911_194712_current_room/rtabmap_2d.db`
- Database SHA-256: `550227ccf65470946d78317f28f200f3dcc904dfa053291f69f5f06b67fdd127`
- Workspace: `/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/20260911_194712_current_room/plannav`
- Frame: `map`
- Coordinate semantic: `OPTIMIZED_RTAB_GRAPH`

## Approved minimal topology

The approved topology uses three actual optimized trajectory poses. It is persisted in `nodes.txt`, `edges.txt`, two trajectory-segment files, `topology_manifest.json`, and the DB-bound PlanNav workspace manifest.

| Node ID | Role | Source graph node | x (m) | y (m) | z (m) | yaw |
|---|---|---:|---:|---:|---:|---:|
| 1 | recording start area | 1 | 0.466870 | -0.188696 | -1.398704 | 0.38 deg |
| 2 | middle/open corridor | 144 | 4.130979 | -0.781886 | -1.336540 | -3.76 deg |
| 3 | far turning area | 176 | 6.988910 | -0.127887 | -1.308576 | 98.41 deg |

Approved directed edges:

- `edge-000001`: 1 -> 2
- `edge-000002`: 2 -> 3

Route diagram: `START (1) -> MID (2) -> FAR (3)`.

Return edges are deliberately absent from this first proposal. They should only be added if the operator explicitly wants a return route and later planner-only checks support the persisted goal orientations.

## Identity and route proof

- Operator response: `approve topology`
- Topology ID: `topology-sha256:47b58b2498a2ab9b43fb934561bc207cfe5d2ef0b79e26e2f00e70a3c4cfb1b8`
- Topology version: `sha256:948217a0129dbf785b4364cdff94fe5424563719495a6aeb1bfc2780814d1a2e`
- Dijkstra 1 -> 3: nodes `1,2,3`, edges `edge-000001,edge-000002`, length `6.644 m`
- Dijkstra 3 -> 1: `NO_TOPOLOGICAL_ROUTE` (expected; no return edges were approved)
- Typed RouteMission: `VALID`, frame `map`, edge directions `[1,1]`

The historical node-ID reindex risk still applies to subsequent destructive GUI edits; it was not changed in this milestone.
