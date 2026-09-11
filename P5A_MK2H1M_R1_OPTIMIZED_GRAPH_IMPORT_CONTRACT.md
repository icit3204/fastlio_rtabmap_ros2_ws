# P5A-MK2H1M-R1 optimized graph import contract

Status: importer and operator-approved current-room topology qualified.

## Authority and storage

- Database: `/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/20260911_194712_current_room/rtabmap_2d.db`
- Database version: RTAB-Map `0.23.4`
- Database SHA-256: `550227ccf65470946d78317f28f200f3dcc904dfa053291f69f5f06b67fdd127`
- Import mode: `OPTIMIZED_GRAPH_MAP_FRAME`
- Importer API: `core.db_loader.load_optimized_map_trajectory()`
- Coordinate semantic: saved optimized RTAB graph, metres/radians, ROS frame `map`
- Importer version: `p5a-mk2h1m-r1-v1`

RTAB-Map's own `DBDriverSqlite3::saveOptimizedPosesQuery()` stores the graph node IDs as a compressed `1 x N`, `CV_32SC1` matrix in `Admin.opt_ids`. It stores the corresponding transforms as a compressed `1 x (12N)`, `CV_32FC1` matrix in `Admin.opt_poses`. Each transform is twelve row-major float32 values representing a 3x4 rigid transform. `loadOptimizedPosesQuery()` performs the inverse operation, checks the types and 12:1 cardinality, and constructs one RTAB `Transform` per ID.

The PlanNav decoder implements that exact saved representation. It does not optimize the graph, infer correspondences, apply a fitted frame transform, or fall back to `Node.pose`. All optimized IDs must exist in the DB `Node` table; timestamps are joined by actual RTAB node ID.

The current DB has no saved `Admin.opt_map` or `opt_map_resolution`. In this trajectory-only case, PlanNav uses `0.01 m/pixel` solely as a reversible display scale so waypoint and persisted edge-ID labels remain readable. This does not scale, transform, or round the imported metric node records. If RTAB stores an occupancy-map resolution, that stored value remains authoritative.

## Read-only behavior

The optimized API opens SQLite using a `file:` URI with `mode=ro`. It has no update, insert, delete, pragma-write, graph-optimization, or map-generation path. The DB hash was checked before and after direct importer tests and graphical import; it remained byte-identical.

## Independent reference oracle

The independent oracle is RTAB-Map's installed native C++ API:

1. `rtabmap::DBDriver::create()`
2. `openConnection(database, false, true)` (read-only)
3. `loadOptimizedPoses()`
4. `Transform::getTranslationAndEulerAngles()`

The resulting 38-pose oracle is `P5A_MK2H1M_R1_REFERENCE_POSES.csv`. It was produced without importing any PlanNav Python module.

## Agreement

All 38 node IDs and poses agree in ascending RTAB graph-node order.

- Maximum translation residual: `6.15e-10 m`
- Maximum roll/pitch/yaw wrapped residual: `1.57e-7 rad` (`8.99e-6 deg`)

The residual is bounded by decimal rendering of the float32 native oracle. No map reconciliation offset, scale, or tolerance-scale correction is present.

## Legacy contract

`core.db_loader.load_db()` remains available and unchanged in meaning: it reads raw `Node.pose` and now labels the result `LEGACY_RAW_NODE_POSE` / `RAW_RTAB_NODE_POSE`. Existing historical workspaces continue to use their prior import path and are not transformed or rewritten.

The GUI presents two explicit controls:

- `导入 .db 文件`: historical raw `Node.pose` import;
- `导入优化 map DB`: authoritative optimized graph / ROS `map` import.

Only the optimized control is valid for new map-frame topology authoring.

## Workspace binding

Optimized import uses the dedicated sibling workspace `plannav/`. Its `plannav_workspace_manifest.json` binds the workspace to the absolute DB path, exact DB SHA-256, mapping session ID, frame, coordinate semantic, import mode, and importer version. Reopening a mismatched binding raises an error rather than rebinding existing topology.
