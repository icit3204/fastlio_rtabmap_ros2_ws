# P5A MK2H1R PlanNav Coordinate Contract

## Finding

PlanNav's historical import path persists the RTAB database `Node.pose` values
directly. In these databases, that column is the input odometry trajectory. It
is not the `Admin.opt_poses` optimized RTAB graph used as the canonical ROS
`map` frame during localization.

The relevant flow is:

1. `plan_nav/core/db_loader.py::load_db()` selects `id, stamp, pose` from
   `Node` and parses each 3x4 transform.
2. `MainWindow.import_db()` passes those values to the trajectory view and to
   `TopologyManager`.
3. waypoint creation stores those same metric x/y/z/yaw values in
   `nodes.txt`.
4. edge trajectory extraction stores the same coordinates in
   `edge_*_traj.txt`.
5. `MapView` converts world metres to scene pixels only for drawing. That
   screen conversion does not alter persisted topology coordinates.
6. `build_sparse_route_spec()` labels persisted waypoint coordinates as
   frame `map`; it performs no odometry-to-map conversion.

Thus old workspaces can be metrically coherent internally while still being
wrong for current NavigateToPose `map` goals.

## Direct provenance checks

- All 20 `underGround_split1` topology nodes match timestamp-associated raw
  `Node.pose` coordinates in `plan_nav/underGround_split1.db` with 0.000046 m
  RMS and 0.000065 m maximum error.
- All nine `map/采集/图书馆2` topology nodes match timestamp-associated raw
  `Node.pose` coordinates in its sibling database with 0.000044 m RMS and
  0.000061 m maximum error.
- The `图书馆2` database is byte-identical to canonical
  `map/rtabmap_2d.db`.
- The MK2H1 live pose is 0.00685 m from canonical optimized graph node 945,
  confirming that the live ROS `map` pose corresponds to `Admin.opt_poses`,
  not the raw `Node.pose` endpoint.

## Canonical graph authority

The canonical database contains 865 optimized poses. Its optimized map-frame
bounds are:

- x: -11.6216 to 113.3062 m
- y: -0.5427 to 117.0285 m
- centroid: (44.1116, 55.6255) m

The live MK2H1 pose `(-7.260, 12.452)` lies on that graph near optimized node
945 `(-7.2585, 12.4587)`.

## Accepted reconciliation

The canonical database's historical sibling topology is converted into a new
sibling workspace at `map/rtabmap_2d/`. A rigid SE(2) conversion is applied to
nodes and all referenced edge trajectory points. No scale or nonlinear warp
is used. Source files and both database files remain unchanged.

The GUI's existing import rule selects the sibling work directory from the DB
stem, so importing `map/rtabmap_2d.db` now loads the reconciled workspace
without a PlanNav source change.

The canonical workspace records both `topology_manifest.json` and
`reconciliation_manifest.json`. RouteMission poses from this workspace are
therefore legitimately labelled `map`.
