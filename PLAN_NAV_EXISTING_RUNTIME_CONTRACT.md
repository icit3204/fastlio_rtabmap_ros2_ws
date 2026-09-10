# PlanNav Existing Runtime Contract — Review Only

Review milestone: `P5A-MK2G2A`

This document records the current PlanNav implementation exactly as found on
2026-09-10. It is not an implementation proposal. No PlanNav source,
configuration, topology, or launch file was changed.

## Current implementation

- Package/layout: standalone `plan_nav/` tree outside the colcon `src/`
  workspace.
- GUI framework: Python 3 + PyQt5.
- Entry point: `plan_nav/main.py`.
- Direct launch: `cd /home/dog/fastlio_rtabmap_ros2_ws/plan_nav && python3 main.py`.
- Convenience launcher: `plan_nav/run.sh`; it may create/use
  `$HOME/venvs/underground_map_editor` and install `plan_nav/requirements.txt`.
- Window title: `Underground Map Editor`.
- Main window: `plan_nav/ui/main_window.py`, with `Sidebar`, `MapView`, and
  `LogPanel`.

No competing current PlanNav GUI was found. Historical references describe
the same standalone application rather than a second active implementation.

## Startup and data flow

1. `main.py` creates the Qt application and `MainWindow`.
2. `导入 .db 文件` selects an RTAB-Map SQLite database.
3. `core/db_loader.py` reads ordered RTAB-Map `Node` poses, `Link` rows, and
   map metadata.
4. The GUI creates a sibling work directory named from the database stem and
   loads `nodes.txt`, `edges.txt`, and `topology_manifest.json` through
   `TopologyManager`.
5. The current import path calls `MapView.setup_trajectory_view`; the
   `decode_map`/`load_map` occupancy-image path is present but commented out
   in `MainWindow.import_db`. The current canvas therefore shows the trajectory
   workspace and overlays, not the decoded occupancy image.

## Visible GUI layout

The current window is a three-pane editor:

- left: debug/operation mode, DB import, statistics, editing tools, playback,
  reset, data processing, `auto_node`, Mission/Nav2 controls, and UDP preview;
- center: QGraphicsView trajectory/topology canvas with pan and wheel zoom;
- right: event log, node-recognition parameters, UDP parameters, and trajectory
  control parameters;
- bottom: node/edge/path status and pan/zoom hint.

The visual review found readable controls and a coherent spatial canvas. The
upper topology cluster becomes visually dense and labels can overlap. Edges
show length labels and color distinctions, but no explicit edge arrows or
edge-ID labels are drawn.

## Topology authority

The current persisted topology example is
`plan_nav/underGround_split1/`:

- `nodes.txt`: numeric node ID, label, annotation, x/y/z, yaw in degrees,
  timestamp, and trajectory index;
- `edges.txt`: `edge_id`, `from_id`, `to_id`, `length_m`, `direction`, and
  `traj_file`;
- `topology_manifest.json`: schema/version, content hashes, topology version,
  and monotonic next-edge-ID index;
- `edge_*_traj.txt`: optional dense path data used for edge rendering and
  trajectory concatenation.

The current schema is version 2. `edge_id` is parsed, persisted, included in
topology identity, and carried into routes. The GUI does not display the
`edge_id` text. It renders a single line for a reverse edge pair in green and
single-direction geometry in orange; this is a visual convention, not an
explicit direction arrow.

## Route and Mission boundary

`plan_nav/core/pathfinder.py` builds a `networkx.DiGraph` and uses weighted
Dijkstra. A `uni` edge contributes only `from_id -> to_id`; a `bi` edge also
contributes the reverse traversal.

`plan_nav/core/mission_bridge.py` converts the selected topology route to a
`RouteSpec`, then to `parking_robot_interfaces/msg/RouteMission`:

- `mission_id`;
- deterministic `route_id` derived from topology version, node IDs, edge IDs,
  and traversal directions;
- `topology_version`;
- ordered `node_ids`;
- inter-node `edge_ids`;
- `edge_directions` (`1` forward, `-1` reverse traversal);
- map-frame `PoseStamped` poses.

The bridge publishes `/mission/route` only when the Mission/Nav2 publish
operation is explicitly invoked. It subscribes to `/mission/state` and uses
`Trigger` clients for `/mission/start` and `/mission/cancel`, plus a `SetBool`
client for `/mission/pause`. Mission Manager remains the runtime authority.

The GUI also retains a legacy authority mode. In legacy mode a selected route
starts `UdpSender`, which emits the historical 16-byte `(R, v)` UDP stream to
the configured endpoint. Mission/Nav2 mode explicitly stops that sender and
records the dense `/plan_nav` path as display-only. This legacy path was not
started during the review.

## Persistence contract

The following are explicit disk mutations when editing is used:

- node creation, node annotation, node deletion, and reset write `nodes.txt`;
- edge creation, deletion, direction toggle, and reset write `edges.txt`;
- edge creation may write an `edge_*_traj.txt` file;
- topology edits refresh `topology_manifest.json` when the v2 format is
  present;
- importing a database creates/uses the database-stem work directory and can
  repair missing trajectory indices/edge trajectory files;
- settings changes write `config/settings.json`.

There is no separate Save/Export dialog for topology; normal editing actions
persist immediately. The review imported a copied database and copied
topology into `/tmp`, not the repository authority.

## Review boundary

PlanNav remains the topology/route-authoring tool. Operator GUI owns runtime
mission supervision, and RViz owns runtime spatial visualization. The current
PlanNav Mission controls and legacy UDP mode are documented as an existing
safety/control overlap for supervisor decision; they were not removed or
changed in this review.
