# PlanNav Edge Visualization Contract

Status: `P5A-MK2G2C` visual-only contract

This contract adds only visual direction and identity cues to the existing
PlanNav topology editor. It does not change topology persistence, Dijkstra,
RouteSpec/RouteMission generation, or runtime command ownership.

## Direction arrows

`MapView.draw_edges()` uses each persisted edge's `from_id` and `to_id` to
derive the scene endpoints. A filled triangular arrowhead is placed along the
rendered edge and points from `from_id` toward `to_id`. The arrow never derives
direction from color, node numbering, or screen orientation.

Separate persisted reverse records such as `A -> B` and `B -> A` are both
rendered. Their rendered polylines use a deterministic `+10 px` / `-10 px`
perpendicular separation, ordered by persisted `edge_id`, so neither arrow nor
identity is hidden. A single persisted `direction=bi` record receives one
arrow in each legal direction while retaining one edge identity.

## Edge identity labels

The label is sourced from the same `edge_id` field loaded from `edges.txt` and
carried by the route identity layer. The normal view displays:

`<persisted edge_id> (<length metres>)`

near the rendered edge midpoint. For paired records, the label follows its
correspondingly offset edge. There is no view toggle; edge IDs are visible by
default.

## Layering and preserved behavior

- base edge geometry: z-order 1;
- selected route: existing blue dashed geometry at z-order 2;
- direction arrows: z-order 3;
- edge identity/length labels: z-order 4.

This keeps the selected route visible while keeping the new direction and
identity cues readable. Waypoint selection, node labels, planning clicks,
route clearing, directed Dijkstra, and RouteMission publication are unchanged.

The helper is observation/rendering-only. It creates no ROS clients,
publishers, network interfaces, velocity commands, or CAN access.

## Deferred items

Node-ID reindexing, occupancy-map display, automatic label layout, color/layout
redesign, and runtime supervision remain outside this milestone.
