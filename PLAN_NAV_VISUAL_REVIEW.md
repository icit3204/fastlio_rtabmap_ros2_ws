# PlanNav Existing GUI Visual Review

Disposition: `PLAN_NAV_EXISTING_GUI_USABLE_WITH_OPTIONAL_IMPROVEMENTS`

This is a factual review, not a redesign approval.

## Review method

- Launched the existing standalone entry point with `python3 main.py` on the
  available desktop (`DISPLAY=:0`).
- Captured the initial empty workspace.
- Imported `plan_nav/underGround_split1.db` through the existing GUI handler,
  but redirected the database and `underGround_split1/` topology into a
  temporary `/tmp/plan_nav_review.*` copy.
- Captured the loaded topology view.
- Exercised the existing plan-click handler in `mission_nav2` authority mode
  against temporary data, so the legacy UDP sender was not started.
- Stopped all PlanNav review processes. No Mission Manager, Nav2, controller,
  backend, SocketCAN, or live sensor process was started.

Evidence images are retained in the MK2G2A supervisor evidence archive:

- `plan_nav_initial.png`
- `plan_nav_loaded_window.png`
- `plan_nav_route_window.png`

## Observed layout

The window is a light-background PyQt5 editor titled `Underground Map Editor`.
The left pane contains mode, DB import, statistics, six tools, playback/reset
and processing controls, Mission/Nav2 controls, and a UDP preview. The center
is a large QGraphicsView canvas. The right pane contains logs, recognition
settings, UDP settings, and trajectory parameters. A bottom status bar reports
node/edge/path state and pan/zoom guidance.

The loaded review view showed a long trajectory with orange waypoint labels,
orange single-direction edges, green paired reverse-edge geometry, a red
trajectory-start marker, and a blue dashed selected route. The presentation is
understandable as a development topology editor, though the dense upper node
cluster has overlapping labels and the Chinese/English mixed labels require
some project familiarity.

## Visual findings

| Area | Classification | Evidence |
|---|---|---|
| Nodes and trajectory | USABLE BUT COULD IMPROVE | Nodes are orange circles with `WP-##` labels; trajectory is visible. |
| Edge geometry | USABLE BUT COULD IMPROVE | Edge paths and lengths are visible; dense areas can overlap. |
| Edge direction | CONFUSING | Orange/green coloring and paired geometry provide an indirect cue; no arrowheads are rendered. |
| Edge ID | MISSING | `edge-######` is persisted but the canvas only draws length text. |
| Selected route | CLEAR | Blue dashed overlay plus bottom path length/node count and log entry. |
| Start/goal selection | USABLE | Two sequential clicks in planning mode; logs identify start and goal. |
| Pan/zoom | CLEAR | Drag/wheel interaction is visible in the status hint and worked as the QGraphicsView interaction. |
| Controls | USABLE BUT COULD IMPROVE | Controls are readable, but the single window exposes authoring, legacy transport, and mission controls together. |
| Occupancy map | MISSING IN CURRENT IMPORT PATH | Current `import_db` uses trajectory view; map decoding is commented. |

No item was changed to obtain this disposition.
