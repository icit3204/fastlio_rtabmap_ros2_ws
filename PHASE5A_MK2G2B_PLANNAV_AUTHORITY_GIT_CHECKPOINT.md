# Phase 5A MK2G2B PlanNav Authority Git Checkpoint

Checkpoint: `P5A_MK2G2B_GIT_GITHUB_CHECKPOINT_PASS`

This repository checkpoint follows the accepted GUI/RViz checkpoint
`phase5a_mk2g1c_operator_visualization_checkpoint` and records the accepted
PlanNav runtime-authority separation.

## Accepted architecture

PlanNav is the topology and route-intent authoring tool. It retains node and
edge editing, direction, edge identity, directed weighted Dijkstra, route
highlighting, RouteSpec construction, and explicit typed `RouteMission`
publication on `/mission/route`.

The Operator GUI is the sole intended runtime mission supervisory GUI and owns
START, PAUSE, RESUME, CANCEL, and mission/navigation/safety status. RViz owns
runtime spatial visualization.

## Separation recorded

PlanNav's normal interface no longer exposes legacy UDP transport, pursuit
controls, operation/authority selectors, mission START/PAUSE/RESUME/CANCEL,
MissionState runtime supervision, or the historical `/plan_nav` runtime
publisher. The route bridge has only the reliable transient-local
`/mission/route` publisher and does not start navigation.

Historical UDP and `/plan_nav` modules remain unbound source for reference.
The historical UDP path is motion-capable but is not required by PlanNav's
topology/routing core and is not normal/default authority.

## Qualification boundary

- mock autonomy through Mission Manager remains accepted;
- Operator GUI and canonical RViz visualization remain accepted;
- PlanNav topology/routing behavior is unchanged;
- RouteMission fields and publication boundary are preserved;
- physical CAN, physical motion, and final-chassis behavior are not qualified;
- node-ID reindexing remains a documented future risk, not repaired here;
- visual polish remains deferred and is not included;
- PlanNav visual/functional review remains the source of the recorded baseline.

The canonical topology regression remains 1→10 with nodes 1–10, 34.133 m,
the reviewed nine edge IDs, forward directions, and 10→1 as
`NO_TOPOLOGICAL_ROUTE`.

Canonical RTAB-map and runtime ROS evidence remain outside this Git checkpoint.
No runtime database, build output, logs, screenshots, or supervisor archives
are repository content.

## Verification

Offline PlanNav tests: 23 passed. Python compilation, RouteMission publication
and no-runtime-control checks, GUI startup smoke, topology regression, and
affected-file `git diff --check` passed. The full worktree check still reports
the pre-existing trailing whitespace at `键盘控制.md:22`; that unrelated file
is intentionally not part of this checkpoint.
