# Phase 5A MK2F4 Mock Autonomy Git Checkpoint

Checkpoint: `phase5a_mk2f4_mock_autonomy_checkpoint`

Latest accepted qualification:
`P5A_MK2F4_MISSION_MANAGER_TO_NAVIGATETOPOSE_SAFETY_MOCK_PASS`

## Scope

This checkpoint preserves the accepted Phase-5A project implementation through
Mission Manager orchestration:

```text
RouteMission / topology route intent
  -> Mission Manager
  -> NavigateToPose / BT Navigator
  -> SmacPlannerHybrid / DUBIN
  -> MPPI / Ackermann
  -> Collision Monitor
  -> Generic Safety Gate
  -> gate_to_labmate_bridge
  -> vendored wheelchair_controller
  -> MockTransport
```

The qualification boundary is mock autonomy. MockTransport was used below the
MK-mini backend; physical CAN, chassis actuation, physical motion, and final
chassis stationary/fault semantics remain unqualified future work.

## Accepted behavior preserved

- Required dual-sensor freshness and fail-close safety remain authoritative.
- Planner and controller are forward-only Ackermann-compatible operation:
  SmacPlannerHybrid/DUBIN and MPPI/Ackermann, minimum turning radius 1.75 m.
- Mission Manager owns mission-driven NavigateToPose action ownership.
- Pause cancels the active goal and preserves the current waypoint.
- Resume resends the preserved waypoint with a new bounded initial-pair epoch.
- Gate interruption forces the lower chain to safe zero and cannot be treated
  as mission completion.
- Mission cancellation is deterministic and dispatches no next waypoint.
- Initial causal-pair acquisition tolerates bounded healthy 20 Hz MPPI / 10 Hz
  Collision Monitor ambiguity, while post-establishment stale behavior remains
  fail-close.

## Map and runtime hygiene

Canonical RTAB-Map reference:
`map/rtabmap_2d.db`

Reference SHA-256:
`73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`

RTAB-Map localization uses a runtime working copy under
`/home/dog/phase5_runtime/rtabmap/`; runtime copies and ROS logs are not
repository content.

## Deferred work

- Physical CAN writer authorization and physical motion qualification.
- Final-chassis gear, parking, and fault-state semantics.
- GUI/RViz/operator visualization design and implementation.

The next planned area is GUI/RViz/operator visualization. It is not part of
this checkpoint.
