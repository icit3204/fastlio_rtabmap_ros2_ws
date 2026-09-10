# P5A MK2H1R Exact H1 Retry Contract

## Fixed authorities

- Canonical database: `map/rtabmap_2d.db`
- SHA-256: `73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`
- PlanNav workspace: `map/rtabmap_2d/`
- Topology version:
  `sha256:f8bd2b688ce827446ea88f42f19777f97c918c2dc1832091fa9b379a85000d70`
- Representative directed route: nodes `4 -> 1 -> 2`
- Edge IDs: `edge-000002`, `edge-000001`
- Route frame: `map`

## Required preconditions

1. Reconnect MID-360 and T-mini only; keep physical CAN disconnected.
2. Verify canonical DB hash and use the qualified runtime-copy mechanism.
3. Launch `robot_bringup mk2h1_live_operator_mock.launch.py`.
4. Verify real localization/perception validity and single-publisher
   authority; controller validity remains labelled mock permission.
5. Verify backend logs and graph prove MockTransport with no SocketCAN.
6. Import `map/rtabmap_2d.db` in PlanNav. Its normal sibling-workspace rule
   must load `map/rtabmap_2d/` and the recorded topology version.
7. Select Start node 4 and Goal node 2. Publish exactly one typed RouteMission.
8. Apply the current-pose admission contract. Do not reject solely because
   the robot is 17.168 m from node 4; node 4 is the first NavigateToPose goal,
   not a claimed current location.
9. Use Operator GUI START. Mission Manager must remain the sole
   NavigateToPose owner.
10. Require the real planner to produce a valid global path from current TF to
    node 4 before command/Gate readiness. If no path exists, stop with a
    planner/map reachability blocker.

## Retry boundary

The retry remains real sensing/localization/Nav2 with MockTransport only. It
does not qualify physical CAN, physical controller health, or robot motion.
No live retry is performed by MK2H1R.
