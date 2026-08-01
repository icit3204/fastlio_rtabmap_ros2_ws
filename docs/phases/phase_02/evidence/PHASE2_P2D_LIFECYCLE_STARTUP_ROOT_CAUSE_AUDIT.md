# Phase 2 P2-D Lifecycle Startup Root-Cause Audit

Final status: `PHASE2_P2D_LIFECYCLE_STARTUP_AUDIT_COMPLETE`

Primary classification: `LIFECYCLE_AUTOSTART_OR_DISCOVERY_RACE`

Confidence: medium.

## Executive conclusion

The intermittent failure is most consistent with startup/discovery/transition timing around lifecycle-manager autostart, not a bad map file, not a PathAngleCritic candidate side effect, and not a persistent map_server configure defect.

The known failure pattern was:

1. lifecycle manager requested map_server configure;
2. map_server began reading `phase2_clean_map.pgm`;
3. lifecycle manager declared `Failed to change state for node: map_server` before map read completion;
4. map_server completed map read roughly one second later.

This audit could not reproduce the failure in fresh domains. Manual map_server transition succeeded once discovery was complete. Minimal map_server + lifecycle-manager autostart passed three times. Full tracked config passed once. Full candidate overlay passed once.

## What was ruled out

### Map file/configure defect

Rejected as primary.

- Manual map_server configure/activate/deactivate/cleanup passed in domain 202.
- Minimal lifecycle-manager runs loaded the same map and reached active repeatedly.
- Full tracked and candidate stack runs loaded the same map and reached active.
- Map file: 4,679,169 bytes on ext2/ext3 filesystem.
- Observed PGM read durations were about 0.84–0.91 s in successful runs.

### External candidate side effect

Rejected.

- Candidate overlay did not alter map_server or lifecycle-manager sections.
- Full candidate startup in domain 209 reached `Managed nodes are active`.
- `PathAngleCritic` loaded successfully; no action was sent.

### Stale DDS/process interference

Not supported by this audit.

- Fresh domains 201–209 were used.
- ROS daemon was stopped before diagnostics.
- Fresh-domain runs passed.
- Prior failures may have been influenced by reused domain 195, but this audit did not prove stale DDS as the direct cause.

### CPU/storage starvation

Not proven.

- Memory was available; swap use was low.
- `vmstat` showed no sustained I/O wait during sampling.
- Successful PGM reads were under one second.
- A resource spike could still contribute intermittently, but evidence is insufficient to classify it as primary.

## Positive evidence for lifecycle startup/discovery race

- Manual case A attempted lifecycle CLI before discovery and returned `Node not found`; case B succeeded when discovery completed.
- `ros2 lifecycle get` intermittently returned `Node not found` even after lifecycle-manager logs showed active nodes. This shows CLI/DDS graph discovery can lag or be unreliable in these short diagnostic windows.
- Prior failures show lifecycle manager declaring transition failure while map_server configure work was still visibly ongoing.
- Minimal explicit delayed startup succeeded with `ManageLifecycleNodes_Response(success=True)`.
- No repository parameter sets a lifecycle service timeout or delayed startup behavior.

## Effective launch/parameter truth

- `map_server` appears once in `phase2_core_nav2.launch.py` as node name `map_server`.
- `lifecycle_manager_navigation` appears once.
- `node_names` order: `map_server`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`.
- `autostart` default: `true`.
- Phase 2 launch does not set `bond_timeout`, `attempt_respawn_reconnection`, `bond_respawn_max_duration`, or service/transition timeout.
- Tracked config sets `map_server.ros__parameters.yaml_filename: ""`; launch overrides it with the installed package map path.
- No `/data/...` or legacy map asset was used.

## Smallest proposed correction for a future task

Do not change map data or controller parameters for this issue.

Smallest correction to validate next:

1. keep the same nodes and parameters;
2. set lifecycle manager autostart false for the diagnostic launch path, or add a Phase-2-only delayed startup wrapper;
3. wait until every managed lifecycle service is discoverable;
4. wait a short quiet interval, e.g. 2 seconds;
5. call `/lifecycle_manager_navigation/manage_nodes` STARTUP once;
6. rerun the PathAngleCritic direct FollowPath gate.

Files/parameters that may change in a future correction:

- `src/parking_robot_bringup/launch/phase2_core_nav2.launch.py`, only if adding an explicit Phase-2-safe delayed startup option.
- Or no tracked source change if the next test uses an external diagnostic launch wrapper.

Items that must remain unchanged:

- map YAML/PGM;
- MPPI/path critic parameters during lifecycle diagnosis;
- progress checker;
- goal checker;
- costmaps/inflation;
- fake-base source;
- command topics and remaps;
- TF frames.

## Limitations

- The exact lifecycle manager C++ implementation body is not installed, so timeout/retry internals were inferred from headers, package artifacts, binary strings and runtime behavior.
- The failure was not reproduced in fresh domains during this audit.
- `ros2 lifecycle get` showed measurement-layer discovery unreliability; lifecycle-manager logs were treated as authoritative for manager success/failure.

## Integrity

No tracked source/configuration was modified by this audit. No action or command topic was used.
