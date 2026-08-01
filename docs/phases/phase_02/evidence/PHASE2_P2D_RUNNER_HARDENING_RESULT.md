# PHASE2 P2D Runner Hardening Result

Generated: 2026-08-01T19:42:51.995412

## Decision

`P2D_RUNNER_HARDENING_RESPONSE_REPEATABILITY_NEEDS_REVIEW`

Primary classification: `RUNNER_HARDENING_INTRODUCES_NEW_FAILURE`

## Preserved pre-edit state

- Timestamp: `20260801_193333`
- Dirty-state report: `/home/dog/phase2_reports/PHASE2_P2D_PRE_HARDENING_DIRTY_STATE_20260801_193333.txt`
- Tracked diff: `/home/dog/phase2_reports/PHASE2_P2D_PRE_HARDENING_TRACKED_DIFF_20260801_193333.patch`
- Untracked archive: `/home/dog/phase2_reports/PHASE2_P2D_PRE_HARDENING_UNTRACKED_FILES_20260801_193333.tar.gz`
- Manifest: `/home/dog/phase2_reports/PHASE2_P2D_PRE_HARDENING_MANIFEST_20260801_193333.tsv`

## Implementation summary

- Runner state machine now includes `GOAL_REQUEST_SENT`.
- Runner keeps action futures, goal handle, and cancel future as instance attributes.
- Runner has `goal_response_timeout_sec` and expanded terminal JSON fields.
- Runner uses an explicit `SingleThreadedExecutor`.
- Runner supports `validation_mode=action_response_probe` with current-pose acceptance rules.
- Fake-base shutdown handles context-invalid `RCLError` only when context is already invalid.

## Build and tests

- Build root: `/home/dog/phase2_builds/p2d_runner_hardening_20260801_193333`
- Package-only build: PASS
- Package tests: PASS, 55 tests

## Three-domain current-pose response trials

| Trial | Domain | Runner exit | Goals sent | Goal response | Goal accepted | Action result | JSON pass | BT goal succeeded log | Fake-base clean exit | Failure reasons |
|---:|---:|---:|---:|---|---|---|---|---|---|---|
| 1 | 176 | 2 | 1 | False | False | None | False | True | True | NavigateToPose goal was rejected |
| 2 | 177 | 2 | 1 | False | False | None | False | True | True | NavigateToPose goal was rejected |
| 3 | 178 | 2 | 1 | False | False | None | False | True | True | NavigateToPose goal was rejected |

## Interpretation

All three lifecycle startups succeeded and each trial sent exactly one current-pose NavigateToPose request. No `Failed to send goal response` server warning appeared. BT Navigator logged `Goal succeeded` in every trial.

However, the hardened runner wrote terminal JSON with `goal_response_received=false`, `goal_accepted=false`, transitioned `GOAL_REQUEST_SENT -> ABORTED`, and exited 2 in every trial. This indicates the new callback-based handling did not process/store the completed send-goal response before the main loop evaluated `_goal_handle`.

Per the task stop condition, no further runner modification or trial rerun was performed.

## Fake-base shutdown

The shutdown correction succeeded in these trials: `phase2_fake_base` exited cleanly with no context-invalid traceback in all three runs.

## Action counts

- Current-pose NavigateToPose requests in this task: 3
- Authoritative 3 m NavigateToPose goals: 0
- Direct FollowPath actions: 0

## Next recommendation

Correct the runner response handling by synchronously reading the completed send-goal future after the explicit executor wait, while retaining callback timestamps as auxiliary evidence, or by making the callback-processing path authoritative and waiting for `_goal_response_received` rather than only `future.done()`. Then rerun the same three current-pose response trials before retrying the 3 m goal.

Do not change Candidate C, Nav2 parameters, map, scenario, progress checker, goal checker, fake-base kinematics, or command topics.

## Integrity

- No commit, push, amend, reset, clean, or tag was performed.
- Candidate C controller parameters were not changed during this task.
- Map and scenario files were not changed during this task.
- No hardware, sensor, CAN, UDP, or physical command access occurred.
- P2-E/P2-F/P2-G and Phase 3 were not started.
