# Phase 2 P2-D NavigateToPose Action Response Root-Cause Audit

Generated: 2026-08-01T19:25:32.172892

## Decision

Primary classification: `ROOT_CAUSE_NOT_PROVEN`

Final audit status basis: action-response path was probed exactly once with a current-pose NavigateToPose request; the probe succeeded, but the exact cause of the prior runner-specific server response timeout is not proven without modifying/retrying the runner.

## Preserved dirty state

Complete current-state preservation bundle timestamp: `20260801_191801`

- `/home/dog/phase2_reports/PHASE2_P2D_CURRENT_DIRTY_STATE_20260801_191801.txt`
- `/home/dog/phase2_reports/PHASE2_P2D_CURRENT_TRACKED_DIFF_20260801_191801.patch`
- `/home/dog/phase2_reports/PHASE2_P2D_CURRENT_UNTRACKED_FILES_20260801_191801.tar.gz`
- `/home/dog/phase2_reports/PHASE2_P2D_CURRENT_DIRTY_FILE_MANIFEST_20260801_191801.tsv`

## Baseline integrity

- HEAD: `985cda0f20859fb6144362248e17b1fa42efd5e4`
- origin/main: `985cda0f20859fb6144362248e17b1fa42efd5e4`
- Branch: `main`
- Staged files: `none`
- Candidate C semantic diff before runtime: `MATCH` per `/home/dog/phase2_reports/PHASE2_P2D_TRACKED_CANDIDATE_C_SEMANTIC_DIFF.md`.

## Main evidence

1. The tracked runner stores its ActionClient as an instance attribute, so a simple local ActionClient lifetime defect is not supported.
2. The runner stores send-goal and result futures only as local variables, but they remain in scope while the corresponding wait loops execute. A future lifetime defect is possible only as a robustness concern, not directly proven.
3. The runner transitions to `GOAL_ACTIVE` before sending the request and before goal acceptance. This is a real state-machine defect: request-pending and accepted-active are conflated.
4. Historical P2-D evidence shows the same runner architecture previously received a NavigateToPose goal response (`action_goal_accepted=true`) and then timed out during navigation result handling.
5. No-action endpoint audit showed one `/navigate_to_pose` action server under `/bt_navigator` and no duplicate BT Navigator action server.
6. Independent current-pose client, with explicit retained ActionClient/Future/GoalHandle and explicit executor, received `accepted=true` and `SUCCEEDED` in the same installed Candidate C stack.
7. Installed rcl/rcl_action contract says the observed warning is produced when a response reader is not ready/reachable at send time.

## Contradictory evidence against candidate causes

- Against permanent BT Navigator defect: independent probe succeeded.
- Against permanent FastDDS/RMW response transport defect: independent probe succeeded in the same domain/stack.
- Against duplicate/stale endpoint interference: endpoint audit found one intended NavigateToPose server.
- Against simple ActionClient object lifetime defect: runner keeps `self._action_client`.
- Against deterministic runner incapability: earlier P2-D runner evidence received a valid goal response.

## Most likely unresolved failure mode

The prior failure is most consistent with an intermittent action response reader/readiness timing interaction involving the runner's current action-client/executor/state-machine implementation. The exact material cause cannot be proven from one successful independent probe because the probe intentionally differed in multiple ways:

- explicit executor;
- retained goal-response future as an instance attribute;
- retained goal handle/result future as instance attributes;
- explicit done-callback logging;
- current-pose goal rather than the 3 m goal.

## Exact correction recommendation

Do not tune Candidate C or Nav2. The smallest runner-side correction to test next is in:

`src/parking_robot_bringup/parking_robot_bringup/phase2_goal_test_runner.py`

Recommended semantic changes:

1. Split action states: `GOAL_REQUEST_SENT` or equivalent before response, then `GOAL_ACTIVE` only after `ClientGoalHandle.accepted` is true.
2. Retain `send_goal_future`, `goal_handle`, and `result_future` as instance attributes until cleanup.
3. Register explicit goal-response and result callbacks that record timestamps and exceptions.
4. Use one explicit executor consistently in `main()`/runner execution instead of repeated implicit `rclpy.spin_once(self, ...)`, or prove the implicit executor path is equivalent.
5. Supply and record a goal UUID when the installed rclpy API supports it; otherwise record that UUID injection is unavailable.
6. Always write terminal JSON on goal-response timeout/failure.

Required tests:

- Pure state-machine test proving request-sent is not marked accepted/active.
- Pure lifetime test proving futures/goal handle are retained until cleanup.
- Callback-path test proving response/result callbacks update state once.
- Integration test: one current-pose NavigateToPose response probe after the runner correction.
- Then retry the authoritative 3 m NavigateToPose test once, unchanged Candidate C profile.

Parameters/files that must remain unchanged:

- Candidate C controller profile.
- Nav2 planner, progress checker, goal checker, BT, costmaps, fake-base kinematics, TF, command topics, map, and scenarios except runner-only evidence additions.

## Shutdown note

The stack exited with no residual process, but `phase2_fake_base` logged a context-invalid traceback and exit code 1 during SIGINT. This is recorded in `PHASE2_P2D_ACTION_RESPONSE_AUDIT_RUNTIME_LOG.txt`. It occurred after the probe result and did not affect response-path interpretation, but it is an integrity item to resolve before accepting a future authoritative runtime.

## Integrity

- Repository source/configuration was not edited during this audit.
- Candidate C remains tracked but uncommitted.
- No Candidate B, D, or E setting was applied.
- NavigateToPose requests sent in this audit: `1`.
- Direct FollowPath actions sent in this audit: `0`.
- No hardware, sensor, CAN, UDP, legacy command path, package install, commit, push, or tag operation was performed.
- P2-E/P2-F/P2-G and Phase 3 were not started.
