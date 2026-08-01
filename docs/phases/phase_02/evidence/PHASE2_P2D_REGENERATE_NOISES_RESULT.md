# Phase 2 P2-D regenerate_noises isolated audit

Generated: 2026-08-01T17:27:41

## Baseline and candidate

- Source baseline: `985cda0f20859fb6144362248e17b1fa42efd5e4`
- Candidate A SHA256: `1adc0f865fefb5f6d42e297ad59edc5e89056367870e37316336301729c42791`
- Candidate C SHA256: `6e206a332f0a0d5dd61ca8691325267014f5455d6a9ab238f716fcbd9aa0f842`
- Candidate C base: Candidate A
- Candidate C only additional semantic change: `controller_server.ros__parameters.FollowPath.regenerate_noises: true`
- Candidate B used: no
- NavigateToPose goals sent: 0
- FollowPath actions sent: 6
- Tracked source/configuration modified: no

## Installed semantics

The installed Humble MPPI package is `ros-humble-nav2-mppi-controller` version `1.1.20-1jammy.20260607.135947`. `regenerate_noises` is treated as a Boolean FollowPath MPPI optimizer parameter. This experiment used it only as an external overlay to test whether repeated noise regeneration removes the persistent left-turn finite-noise bias observed under Candidate A.

## Build/test validation

- Build root: `/home/dog/phase2_builds/p2d_regenerate_noises_20260801_170929`
- Package built: `parking_robot_bringup` only
- Tests: 43 passed, 0 failures, 0 errors
- Candidate C remained external/untracked.

## Candidate C six-trial result

| Trial | Side | Domain | Result | Median angular sign first 2s | Median angular sign first 10s | Expected sign % | Measured yaw change | Final endpoint error m |
|---:|---|---:|---|---:|---:|---:|---:|---:|
| 1 | LEFT | 221 | TIMEOUT | 1 | 1 | 73.570 | 1.568405 | 0.422045 |
| 2 | RIGHT | 222 | TIMEOUT | -1 | -1 | 75.902 | -1.573882 | 0.419815 |
| 3 | RIGHT | 223 | TIMEOUT | -1 | -1 | 75.736 | -1.573765 | 0.420480 |
| 4 | LEFT | 224 | TIMEOUT | 1 | 1 | 72.682 | 1.568526 | 0.422850 |
| 5 | LEFT | 225 | TIMEOUT | 1 | 1 | 73.182 | 1.568411 | 0.423603 |
| 6 | RIGHT | 226 | TIMEOUT | -1 | -1 | 75.458 | -1.573500 | 0.420271 |

## Candidate C aggregate

| Side | Successes | Timeouts | Aborts | Median final endpoint error m | Expected sign % range | Early 2s sign pattern |
|---|---:|---:|---:|---:|---|---|
| LEFT | 0/3 | 3 | 0 | 0.422850 | 72.681843..73.570239 | 1,1,1 |
| RIGHT | 0/3 | 3 | 0 | 0.420271 | 75.458079..75.902277 | -1,-1,-1 |

## Candidate A versus Candidate C

| Side | A successes | C successes | A timeouts | C timeouts | A median endpoint error | C median endpoint error | A early 2s signs | C early 2s signs |
|---|---:|---:|---:|---:|---:|---:|---|---|
| LEFT | 0 | 0 | 3 | 3 | 0.5530872835449276 | 0.4228496978639661 | -1,-1,-1 | 1,1,1 |
| RIGHT | 3 | 0 | 0 | 3 | 0.24900575423737795 | 0.42027129601277036 | -1,-1,-1 | -1,-1,-1 |

## Acceptance decision

`P2D_REGENERATE_NOISES_DIRECT_NEEDS_REVIEW`

Candidate C did not satisfy the pass gate unless all six trials succeeded, all left trials had positive first-two-second median angular sign, all right trials had negative first-two-second median angular sign, and every final endpoint error was <= 0.25 m.

## Classification

`REGENERATED_NOISES_IMPROVE_BUT_INSUFFICIENT`

Confidence: `MEDIUM`

Direct evidence:

- Candidate C kept Candidate A's PathAngleCritic values unchanged and changed only `regenerate_noises` to true.
- Left trials did not all switch to the required positive early angular sign.
- Right trials retained the expected negative early angular response where observed.
- No command-limit or invalid-command defect was observed in the trial metrics.

Contradictory/limiting evidence:

- This remains a six-run direct FollowPath diagnostic, not a general MPPI tuning proof.
- The installed binary package does not expose full optimizer source text under the share tree, so runtime behavior is the deciding evidence for this candidate.

## Exact next recommendation

Do not apply Candidate C. Do not combine it with other changes. The next isolated candidate should target sampling robustness or critic/path-angle behavior with one semantic change at a time, preserving progress checker, goal checker, fake base, costmaps, TF, map, command topics, velocity limits, and BT behavior. Validate with mirrored direct FollowPath repeat trials before any NavigateToPose retest.

## Integrity statement

- No tracked repository file was changed by this audit.
- Only the expected dirty Phase 2 runner files remain dirty.
- Candidate A remained external.
- Candidate B remained rejected and unused.
- Candidate C remained external.
- Exactly six FollowPath actions were sent: three left and three right.
- Zero NavigateToPose goals were sent.
- No hardware, sensor, CAN, or UDP access was performed.
- No system package was installed or removed.
- P2-E/P2-F/P2-G were not started.

Final audit status: `PHASE2_P2D_REGENERATE_NOISES_AUDIT_COMPLETE`
