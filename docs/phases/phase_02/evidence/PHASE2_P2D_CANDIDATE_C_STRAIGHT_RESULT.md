# Phase 2 P2-D Candidate C straight-path isolation result

Generated: 2026-08-01T18:39:27

## Scope

- Candidate C used unchanged.
- Candidate C SHA256: `6e206a332f0a0d5dd61ca8691325267014f5455d6a9ab238f716fcbd9aa0f842`
- ROS_DOMAIN_ID: 173
- FollowPath actions sent: 1
- NavigateToPose goals sent: 0
- Tracked source/configuration modified: no

## Result

Decision: `P2D_CANDIDATE_C_STRAIGHT_DIRECT_PASS`

Interpretation: `CANDIDATE_C_STRAIGHT_PASS_EXTENDED_HORIZON_NOT_REQUIRED`

## Metrics

| Metric | Value |
|---|---:|
| Lifecycle result | PASS |
| Action accepted | True |
| Action result | SUCCEEDED |
| Final endpoint error m | 0.24916319341470655 |
| Minimum endpoint error m | 0.24916319341470655 |
| Final yaw error rad | 1.9428797242149187e-06 |
| Max lateral deviation m | 0.03095644939785558 |
| Mean lateral deviation m | 0.015232903996402853 |
| Command count | 532 |
| Active command-stream frequency Hz | 20.042516981652664 |
| Controller action-window frequency Hz | 20.066397240558665 |
| Client-duration frequency Hz | 18.016210285418296 |
| Invalid commands | 0 |
| Command-limit violations | 0 |

## Candidate E timing correction

`PHASE2_P2D_EXTENDED_HORIZON_TIMING_ERRATUM.md` corrects the earlier Candidate E timing classification. Candidate E did not show controller-rate overrun based on controller action-window timestamps; its curved paths still failed outside tolerance.

## Interpretation

Candidate C already satisfies the isolated straight direct FollowPath gate if the pass decision above is `P2D_CANDIDATE_C_STRAIGHT_DIRECT_PASS`. The 80-step Candidate E horizon is therefore not required for this simple straight controller case.

## Integrity statement

- No tracked repository file was changed.
- Candidate C and Candidate E remained external.
- Exactly one FollowPath action was sent.
- Zero NavigateToPose goals were sent.
- No hardware, sensor, CAN, or UDP access occurred.
- P2-E/P2-F/P2-G were not started.

Final status basis: `P2D_CANDIDATE_C_STRAIGHT_DIRECT_PASS`
