# Phase 4 Final Engineering Closeout

## Executive summary

Phase 4 established and exercised the software-only command, safety, health,
progress, cancellation, and qualification architecture. The campaign is now
engineering-closed with one documented qualification gap. This is a closeout
boundary, not a declaration that every progress experiment qualified.

The final authoritative disposition is `P4E6C4G_PROGRESS_FAILURE_BLOCK_CLOSED_PARTIAL_NEEDS_SUPERVISOR_NEXT_PHASE_REVIEW`.
No further P4-E.6C live attempts are authorized by this closeout.

## Architecture at Phase-4 exit

The accepted architecture keeps Mission Manager as the owner of mission and
NavigateToPose identity, routes commands through the safety path, and uses the
Generic Gate and mock/qualification adapters as safety boundaries. The fake
base is software-only and consumes the accepted safe command path. Progress
observation remains source-derived and identity-bound; qualification tooling
does not modify production Mission Manager, Nav2, Gate, Collision Monitor, or
fake-base policy.

The accepted qualification path includes route/start/readiness sequencing,
persistent service clients, source-row readiness adjudication, branch-correct
command-pair interpretation, an odom-frame persistent pose clamp, and bounded
terminal/physical cleanup evidence. These are engineering and qualification
mechanisms; they do not authorize physical wheelchair or CAN output.

## Major implementation work completed

- Mission Manager gained the Phase-4 progress, health, identity, recovery,
  cancellation, and BLOCK evidence interfaces used by the accepted tests.
- The vehicle command safety package gained validity/currentness supervision
  and bounded service-query handling.
- Bringup gained the fake base, accepted launch/configuration paths, route and
  readiness qualification tooling, persistent pose-clamp support, and evidence
  runners/observers.
- The Nav2 progress-checker and reconciled lifecycle package provide the
  phase-boundary integration points exercised by the non-effectful tests.
- The mock wheelchair adapter exposes qualification-only health control while
  preserving the no-physical-authority boundary.

## Important architectural corrections

The campaign corrected endpoint/readiness false negatives, Humble service
response interpretation, fixed-client lifetime, pre-arm safe-command timing,
source-row active-readiness adjudication, established-PENDING command-pair
semantics, persistent odom-frame clamp cadence, stepwise recovery evidence,
and terminal-drain ordering. The C4G record shows that the remaining gap was
qualification evidence/runner completeness, not a newly accepted production
failure.

## Qualification summary

- P4-E.6B health-failure block: `QUALIFIED_PASS`.
- C-P01: `QUALIFIED_PASS_RETAINED_EVIDENCE_RECONCILIATION`.
- C-P02: `NOT_QUALIFIED`.
- P4-E.6C progress-failure block: `NOT_FULLY_QUALIFIED`.

C-P02 Attempt 1 observed the production
`RECOVERY_EXHAUSTED_NO_PROGRESS` branch, but its same-attempt stepwise,
clamp, BLOCK/status, and physical evidence was incomplete. Attempts 2 and 3
were consumed qualification-tool failures: insufficient clamp publications
and terminal-drain timeout without a product terminal. No Attempt 4 is
authorized.

## Accepted / closed bucket

- C-P01 production no-progress behavior, BLOCK/cancel chain, and physical
  closure as reconciled from retained evidence.
- Route/start/Gate/readiness lifecycle and accepted service-client path.
- Persistent odom-frame 10-Hz pose-clamp mechanism.
- P4-E.6B health-failure qualification.
- Observation of the production C-P02 recovery-exhaustion branch in Attempt 1.

## Deferred robustness / qualification bucket

- Complete same-attempt C-P02 source-ACKed recovery sequence 1 through 6.
- Complete same-attempt C-P02 clamp, BLOCK/status-3/status-5, and post-terminal
  physical closure evidence.
- Qualification runner behavior that entered terminal drain without a product
  terminal.

## Later integration bucket

Real-wheelchair/chassis and CAN integration, plus any project-level end-to-end
integration outside this closed software qualification campaign, belong to
later architecture phases.

## Do-not-reopen decisions

C-P01 Attempt 3 and C-P02 Attempt 4 are permanently not authorized under this
campaign. P4-E.6C is not automatically reopened, and no automatic retry is
created by this document.

## Safety boundary at exit

Real CAN and physical wheelchair authority are not part of the accepted
Phase-4 state. The accepted evidence is software-only and uses mock/fake-base
boundaries where applicable.

## Test/build evidence

The current sourced ROS 2 Humble environment imported the required ROS and
project interfaces. The current non-effectful selected suite completed with
375 passed, 0 failed, 0 errors, and 0 skipped. `git diff --check` passed.
The affected build result and exact commands are recorded in the supervisor
audit evidence; no live case, Gate arm, route publication, pose clamp, or
recovery injection was run by this audit.

## Known risks and limitations

The worktree contains a mixed, largely uncommitted Phase-4 engineering and
qualification tree alongside unrelated/pre-existing files. It is not safe to
silently stage the entire tree. The C-P02 qualification gap remains real and
must not be represented as product failure or product pass.

## Phase-5 entry prerequisites

Phase 5 may rely on the accepted architecture and the closed C4G disposition,
but must read this document and the machine-readable state first. It must
preserve the C-P02 boundary, decide separately whether any deferred
qualification is needed, and define its own architecture authority and
integration evidence. It must not infer a future closeout commit or remote
SHA until a supervisor-authorized Git boundary exists.

## Git checkpoint

`PHASE4_CLOSEOUT_COMMIT_SHA = PENDING_SUPERVISOR_COMMIT`

`PHASE4_CLOSEOUT_REMOTE_SHA = PENDING_SUPERVISOR_PUSH`

`PHASE4_CLOSEOUT_TAG = PENDING_SUPERVISOR_DECISION`

## Repository snapshot contents

The final file-level manifest is
PHASE4_FINAL_COMMIT_MANIFEST.json. It includes the current Phase-4
production/integration source, configuration and launch files, interfaces,
tests, qualification tooling, the architecture authority, and the closeout
documentation. C4D/C4E/C4F and related C-P02 tools are retained as
PHASE4_QUALIFICATION_TOOLING_KNOWN_GAP; their presence records engineering
history and does not change the C-P02 result.

Generated runtime evidence, B2* status/report files, phase4 report archives,
build/install/log/cache output, large maps/databases/point clouds/models,
temporary text, and unrelated scratch files are excluded. The prior
PHASE4_PROPOSED_COMMIT_MANIFEST.json is superseded by the final manifest and
is excluded from the future engineering commit.
