# Mission Manager state machine

| State | Meaning and exit |
|---|---|
| IDLE | No accepted route. |
| RECEIVED | Typed route stored; start has not been requested. |
| VALIDATING | Contract and topology identity are checked. |
| PLANNING | Current waypoint is about to be sent to NavigateToPose. |
| NAVIGATING | Exactly one accepted NavigateToPose goal is owned. |
| PAUSED | The active goal reached terminal CANCELED after an acknowledged pause cancellation; index and goal are preserved. |
| CANCELLING | One bounded cancel transaction is pending. |
| CANCELLED | User stop/cancel completed; no next goal may dispatch. |
| TEMPORARILY_BLOCKED | Progress policy observed a temporary obstacle; current Nav2 goal remains authoritative. |
| BLOCKED / HELP_REQUIRED | Bounded block policy exhausted. |
| SUCCEEDED | Every waypoint succeeded exactly once. |
| FAILED | Rejection, abort, unexpected cancel, health failure, or cancellation protocol failure. |

Pause cancels the active NavigateToPose goal and reaches PAUSED only after Nav2 acknowledges cancellation and returns CANCELED. Resume re-sends the preserved current waypoint. Stop/cancel overrides lesser pending intents, cancels once, and never advances the route. Gate safety interruption is independent mission evidence: it can block or fail a mission, but it can never mark a waypoint complete.

`INITIAL_PRIMING`, `PAIR_NOT_YET_ESTABLISHED`, `PAIR_ESTABLISHED`, and runtime
pair health are internal progress-observer phases, not public mission states.
The first-pair phase is bounded by 2.0 s and tolerates only causal ambiguity
between otherwise fresh epoch-local command streams. Once established, later
pair staleness is strict. Explicit safety/validity faults remain immediate in
all observer phases.

## Goal ownership and cancel contract

| Reported state | Accepted goal owned | Result future | User cancel | Expected result |
|---|---:|---:|---:|---|
| RECEIVED / PLANNING | No | No or pending send | Yes | CANCELLED without dispatch, or bounded cancellation if dispatch wins |
| NAVIGATING | Yes | Active | Yes | CANCELLING, then CANCELLED after acknowledged terminal CANCELED |
| TEMPORARILY_BLOCKED | Yes | Active | Yes | User cancel overrides the lower-priority block intent |
| CANCELLING | Yes | Active | Yes | Existing operation becomes/remains user cancel; no duplicate request |
| PAUSED | No | Terminal | Yes | CANCELLED; preserved waypoint is not dispatched |
| FAILED / SUCCEEDED / BLOCKED | No | Terminal | No | Service rejects because mission authority is terminal |
| CANCELLED | No | Terminal | Idempotent yes | Remains CANCELLED |

An unexpected terminal ABORTED or CANCELED result clears
`active_goal_uuid` before publishing FAILED. Consequently, no persistent
user-visible state may claim active goal ownership after Nav2 has returned a
terminal result.
