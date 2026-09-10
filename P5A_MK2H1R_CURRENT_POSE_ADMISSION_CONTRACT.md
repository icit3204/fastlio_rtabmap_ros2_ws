# P5A MK2H1R Current-Pose Admission Contract

## Current semantics

The robot does **not** have to be physically located at the PlanNav Start
Node.

PlanNav includes every selected route node in `RouteMission.poses`. Mission
Manager initializes `current_waypoint_index` to zero and dispatches
`poses[0]`. Nav2 independently obtains the planner start pose from current TF.
Consequently:

- PlanNav Start Node = first metric NavigateToPose goal;
- current robot pose = planner start;
- no implicit topology edge or projection connects the robot to Start Node;
- Nav2's global planner provides the metric approach to that first goal.

The previous 16.608 m rejection was safe but was based on the wrong semantic
assumption that the robot must already occupy Start Node.

## Required future admission

A route may be admitted only when all of the following hold:

1. localization is valid and the robot pose is finite in frame `map`;
2. the selected topology reconciliation manifest names the exact canonical DB
   SHA-256;
3. RouteMission and every pose use frame `map`;
4. topology identity/version is current and all directed edges resolve;
5. robot pose and first goal lie within the canonical optimized graph/map
   envelope;
6. no coordinate scale or unrecorded transform is applied;
7. the normal Nav2 planner can find a path from the live robot pose to the
   first goal. Planner rejection remains the authoritative fail-closed result.

Nearest-node distance is recorded for operator awareness, not used as a hard
admission threshold. The reconciled topology's nearest node to the MK2H1 pose
is node 4 at 17.168 m. This is permitted by current software semantics, but it
means the first global plan must cover that approach.

## Unsupported alternatives

- Projecting the robot onto a topology edge is not implemented.
- Creating an automatic connector edge is not implemented.
- Reinterpreting Start Node as already completed is not implemented.
- Orientation is retained in RouteMission, although the current canonical
  Nav2 profile has a permissive 6.28 rad yaw goal tolerance.

## Conservative operator policy

For a demonstration, select the closest meaningful canonical Start Node and a
short directed route. Use node 4 as Start for the recorded MK2H1 pose, then
node 1 and optionally node 2. Observe the generated global plan before Gate
arming. A planner failure is a safe blocker, not justification for changing
coordinates or thresholds.
