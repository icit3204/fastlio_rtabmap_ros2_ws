# Phase 2 — Core Simple Nav2 Navigation Baseline

Date: 2026-08-02
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Current status

Runtime status: `IN_PROGRESS`

Completed scope:

- P2-A isolated package/configuration scaffolding: COMPLETE.
- P2-B static validation and package build/install validation: COMPLETE.
- P2-C isolated no-goal runtime: COMPLETE after remediation.
- P2-D bounded one-goal fake closed-loop NavigateToPose runtime: COMPLETE.
- P2-E sequential goals and active-goal cancellation: COMPLETE.

Remaining Phase 2 stages:

- P2-F: NOT_STARTED.
- P2-G: NOT_STARTED.

Phase 2 overall remains `IN_PROGRESS`.

## Implemented package

`src/parking_robot_bringup/`

The package is isolated from legacy `robot_bringup` runtime launch paths and does not include or activate semantic grid tools, Collision Monitor, Generic Command Safety Gate, Wheelchair Command Adapter, wheelchair controller/CAN, plan_nav, Pure Pursuit, Livox/FAST-LIO/RTAB-Map live localization, camera/YDLIDAR/ultrasonic drivers, keepout/speed filters, Gazebo, or Phase 3 Mission Manager.

## Runtime chain validated for P2-D

```text
NavigateToPose
  -> BT Navigator
  -> NavFn GridBased planner
  -> MPPI FollowPath controller with Candidate C
  -> /cmd_vel_phase2_mock
  -> phase2_fake_base
  -> /Odometry
  -> odom -> base_footprint

static_transform_publisher
  -> map -> odom identity
```

No physical or real command interface was involved.

## Map and scenario

Packaged isolated test asset:

- `src/parking_robot_bringup/maps/phase2_clean_map.yaml`
- `src/parking_robot_bringup/maps/phase2_clean_map.pgm`

Authoritative P2-D scenario:

- scenario name: `p2d_forward_goal`
- start: `(5.425, -53.725, 0.0)`
- goal: `(8.425, -53.725, 0.0)`
- frame: `map`
- nominal distance: 3.0 m

The scenario is tracked and covered by static map tests.

Authoritative P2-E scenarios:

- sequential scenario: `p2e_sequential_forward`
- cancellation scenario: `p2e_cancel_forward`
- start: `(5.425, -53.725, 0.0)`
- sequential goals: `(6.425, -53.725, 0.0)`, `(7.425, -53.725, 0.0)`, `(8.425, -53.725, 0.0)`
- cancellation goal: `(8.425, -53.725, 0.0)`
- cancellation request timing: 3.0 seconds after goal acceptance
- frame: `map`

The P2-E scenarios use the same validated straight corridor as P2-D and are covered by static map tests.

## P2-C status

The first P2-C no-goal runtime failed for two scoped reasons: BT Navigator default BT plugin coverage and fake-base shutdown handling. Remediation was completed, and the passing retest is the authoritative P2-C runtime evidence.

## P2-D diagnostic and correction history

- The original P2-D controller profile failed to complete the initial one-goal attempt.
- Fake-base open-loop kinematics passed.
- Lifecycle startup race was resolved operationally through explicit delayed lifecycle STARTUP.
- PathAngleCritic corrected severe path-heading divergence in direct controller testing.
- `regenerate_noises=true` removed deterministic left/right sampling bias.
- Candidate C passed the direct simple straight FollowPath test.
- Candidate E (`time_steps=80`) was unnecessary for the straight case and did not materially improve curved-path behavior.
- NavigateToPose runner action-state and Future handling were hardened.
- The introduced callback-ordering race was corrected.

## Final authoritative P2-D result

The final authoritative P2-D run passed using the normal Nav2 pipeline:

```text
NavigateToPose -> BT Navigator -> NavFn -> MPPI Candidate C -> /cmd_vel_phase2_mock -> phase2_fake_base
```

Result summary:

- one NavigateToPose request sent;
- goal response received and accepted;
- NavFn produced a finite map-compatible path;
- MPPI produced finite commands within unchanged limits;
- fake base moved materially toward the goal;
- action result: `SUCCEEDED`;
- final XY error: 0.24780476515545505 m;
- final yaw error: 0.003909699763943841 rad;
- active command-stream frequency: approximately 20.024 Hz;
- command stop latency: 0.0 s;
- fake base exited cleanly;
- no legacy, physical, CAN, UDP, sensor, or real command path appeared.

P2-D status: `COMPLETE`.

## Final authoritative P2-E result

P2-E validated two bounded capabilities using the normal NavigateToPose pipeline and the unchanged Candidate C controller profile.

Sequential run:

- one initial pose was published;
- exactly three NavigateToPose requests were sent;
- goals completed strictly in order;
- no fake-base odometry reset occurred between goals;
- all three goals returned `SUCCEEDED`;
- final XY errors were 0.24965727929978862 m, 0.247894889511765 m, and 0.2488566383313176 m;
- command stop latency after every goal was 0.0 s;
- no command-limit violation occurred;
- no physical or legacy command path appeared;
- stack shutdown was clean.

Cancellation run:

- exactly one NavigateToPose request was sent;
- the goal was accepted and entered `GOAL_ACTIVE`;
- exactly one cancellation request was sent 3.002859725005692 s after acceptance;
- terminal action status was `5` (`CANCELED`);
- the runner acceptance gate verified `/cmd_vel_phase2_mock` and fake-base velocity stopped within the configured 0.5 s limit;
- translation after cancel request was 0.0 m;
- one-second post-stop observed motion was 0.0 m;
- no second goal was sent;
- no physical or legacy command path appeared;
- stack shutdown was clean.

P2-E status: `COMPLETE`.

## Limitations retained

- Mirrored curved fixed-path tracking remains unreliable.
- Candidate C passed only the bounded simple straight scenario.
- P2-E used one validated straight corridor; general curved sequential navigation remains unproven.
- No obstacle-avoidance acceptance occurred.
- No semantic perception integration occurred.
- No physical robot motion occurred.
- No CAN, UDP, or real command path was tested.
- No physical safety validation occurred.
- The exact historical isolated server-response timeout cause remains unproven.

Do not claim general navigation reliability, general curved-path reliability, physical navigation completion, obstacle avoidance completion, Phase 2 completion, or Phase 3 readiness from this evidence.

## Evidence

See `PHASE_02_EVIDENCE_INDEX.md`.
