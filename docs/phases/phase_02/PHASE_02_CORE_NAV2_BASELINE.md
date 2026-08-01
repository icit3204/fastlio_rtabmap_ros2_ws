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

Remaining Phase 2 stages:

- P2-E: NOT_STARTED.
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

## Limitations retained

- Mirrored curved fixed-path tracking remains unreliable.
- Candidate C passed only the bounded simple straight scenario.
- No obstacle-avoidance acceptance occurred.
- No semantic perception integration occurred.
- No physical robot motion occurred.
- No CAN, UDP, or real command path was tested.
- No physical safety validation occurred.
- The exact historical isolated server-response timeout cause remains unproven.

Do not claim general navigation reliability, general curved-path reliability, physical navigation completion, obstacle avoidance completion, Phase 2 completion, or Phase 3 readiness from this evidence.

## Evidence

See `PHASE_02_EVIDENCE_INDEX.md`.
