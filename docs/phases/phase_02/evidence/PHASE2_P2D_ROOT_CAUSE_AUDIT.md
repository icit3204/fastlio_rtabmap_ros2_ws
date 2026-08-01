# Phase 2 P2-D Kinematic Consistency and Controller Root-Cause Audit

Final status: `PHASE2_P2D_ROOT_CAUSE_AUDIT_COMPLETE`

Primary disposition: `MPPI_CONTROLLER_PROFILE_OR_FEEDBACK_DEFECT`

Confidence: medium-high. Fake-base kinematics passed in isolation. The controller-driven system still curved away and aborted on a direct straight +X FollowPath action, which bypasses NavFn planning output and NavigateToPose BT/replanning. The direct FollowPath run had a lifecycle-manager map-server timeout in its launch log, so the result is not a clean runtime validation pass; it is still sufficient controller/fake-base separation evidence for a root-cause audit.

## Scope and integrity

- Repository source/configuration was not modified by this audit.
- Existing dirty P2-D runner files were preserved.
- No commit, tag, or push was performed.
- No NavigateToPose goal was sent.
- Runtime diagnostics used `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=195`.
- No hardware, sensor, CAN, UDP, legacy command topic, or real controller path was used.

## Dirty-state preservation

Fresh backup existed before runtime work:

- `/home/dog/phase2_reports/PHASE2_P2D_UNCOMMITTED_RUNNER_20260801_053000.patch`
- `/home/dog/phase2_reports/PHASE2_P2D_UNCOMMITTED_STATE_20260801_053000.txt`

Only the expected dirty/untracked P2-D runner files were present.

## Fake-base source audit

Source files:

- `src/parking_robot_bringup/parking_robot_bringup/phase2_fake_base_math.py`
- `src/parking_robot_bringup/parking_robot_bringup/phase2_fake_base.py`

Findings:

- `Twist.linear.x` is interpreted as body-frame forward velocity `v`.
- `Twist.angular.z` is interpreted as yaw rate `w`.
- Straight integration uses `x += v*cos(yaw)*dt`, `y += v*sin(yaw)*dt`.
- Curved integration uses the exact differential-drive arc form with radius `v/w`.
- Positive `w` increases yaw and produces positive Y displacement for a forward-left curve from yaw 0.
- Yaw is normalized to `[-pi, pi)`.
- Odometry pose and `odom -> base_footprint` TF are built from the same pose and normalized quaternion.
- Odometry twist reports the active filtered command as `linear.x` and `angular.z`.
- `/initialpose` resets pose, clears active velocity, clears command freshness, and resets integration time.
- Command staleness uses steady/monotonic time and `cmd_timeout_sec=0.5`.
- Integration timestep is clamped by `max_integration_dt_sec=0.1`.

The pure unit tests verify internal math consistency. The runtime open-loop test below additionally verified ROS/Nav2 sign conventions on published odometry and TF.

## Controller feedback and parameter audit

Effective source configuration:

- Controller odom topic: `/Odometry`
- `min_x_velocity_threshold`: `0.001`
- `min_y_velocity_threshold`: `0.001`
- `min_theta_velocity_threshold`: `0.001`
- `controller_frequency`: `20.0`
- `failure_tolerance`: `0.3`
- Progress checker: `nav2_controller::SimpleProgressChecker`, `required_movement_radius=0.10`, `movement_time_allowance=20.0`
- Goal checker: `nav2_controller::SimpleGoalChecker`, `xy_goal_tolerance=0.25`, `yaw_goal_tolerance=0.50`, `stateful=true`
- MPPI: `DiffDrive`, `vx_min=0.0`, `vx_max=0.25`, `vy_max=0.0`, `wz_max=0.8`, `time_steps=40`, `model_dt=0.05`, `batch_size=500`, `iteration_count=1`, `prune_distance=1.0`, `transform_tolerance=0.5`
- Costmaps: global frame `map`, local frame `odom`, base frame `base_footprint`, static+inflation layers only, no obstacle/voxel observations.

The fake base publishes `/Odometry` and `odom -> base_footprint` consistently. The open-loop test verified that odometry twist signs match the commanded signs.

## Forward failure reconstruction

Forward goal: `(8.425, -53.725, yaw=0.0)` from start `(5.425, -53.725, yaw=0.0)`.

Key runtime facts from existing forward diagnostic:

- Goal was accepted once.
- Path was valid, finite, in frame `map`, 59 poses, length `1.508668738828552` m, with zero occupied/unknown/out-of-bounds hits.
- Commands were finite and within configured limits.
- Robot moved initially along positive X, then increasingly departed negative Y while yaw drifted negative.
- Final pose: `{'x': 7.130008824540141, 'y': -54.479052371807235, 'yaw': -2.3920425480049348}`
- Final position error: `1.4985316559709452` m.
- Progress failures: 3.

The forward evidence contains odometry buckets but does not contain synchronized command buckets. Therefore the audit cannot prove from the prior NavigateToPose evidence alone whether each angular command sample reduced or increased instantaneous heading error. The odometry trend is still clear: heading error grows after the robot departs the path, and final yaw is strongly negative on a goal that required yaw 0.

See `PHASE2_P2D_CONTROL_FEEDBACK_TIMELINE.tsv` for the bucketed reconstruction.

## Open-loop fake-base diagnostic

Decision: `OPEN_LOOP_PASS`

All four commanded cases passed:

- Case A: +0.10 m/s straight for 2 s -> +0.200003 m X, yaw 0.
- Case B: +0.20 rad/s for 2 s -> +0.400010 rad yaw.
- Case C: -0.20 rad/s for 2 s -> -0.399995 rad yaw.
- Case D: +0.10 m/s, +0.20 rad/s for 2 s -> x 0.194728 m, y 0.039478 m, yaw 0.400042 rad.

TF and odometry matched within the requested tolerance. This rejects `FAKE_BASE_KINEMATIC_OR_INTERFACE_DEFECT` as the primary root cause.

## Direct FollowPath diagnostic

Status: `DIRECT_FOLLOW_PATH_FAIL_WITH_HEADING_DIVERGENCE`

One direct `/follow_path` goal was sent with a straight +X path from `(5.425, -53.725)` to `(7.425, -53.725)`. No NavigateToPose goal was sent.

Results:

- Goal accepted: `True`
- Result received: `True`
- Action status: `6` (`6` = aborted)
- Command messages: `1059`
- Nonzero commands: `1058`
- Max abs linear.x: `0.06738843768835068`
- Max abs angular.z: `0.0280827134847641`
- Final pose: `{'child_frame_id': 'base_footprint', 'distance_to_end': 0.758011187300329, 'frame_id': 'odom', 'goal_bearing': 0.715821324295152, 'heading_error': 1.9140960322053537, 't': 48149.777489331, 'vx': 0.0, 'wz': 0.0, 'x': 6.853039238418097, 'y': -54.22243526944026, 'yaw': -1.1982747079102023}`
- Final position error: `0.758011187300329` m

Interpretation: this is not a command transport or fake-base no-motion failure. The controller generated finite commands and the fake base moved, but the path-following behavior curved away and aborted before reaching the configured goal tolerance.

Limitation: lifecycle manager logged a map-server transition timeout during the direct diagnostic launch. Because the action still accepted and generated commands, the run is usable as behavior evidence but not as a clean lifecycle-validation run.

## MPPI critic inspection

Installed package version: `ros-humble-nav2-mppi-controller 1.1.20-1jammy.20260607.135947`.

| Critic | Installed plugin available | Loaded by current profile | Current explicit params | Observed/default evidence | Consequence |
|---|---:|---:|---|---|---|
| GoalCritic | yes | yes | none per-critic | runtime log: power 1, weight 5.0 | attracts trajectories toward goal |
| GoalAngleCritic | yes | yes | none per-critic | runtime log: power 1, weight 3.0, threshold 0.5 | enforces final yaw near goal |
| PathAlignCritic | yes | yes | none per-critic | runtime log: power 1, weight 10.0 | aligns trajectories to path, but current behavior still curved away |
| PathFollowCritic | yes | yes | none per-critic | loaded; no detailed startup line observed | encourages forward progress along path |
| ObstaclesCritic | yes | yes | none per-critic | runtime log: power 1, repulsion 20.0, critical 1.5, circular cost | obstacle/cost avoidance only; path was in free space |
| ConstraintCritic | yes | no | absent | plugin listed in installed `critics.xml` | absent from current profile; no explicit dynamic constraint critic shaping |
| PreferForwardCritic | yes | no | absent | plugin listed in installed `critics.xml` | absent; current profile relies on `vx_min: 0.0` but has no explicit forward-preference critic |
| PathAngleCritic | yes | no | absent | plugin listed in installed `critics.xml` | absent; no explicit critic enforcing heading along path direction |
| TwirlingCritic | yes | no | absent | plugin listed in installed `critics.xml` | absent; mostly relevant to omni/twirling behavior |


The current profile appears minimal and underconstrained for heading/path-following behavior: it omits `PathAngleCritic`, `PreferForwardCritic`, and `ConstraintCritic` even though they are installed. No parameter was changed in this audit.

## Root-cause disposition

Selected primary disposition: `MPPI_CONTROLLER_PROFILE_OR_FEEDBACK_DEFECT`.

Direct evidence:

1. Fake-base open-loop kinematics and ROS interface signs pass.
2. Forward NavigateToPose failed even with a forward-aligned goal, falsifying the “reverse heading only” hypothesis.
3. Direct FollowPath, bypassing NavFn output and BT NavigateToPose orchestration, still produced a negative-y/yaw curve and aborted.
4. Controller commands were finite and isolated to `/cmd_vel_phase2_mock`; no legacy or hardware command path was involved.

Contradictory/limiting evidence:

- The direct FollowPath launch had a lifecycle-manager map-server timeout, so a future clean direct controller run should be repeated after fixing/avoiding that launch timing issue.
- The existing runner did not preserve synchronized command buckets, limiting exact angular-command-vs-heading-error proof from prior NavigateToPose evidence.

Smallest proposed corrective action for a future task:

- Keep fake-base source unchanged initially.
- Keep command authority, topics, map, TF, costmaps, goal checker, and progress checker unchanged initially.
- Make the smallest MPPI profile correction consistent with Humble 1.1.20, likely adding/setting path-heading and forward/constraint critics (`PathAngleCritic`, `PreferForwardCritic`, `ConstraintCritic`) or explicitly setting per-critic weights/offsets after inspecting upstream defaults.
- Validate with one clean direct FollowPath test before another NavigateToPose P2-D retest.

Files likely involved in a future remediation:

- `src/parking_robot_bringup/config/phase2_nav2_params.yaml`
- tests under `src/parking_robot_bringup/test/` to assert intended MPPI critic profile.

Files/behavior that should remain unchanged initially:

- fake-base integration source;
- `/cmd_vel_phase2_mock`, `/Odometry`, and TF ownership;
- map and costmap static/inflation structure;
- progress checker and goal checker, until controller profile is isolated.

## Closure

This audit satisfies the requested source-level, open-loop, direct-controller, and evidence-based classification work without tuning or remediation.
