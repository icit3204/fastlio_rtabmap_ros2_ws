# Phase 3 Mission And Plan Nav Closure

## Status

Phase 3 is closed for the fake/mock Nav2 mission-integration scope.

Closure commit:

- Baseline before closure: `92bd226dd334842b8ed9fc225b24ca74c393e1f6`
- Closure documentation commit: recorded in the external handoff after push

Phase 3 does not start Phase 4 and does not validate physical navigation, sensors, CAN, wheelchair command topics, application UDP, Collision Monitor, Generic Command Safety Gate, or VehicleState.

## Implemented Commits

- P3-A typed mission core: `9c705a4b85194f030a2fb3af14167a2ede8467f7`
- P3-A.1 authority alignment: `d4026e9d3c3051a5cbc5124258cd4d0b9c4a0d94`
- P3-A.2 runtime hardening: `94d07539d0516fa93a266a3674d105788f6e4729`
- P3-B.1 topology identity: `85dda1631532d231572d15b13a250e3a2da8af29`
- P3-B.2 typed route integration: `92bd226dd334842b8ed9fc225b24ca74c393e1f6`

## Frozen Interfaces

- `RouteMission.msg`: `7524239b6aaf17809a63478a1608edf58759d04d1b3ba444641beedd34d8aea4`
- `MissionState.msg`: `e2b7bf495ad7d92f9695f8b7905af0a4eae2773455afb446bbfd0e8855a4fd90`

The accepted public mission API remains:

- `/mission/route`
- `/mission/state`
- `/mission/start`
- `/mission/cancel`
- `/mission/pause`
- `/mission/status`
- `/mission/block_reason`

## Topology Identity

The authoritative P3-B.1 topology assets remained unchanged during P3-C final B/C validation.

- Topology manifest: `ade6a0720afdc1e37d8a6a511b7f6cc09d8a24fa870fefdd8b44ef8c7211ae42`
- Authoritative edges: `cddda82ec09f28179b7bdd41fc9cdc3c84b8ea8e81ce67e556723177c59ce81c`

P3-C used an external temporary three-node fixture because the authoritative plan_nav topology is not the short Phase 2 corridor scenario. The fixture was generated through the committed topology, RouteSpec, and MissionBridge code rather than by manually constructing `RouteMission` arrays.

Fixture route:

- Nodes: `1 -> 2 -> 3`
- Poses: `(6.425, -53.725)`, `(7.425, -53.725)`, `(8.425, -53.725)`
- Edge IDs: `edge-p3c-000001`, `edge-p3c-000002`
- Edge directions: `+1`, `+1`
- Route ID: `route-sha256:3e92b044e133d8016c42786a9f784af23d40bcd4024f023a0e672be15518b8c1`
- Temporary topology version: `sha256:1e2d3f1085be690fb98e3550628fe12d867a17469de072400a7d63cc0cebe9ae`

## Runtime Campaigns

Three earlier P3-C campaigns are preserved as non-closure evidence.

| Campaign | Status | Classification | Raw archive SHA256 |
| --- | --- | --- | --- |
| 1 | `PHASE3_P3C_NEEDS_REVIEW` | `POST_START_EVIDENCE_MONITOR_DETAIL_GAP` | `18e70024b8e15ddf76284d591f91c677c816607fd707527464329ffadb27099a` |
| 2 | `PHASE3_P3C_RESTARTED_NEEDS_REVIEW` | `SCENARIO_A_POST_START_ZERO_AFTER_TERMINAL_EVIDENCE_FAILURE` | `1d0a1dfc11d6aa86acb02363c7541513028875549839f89bc35fcd735ca6deef` |
| 3 / V2 | `PHASE3_P3C_RESTARTED_V2_NEEDS_REVIEW` | `SCENARIO_A_POST_TERMINAL_ODOMETRY_WINDOW_MISSING_AFTER_V2_WAIT` | `1b4e3cca41cf12553074c6c669f49ce8ef242443a56eddc4828d053d3b6d2494` |

V2 Scenario A is accepted cumulatively only for the actual sequential success and corrected command-stop contract. It is not used as measured post-terminal odometry evidence because the V2 custom monitor stopped its odometry and TF timeline at mission terminal.

The final B/C campaign repaired the lifecycle by separating:

- `mission_terminal_seen`
- `observation_window_complete`
- `scenario_shutdown_requested`

The repaired heartbeat confirmed the executor, subscriptions, rosbag, and fake base stayed alive throughout each post-terminal observation window.

## Accepted Scenario A

Scenario A was not rerun in the final B/C campaign.

Cumulative accepted evidence:

- Typed `RouteMission` reached actual Nav2 through the Mission Manager.
- Three ordered `NavigateToPose` goals were sent.
- Three unique goal UUIDs succeeded.
- `MissionState` reached `SUCCEEDED`.
- Completed count was `3`.
- Progress was `1.0`.
- Command frequency was `19.989869 Hz`.
- Final zero Twist preceded action terminal by `0.033032 s`.
- Final zero Twist preceded `MissionState SUCCEEDED` by `0.044670 s`.
- No later nonzero command was observed.
- Corrected stop contract passed.

## Scenario B: Cancel

Scenario B ran on ROS domain `213`.

- Mission ID: `p3c-restarted-b-31c0e2b4`
- Goal UUID: `3ef735f762754853bf5013bf076b8a80`
- State sequence: `RECEIVED -> VALIDATING -> PLANNING -> NAVIGATING -> CANCELLING -> CANCELLED`
- One cancel service request was accepted for processing.
- The active Nav2 action reached `CANCELED`.
- No second or third waypoint goal was dispatched.
- Current waypoint index remained `0`.

Stop and motion metrics:

- Cancel request: `94851748745032`
- Mission `CANCELLED`: `94851786685449`
- Action `CANCELED`: `94851804621991`
- Final zero command: `94851807781042`
- Final zero after cancel request: `0.059036010 s`
- Final zero after action terminal: `0.003159051 s`
- Final zero after mission terminal: `0.021095593 s`
- Nonzero commands after final zero: `0`
- Command frequency: `20.534885783119638 Hz`
- Odometry samples after terminal: `130`
- TF samples after terminal: `130`
- Post-CANCELLED translation: `0.0015128811058040669 m`
- Post-CANCELLED yaw change: `0.000411873853188194 rad`

Heartbeat evidence showed rosbag, fake base, subscriptions, and executor alive through the 2.5 second post-CANCELLED observation window.

## Scenario C: Pause And Resume

Scenario C ran on ROS domain `214`.

- Mission ID: `p3c-restarted-c-6f0c76ed`
- First goal UUID before pause: `c3ced487300e4e02b56528a951fa1574`
- Resumed current-waypoint UUID: `283dc14ab42c4e4b9ff0e4d22bdd7f63`
- Later waypoint UUIDs: `c399d526dd4848e7a40b733fa41c5716`, `967d3cd2c04046599927ffc1cf5cbe41`
- State sequence included `PAUSED`, resumed through `PLANNING`, and ended at `SUCCEEDED`.
- The paused waypoint index remained `0`.
- Completed count remained `0` during pause.
- No new goal was sent while paused.
- Resume resent the same waypoint with a new Nav2 UUID.
- Final completed count was `3`.
- Final progress was `1.0`.

Pause stop metrics:

- Pause request: `95067860196719`
- Action `CANCELED`: `95067914396670`
- Mission `PAUSED`: `95067917606449`
- Final zero command: `95067914788148`
- Final zero after pause request: `0.054591429 s`
- Nonzero commands after final zero: `0`
- Odometry samples after PAUSED: `130`
- TF samples after PAUSED: `130`
- Pause-window translation: `0.0013378546051139742 m`
- Pause-window yaw change: `0.0002379363646061172 rad`

Final success metrics after resume:

- Final action `SUCCEEDED`: `95110411903672`
- Mission `SUCCEEDED`: `95110418462620`
- Final zero command: `95110344259108`
- Final zero before action terminal: `0.067644564 s`
- Final zero before mission terminal: `0.074203512 s`
- Nonzero commands after final zero: `0`
- Command frequency: `18.642039099869677 Hz`
- Final XY error: `0.24872172565915507 m`
- Odometry samples after terminal: `129`
- TF samples after terminal: `129`
- Post-terminal translation: `0.0 m`
- Post-terminal yaw change: `0.0 rad`

Heartbeat evidence showed rosbag, fake base, subscriptions, and executor alive through both the pause and final success observation windows.

## Acceptance Matrix

| Criterion | Result |
| --- | --- |
| Typed interfaces frozen | PASS |
| Topology identity frozen | PASS |
| Legacy dense `/plan_nav` preserved as display path in mission mode | PASS |
| `mission_nav2` has no plan_nav velocity authority | PASS |
| `mission_nav2` has no plan_nav application UDP authority | PASS |
| Typed route reaches actual Nav2 | PASS |
| Sequential sparse goals succeed | PASS |
| Cancellation stops command generation and prevents later dispatch | PASS |
| Pause stops command generation | PASS |
| Resume resends the same waypoint | PASS |
| No-route, ambiguity, and topology-race rejection remain covered by P3-B.2 | PASS |
| No physical command topic, hardware, sensor, CAN, or application UDP access | PASS |
| Phase 2 frozen checksums unchanged | PASS |

## Limitations

Phase 3 remains a fake/mock Nav2 mission-integration validation. It does not validate physical navigation, obstacle avoidance, semantic perception, real localization freshness suppression, Collision Monitor, Generic Command Safety Gate, VehicleState, CAN integration, or wheelchair actuation.

The cached-transform freshness limitation identified in Phase 2 remains mandatory Phase 4/5 work. Phase 4 must provide command-safety suppression when localization/freshness permission is invalid. Phase 5 must independently validate localization freshness, transform age, finite pose/quaternion, jump policy, and stability policy before physical Nav2 arming.

## Phase 4 Entry Conditions

Before any physical authority is enabled:

- Keep Candidate C and Phase 2 frozen evidence intact.
- Implement the Generic Command Safety Gate.
- Implement localization-valid/freshness monitoring.
- Prove stale odometry/TF suppresses downstream safe vehicle commands.
- Keep Mission Manager typed route authority separate from velocity authority.
- Preserve explicit cancellation and pause stop evidence with post-terminal observation windows.
