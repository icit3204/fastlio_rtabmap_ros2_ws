# PHASE2 P2D Final Authoritative Runtime Result

Generated: 2026-08-02T00:50:02.800982

Decision: `PHASE2_P2D_ONE_GOAL_RUNTIME_RETEST_PASS`

## Runtime

- Build root: `/home/dog/phase2_builds/p2d_final_authoritative_20260802_004444`
- Runtime root: `/home/dog/phase2_runtime/p2d_final_authoritative_20260802_004444`
- ROS_DOMAIN_ID: `182`
- ROS_LOCALHOST_ONLY: `1`
- External params overlay: none
- Validation mode: `navigation`
- NavigateToPose requests sent: `1`
- Direct FollowPath actions sent: `0`

## Action response

- Goal response received: `True`
- Goal response latency: `0.004880931999650784` sec
- Goal accepted: `True`
- Goal ID: `7feffb85898042008b77bd8c3d4ba529`
- Result response received: `True`
- Action result: `SUCCEEDED`
- Runner exit code: `0`
- State sequence: `WAITING_FOR_NAV2 -> SETTING_START -> VERIFYING_START -> GOAL_REQUEST_SENT -> GOAL_ACTIVE -> SUCCEEDED -> CLEANUP`

## Planning

- Path received: `True`
- Path topic: `/plan`
- Path frame: `map`
- Path poses: `10`
- Path length: `0.30606518986571024` m
- Path endpoint error: `0.0` m
- Occupied/unknown/out-of-bounds samples: `0` / `0` / `0`

## Controller and movement

- Command count: `827`
- Nonzero command count: `826`
- Active command-stream frequency: `20.02407033422651` Hz
- Controller action-window frequency: `19.999560965866362` Hz
- Max abs linear.x: `0.08317729830741882` m/s
- Max abs angular.z: `0.04828613996505737` rad/s
- Invalid commands: `0`
- Unsupported command fields: `0`
- Command-limit violations: `0`
- Initial pose: `{'x': 5.425, 'y': -53.725, 'yaw': 0.0}`
- Final pose: `{'x': 8.185453032013545, 'y': -53.78843856683632, 'yaw': 0.003909699763943841}`
- Final XY error: `0.24780476515545505` m
- Final yaw error: `0.003909699763943841` rad
- Total translation: `2.761417241533026` m
- Odom samples: `2327`
- Post-result command stop latency: `0.0` sec

## Isolation and shutdown

- `/navigate_to_pose` server count: one `/bt_navigator` server before goal.
- `/cmd_vel_phase2_mock` subscriber: `phase2_fake_base` only.
- Excluded physical/legacy nodes/topics: not observed.
- `Failed to send goal response` warning: `False`
- Fake-base clean exit: `True`
- Shutdown traceback: `False`

## Limitations

This pass covers one bounded simple straight 3 m fake closed-loop NavigateToPose scenario. It does not prove general curved-path reliability, obstacle avoidance, physical navigation, semantic perception, CAN/UDP command safety, or Phase 2 completion.
