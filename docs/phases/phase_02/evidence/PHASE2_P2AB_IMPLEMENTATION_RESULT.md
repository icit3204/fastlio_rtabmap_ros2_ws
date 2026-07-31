# Phase 2 P2-A/P2-B Implementation Result

Date: 2026-08-01  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Status

`PHASE2_P2AB_STATIC_IMPLEMENTATION_PASS`

Runtime status: `NOT_STARTED`

No Nav2 runtime launch was executed. No goal was sent. P2-C and later stages remain open.

## Source revision

Before implementation:

- Branch: `main`
- HEAD: `2dc53a117ead6bac5802bdd318b64337ebada930`
- `origin/main`: `2dc53a117ead6bac5802bdd318b64337ebada930`
- Working tree: clean

After implementation:

- New source tree staged for one commit: `feat(phase2): add isolated Nav2 fake-base baseline`
- Final commit SHA is recorded by git and the task final handoff after commit/push.

Required tags were verified before implementation and were not modified.

## Files created

Created isolated package:

`src/parking_robot_bringup/`

Package structure:

```text
src/parking_robot_bringup/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/parking_robot_bringup
├── parking_robot_bringup/
│   ├── __init__.py
│   ├── phase2_fake_base.py
│   └── phase2_fake_base_math.py
├── launch/phase2_core_nav2.launch.py
├── config/
│   ├── phase2_nav2_params.yaml
│   └── phase2_test_scenarios.yaml
├── maps/
│   ├── phase2_clean_map.yaml
│   ├── phase2_clean_map.pgm
│   └── PHASE2_MAP_PROVENANCE.md
├── test/
│   ├── test_phase2_fake_base_math.py
│   ├── test_phase2_config_scope.py
│   └── test_phase2_map_scenarios.py
└── README.md
```

Created Phase 2 documentation:

- `docs/phases/phase_02/PHASE_02_CORE_NAV2_BASELINE.md`
- `docs/phases/phase_02/PHASE_02_EVIDENCE_INDEX.md`
- `docs/phases/phase_02/evidence/`

## Map provenance

Copied, not symlinked:

- `scripts/offline_nav_maps/clean_map.yaml` -> `src/parking_robot_bringup/maps/phase2_clean_map.yaml`
- `scripts/offline_nav_maps/clean_map.pgm` -> `src/parking_robot_bringup/maps/phase2_clean_map.pgm`

The copied PGM is byte-identical:

`69428c94a8032fd54939492afe848156ef1687faf7a8023609a3e26d9604beb0`

The copied YAML differs only to point at `phase2_clean_map.pgm`.

See:

- `src/parking_robot_bringup/maps/PHASE2_MAP_PROVENANCE.md`
- `/home/dog/phase2_reports/PHASE2_MAP_SCENARIO_SELECTION.md`

## Selected static scenarios

| Scenario | Pixel | Metric pose | Classification | Static evidence |
|---|---|---|---|---|
| start | (890, 2054) | (5.425, -53.725, 0.0) | free | component 1, 7.393 m clearance |
| goal_a | (1254, 2602) | (23.625, -81.125, 0.0) | free | component 1, 5.430 m clearance |
| goal_b | (788, 2083) | (0.325, -55.175, 0.0) | free | component 1, 5.437 m clearance |
| goal_c | (876, 2147) | (4.725, -58.375, 0.0) | free | component 1, 5.423 m clearance |
| planner_failure_goal | (621, 1154) | (-8.025, -8.725, 0.0) | occupied | occupied probability 1.0 |

## Fake-base design

Implemented:

- `parking_robot_bringup.phase2_fake_base_math`: pure, ROS-free integration helpers.
- `parking_robot_bringup.phase2_fake_base`: ROS node wrapper.

ROS interfaces:

- subscribes to `/cmd_vel_phase2_mock` by default;
- subscribes to `/initialpose`;
- publishes `/Odometry`;
- broadcasts `odom -> base_footprint`.

Safety and behavior:

- rejects NaN/infinite commands;
- uses monotonic time for command staleness;
- stale or missing command integrates zero velocity;
- clamps abnormal integration timestep;
- `/initialpose` atomically resets pose and clears active command freshness;
- publishes finite normalized yaw quaternion;
- does not publish/use `body`;
- no file, hardware, CAN, UDP, legacy controller, or real command topic access.

## Unit/static tests

Local source-level validation:

- Python syntax compilation: PASS
- YAML parsing: PASS
- package XML parsing: PASS
- pytest with plugin autoload disabled: 22 passed

Covered:

- zero command;
- straight integration;
- positive/negative yaw rotation;
- combined curve;
- yaw normalization;
- NaN/infinity rejection;
- large-dt clamping;
- stale-command zeroing;
- reset semantics;
- quaternion normalization;
- excluded-scope strings;
- costmap plugin scope;
- velocity topic isolation;
- map/scenario classification.

## Nav2 configuration

Config:

`src/parking_robot_bringup/config/phase2_nav2_params.yaml`

Selected Nav2 profile:

- `use_sim_time: false`
- map server with launch-supplied packaged map
- NavFn planner plugin `nav2_navfn_planner/NavfnPlanner`
- `use_astar: true`
- `allow_unknown: false`
- MPPI controller plugin `nav2_mppi_controller::MPPIController`
- `motion_model: DiffDrive`
- odom topic `/Odometry`
- global costmap frame `map`
- local costmap frame `odom`
- robot base frame `base_footprint`
- static and inflation layers only
- no obstacle layer
- no voxel layer
- no observation sources
- no keepout/speed filters

Plugin IDs were verified against installed Humble plugin metadata.

## Selected BT

Selected installed Humble BT XML:

`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

Required servers/actions are covered by launched Phase 2-safe nodes:

- planner_server: `ComputePathToPose`
- controller_server: `FollowPath`
- behavior_server: `Spin`, `BackUp`, `Wait`
- controller/planner costmaps: clear-costmap services
- bt_navigator: NavigateToPose action server

Behavior-server velocity output is remapped to `/cmd_vel_phase2_mock`.

## Launch node matrix

See:

`/home/dog/phase2_reports/PHASE2_P2AB_STATIC_NODE_AND_TOPIC_MATRIX.tsv`

Summary:

- exactly one `map -> odom` static TF publisher;
- exactly one `odom -> base_footprint` TF publisher (`phase2_fake_base`);
- no `body` frame;
- controller_server `/cmd_vel` remapped to `/cmd_vel_phase2_mock`;
- behavior_server `/cmd_vel` remapped to `/cmd_vel_phase2_mock`;
- no `/cmd_vel_nav`, `/wheelchair_control_command`, or `/vehicle_cmd_safe`.

## Excluded-component verification

Static tests and targeted inspection verified no reference to excluded runtime components in the Phase 2 launch/config:

- no semantic grid topics/tools;
- no Collision Monitor;
- no Generic Command Safety Gate;
- no Wheelchair Command Adapter;
- no wheelchair controller;
- no CAN/can0;
- no plan_nav;
- no Pure Pursuit;
- no Livox/MID-360;
- no FAST-LIO;
- no RTAB-Map live localization;
- no camera;
- no YDLIDAR;
- no ultrasonic sensors;
- no keepout/speed filters;
- no Gazebo;
- no Phase 3 Mission Manager.

## Build/install validation

Validation root:

`/home/dog/phase2_builds/p2ab_scaffolding_validation_20260801_025103/`

Build/test:

- clean `env -i`;
- ROS Humble sourced;
- stable Phase 1 ROS underlay sourced;
- stable RTAB-Map/OpenCV prefixes supplied only as underlays;
- package selected: `parking_robot_bringup`;
- sequential executor;
- one worker;
- tests enabled.

Result:

- build succeeded;
- only `parking_robot_bringup` built;
- colcon test result: 22 tests, 0 errors, 0 failures, 0 skipped;
- installed launch/config/map/Python files exist;
- installed checksums match source;
- installed map YAML resolves to installed `phase2_clean_map.pgm`.

## Limitations and P2-C prerequisites

Not yet done:

- no Nav2 runtime launch;
- no initial pose publication;
- no NavigateToPose goal;
- no planner success/failure runtime evidence;
- no controller/fake-base runtime odometry evidence;
- no RViz runtime;
- no physical sensor or motion validation.

P2-C prerequisites:

- run the isolated launch in localhost/domain-isolated mode;
- publish initial pose only within the Phase 2 isolated runtime;
- send the selected goal sequence;
- collect Nav2 action, path, odometry, and TF evidence;
- confirm only `/cmd_vel_phase2_mock` receives velocity commands.

## Integrity confirmation

During P2-A/P2-B:

- no ROS node was launched;
- no message was published;
- no Nav2 goal was sent;
- no sensor, hardware, CAN, or UDP access occurred;
- no system package was installed or removed;
- no RTAB-Map rebuild occurred;
- no legacy package/source was modified;
- no existing Phase 0/1 tag was modified.

