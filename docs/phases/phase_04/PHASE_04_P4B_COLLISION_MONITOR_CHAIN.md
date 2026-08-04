# Phase 4 P4-B Collision Monitor Chain

## Baseline and Provenance

P4-B resumes from `main` commit
`283686d4c18919a27d9120e3a74311cd7c05d9a7`
(`docs(phase4): define stale observation safety gate`).  The preserved P4-B
implementation was restored with:

- stash object: `62834daaf4a143d69b32742b4f6603999bc5f2e3`
- stash message: `P4-B stale-source fail-open preserved WIP before P4-A.2`
- stash patch SHA256:
  `a831dc186bbe7de35d64262f204cc88e35bf22cfde8b7d9a397c5ef9d40188bd`
- preservation archive SHA256:
  `8e222a1113fdad4e336d370cc755a3e45ad503a6b63084a99a96e65fa195f9c4`
- preservation manifest SHA256:
  `bcf08bfd4bb51c7a84be86de08e039bb4cf010ca4a52b79babdc9d4ad3c1c019`

P4-B implements only:

`Nav2 controller -> /cmd_vel_nav_raw -> Collision Monitor -> /cmd_vel_nav_safe -> Phase 2 fake base`

It does not implement `/vehicle_cmd_safe`,
`/system/collision_monitor_valid`, the Generic Command Safety Gate, or the
wheelchair command adapter.

## Installed Collision Monitor

The locally installed package is `ros-humble-nav2-collision-monitor`
`1.1.20-1jammy.20260607.134559`, executable
`/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor`.  The command
input and output type is `geometry_msgs/msg/Twist`.

The P4-B configuration uses the installed Humble schema:

- `cmd_vel_in_topic`
- `cmd_vel_out_topic`
- `base_frame_id`
- `odom_frame_id`
- `transform_tolerance`
- `source_timeout`
- `stop_pub_timeout`
- `polygons`
- polygon `type: "polygon"`
- polygon `points`
- polygon `action_type: "stop"` or `"slowdown"`
- polygon `max_points`
- polygon `slowdown_ratio`
- polygon `visualize`
- polygon `polygon_pub_topic`
- polygon `enabled`
- `observation_sources`
- source `type: "scan"` or `"pointcloud"`
- source `topic`
- source `enabled`
- PointCloud2 `min_height` and `max_height`

## Topic Routing

The P4-B launch remaps the Nav2 controller output from `/cmd_vel` to
`/cmd_vel_nav_raw`.  Collision Monitor consumes `/cmd_vel_nav_raw` and
publishes `/cmd_vel_nav_safe`.  The frozen Phase 2 fake base is reused without
source modification by setting its existing `cmd_vel_topic` parameter to
`/cmd_vel_nav_safe`.

`/cmd_vel_nav_safe` means Collision Monitor-filtered Twist under the installed
package behavior.  It does not mean independently fail-safe generic chassis
command.  It must not reach a chassis adapter directly; stale-source chain
safety is assigned to P4-C through `/system/collision_monitor_valid` and the
Generic Command Safety Gate.

## Synthetic Sources

The deterministic fixture publishes only synthetic observations:

- LaserScan: `/phase4/synthetic_scan`
- PointCloud2: `/phase4/synthetic_points`

Both use `base_footprint` as the frame and publish at 20 Hz or faster.  The
runtime mode is externally controlled with a standard ROS 2 parameter named
`mode` with values `CLEAR`, `SLOW`, `STOP`, and `SILENT`.

`CLEAR` publishes fresh valid observations with no points in action polygons.
`SLOW` publishes deterministic points inside the slowdown polygon and outside
the stop polygon.  `STOP` publishes deterministic points inside the stop
polygon.  `SILENT` remains available for later P4-C stale-observation testing,
but it is not a P4-B acceptance criterion.

## Collision Geometry

The stop polygon is a front-centered rectangle inside the slowdown polygon and
does not include the robot origin:

- STOP: `(0.05, 0.35)`, `(0.45, 0.35)`, `(0.45, -0.35)`, `(0.05, -0.35)`
- SLOWDOWN: `(0.05, 0.50)`, `(0.90, 0.50)`, `(0.90, -0.50)`, `(0.05, -0.50)`

The slowdown ratio is `0.30`.  Both polygons use `max_points: 3`, and the
fixture publishes more than three deterministic points in the selected region.

## Fresh-Source Validation Scope

P4-B validates fresh-source Collision Monitor filtering:

- clear observations pass raw commands without unexpected scaling;
- slowdown observations reduce output by the configured ratio;
- stop observations force safe zero while raw remains nonzero;
- clear observations after stop release safe nonzero output;
- LaserScan and PointCloud2 synthetic sources are both recognized;
- publisher ownership remains unique;
- the fake base consumes only `/cmd_vel_nav_safe`;
- forbidden command, gate, wheelchair, live sensor, application UDP, CAN, and
  vcan paths are absent.

## Installed Stale-Source Limitation

Prior stopped P4-B evidence remains authoritative for stale-source behavior:

- raw archive SHA256:
  `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1`
- raw manifest SHA256:
  `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea`
- classification: `INSTALLED_COLLISION_MONITOR_STALE_SOURCE_FAIL_OPEN`
- `silent_safe_zero_count = 0`
- `silent_zero_latency_sec = null`

P4-B does not claim chain-level stale-source safety.  P4-C must enforce stale
observation safety using `/system/collision_monitor_valid` and the Generic
Command Safety Gate.

## Results

Focused static tests passed:

- command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/parking_robot_bringup/test/test_phase4_p4b_collision_monitor_contract.py`
- result: `20 passed in 0.86s`

The fresh-source preflight status is
`P4B_RESUMED_FRESH_SOURCE_PREFLIGHT_PASS`.

LaserScan preflight used `CLEAR -> SLOW -> STOP -> CLEAR`:

| Metric | Value |
| --- | ---: |
| raw samples | 249 |
| safe samples | 237 |
| clear raw nonzero samples | 52 |
| clear safe nonzero samples | 52 |
| slowdown ratio | 0.30 |
| stop latency | 0.047951798 s |
| release latency | 0.076666325 s |
| stop safe-zero samples | 41 |
| all raw/safe values finite | true |

PointCloud2 preflight used `CLEAR -> STOP -> CLEAR`:

| Metric | Value |
| --- | ---: |
| raw samples | 197 |
| safe samples | 186 |
| clear raw nonzero samples | 52 |
| clear safe nonzero samples | 52 |
| stop latency | 0.054334870 s |
| release latency | 0.046719790 s |
| stop safe-zero samples | 41 |
| all raw/safe values finite | true |

Nav2 runtime used exactly one initial pose and one external
`NavigateToPose` goal per accepted scenario:

- initial pose: `x = 5.425`, `y = -53.725`, `yaw = 0.0`
- goal: `x = 8.425`, `y = -53.725`, `yaw = 0.0`
- tolerance: `XY <= 0.25 m`, `yaw <= 0.10 rad`

The P4-B launch starts the frozen map/localization support, Nav2 planner,
Nav2 controller, BT Navigator, the frozen Phase 2 fake base, installed
Collision Monitor, Collision Monitor lifecycle manager, and the selected
synthetic source fixture.  It does not start `behavior_server`, velocity
smoother, Mission Manager, `plan_nav`, Pure Pursuit, legacy laser filter,
wheelchair controller, Generic Safety Gate, mock adapter, live sensors,
application UDP, CAN, or vcan paths.  The BT Navigator is configured with the
installed no-recovery tree
`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml`
so the runtime does not introduce a recovery behavior Twist publisher.

Scenario A, clear-path transparency:

| Metric | Value |
| --- | ---: |
| result | SUCCEEDED |
| raw samples | 827 |
| safe samples | 827 |
| clear ratio | 1.00 |
| safe frequency while moving | 20.033212529 Hz |
| final XY error | 0.246870526 m |
| final yaw error | 0.004398929 rad |
| path samples | 40 |
| raw publisher max | 1 |
| safe publisher max | 1 |
| forbidden publisher max | 0 |

Scenario B, slowdown and release:

| Metric | Value |
| --- | ---: |
| result | SUCCEEDED |
| raw samples | 859 |
| safe samples | 859 |
| clear ratio | 1.00 |
| measured slow ratio | 0.30 |
| release latency | 0.016879457 s |
| safe frequency while moving | 20.029683709 Hz |
| final XY error | 0.245892246 m |
| final yaw error | 0.006126557 rad |
| raw publisher max | 1 |
| safe publisher max | 1 |
| forbidden publisher max | 0 |

Scenario C, stop and release:

| Metric | Value |
| --- | ---: |
| result | SUCCEEDED |
| raw samples | 876 |
| safe samples | 864 |
| stop latency | 0.096935937 s |
| release latency | 0.039628401 s |
| stop safe-zero samples | 40 |
| stop raw-nonzero samples | 52 |
| stop-window translation | 0.003108229 m |
| stop-window yaw change | 0.000026180 rad |
| safe frequency while moving | 18.837872196 Hz |
| final XY error | 0.244550547 m |
| final yaw error | 0.003366399 rad |
| raw publisher max | 1 |
| safe publisher max | 1 |
| forbidden publisher max | 0 |

Full external build and regression passed after disabling user-site pytest
plugin autoload for the test step:

- build packages: `parking_robot_interfaces`, `parking_robot_mission_manager`,
  `parking_robot_bringup`
- test result: `149 passed` in `parking_robot_bringup`
- test result: `50 passed` in `parking_robot_mission_manager`
- combined result: `199 passed`, `0 failed`, `0 errors`, `0 skipped`
- dense path regression remained covered by the existing bringup regression
  and passed `47/47`

The first full `colcon test` attempt failed before project tests ran because a
user-site pytest plugin imported a missing `_pytest.scope` symbol through
`anyio`.  The accepted rerun set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, matching
the focused-test environment, and all project tests passed.

The final evidence package records the fresh LaserScan preflight, fresh
PointCloud2 preflight, Nav2 Scenario A/B/C measurements, raw/safe
correspondence, slowdown ratio, stop/release latency, stop-window odometry,
publisher ownership, process authority, forbidden-topic proof, rosbag
metadata, build and test logs, and the prior stale-source limitation hashes.

## P4-C Entry Conditions

P4-C may begin after P4-B fresh-source filtering is committed and pushed.  P4-C
must implement or provide `collision_monitor_validity_monitor`, consume
`/system/collision_monitor_valid` in `guarded_vehicle_cmd_gate`, and prove stale
synthetic collision observation cannot pass nonzero commands beyond
`/vehicle_cmd_safe`.
