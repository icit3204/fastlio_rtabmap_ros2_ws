# Phase 1 Unified RViz Profile Validation

Date: 2026-08-01
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
Source revision: `32a3057e9a9d5ff12be910925f7666dad63fb1cb`
Authority document SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`

## Closure decision

`PHASE1_UNIFIED_RVIZ_PROFILE_VALIDATION_PASS`

The unified Phase 1 RViz baseline profile exists, parses as YAML, uses only
current implemented/configured topics and frames, uses Humble default RViz
display classes, installs with `robot_bringup`, and contains no machine-local
absolute paths. The graphical smoke test was not completed because Qt could
not connect to the available display; this is recorded as
`NOT_RUN_NO_DISPLAY` and is not treated as a profile configuration failure.

This report does not declare full Phase 1 closure. Hard-coded-path
parameterization remains open.

## Preflight

| Check | Result |
|---|---|
| Branch | `main` |
| HEAD | `32a3057e9a9d5ff12be910925f7666dad63fb1cb` |
| `origin/main` | matched HEAD before the profile commit |
| Working tree | clean before changes |
| Authority checksum | matched expected SHA256 |
| Stable OpenCV sysroot | present: `/home/dog/phase1_builds/opencv454d_development_sysroot_20260731_035528/sysroot` |
| Stable RTAB-Map install | present: `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459/install` |
| Stable ROS install | present: `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543/install` |
| Process scan | sandbox-visible process scan found no ROS, RViz, sensor, controller, CAN, compiler, or colcon process before validation |
| Existing tags | `phase0_verified_baseline`, `phase1_native_build_verified` present before changes |

Process-scan limitation: the command ran inside the managed sandbox PID
namespace, so it confirms the absence of matching processes visible to this
task environment.

## Existing RViz profile inventory

| Path | Owning package | Fixed frame | Important displays | Absolute paths | Missing/unavailable plugins | Installed by package | Baseline without modification |
|---|---|---|---|---|---|---|---|
| `src/FAST_LIO_ROS2/rviz/fastlio.rviz` | `fast_lio` | `odom` | TF, `/Odometry`, `/path`, disabled `/cloud_registered_body`, RTAB-Map plugin displays | none | RTAB-Map plugin dependency, not Humble default only | yes, via package RViz install | no; fixed frame is not `map`, missing `/map` baseline, extra non-default plugins |
| `src/FAST_LIO_ROS2/rviz_cfg/loam_livox.rviz` | `fast_lio` | `odom` | legacy Grid, TF, point cloud, odometry/path style displays | none | uses old `rviz/*` class identifiers, not current Humble `rviz_default_plugins/*` syntax | no package install rule identified for `rviz_cfg` | no; legacy syntax and fixed frame |
| `src/rtabmap_ros/rtabmap_launch/launch/config/rgbd.rviz` | `rtabmap_launch` | `map` | RGB-D/camera/RTAB-Map-oriented displays | none | RTAB-Map RViz plugins required | yes, RTAB-Map package resource | no; RGB-D demo/RTAB-Map profile, not Phase 1 authority baseline |
| `src/rtabmap_ros/rtabmap_demos/config/demo_robot_mapping.rviz` | `rtabmap_demos` | `map` | `/map`, demo laser/camera/RTAB-Map displays | none | RTAB-Map RViz plugins required | yes, RTAB-Map package resource | no; demo-specific topics |
| `src/rtabmap_ros/rtabmap_examples/config/slam_D405x3_config.rviz` | `rtabmap_examples` | `map` | D405 camera and RTAB-Map example displays | none | RTAB-Map RViz plugins required | yes, RTAB-Map package resource | no; camera-example-specific |
| `src/rtabmap_ros/rtabmap_examples/config/slam_D405x2_config.rviz` | `rtabmap_examples` | `map` | D405 camera and RTAB-Map example displays | none | RTAB-Map RViz plugins required | yes, RTAB-Map package resource | no; camera-example-specific |
| `src/robot_bringup/config/fast_lio2.rviz` | `robot_bringup` | `map` | `/cloud_registered_body`, `/Odometry`, `/rtabmap/map`, RTAB-Map local map displays | none | RTAB-Map plugin dependency | yes; `robot_bringup` installs `launch config` | no; lacks RobotModel and current `/map` display |
| `src/robot_bringup/config/nav2_navigation.rviz` | `robot_bringup` | `map` | TF, `/grid_prob_map`, `/plan`, `/transformed_global_plan`, local/global costmaps, collision monitor polygons, markers | none | Humble default display classes available; operational Nav2 focus | yes; `robot_bringup` installs `launch config` | no; map topic is not `/map`, many operational/expensive displays enabled |

## Selected authoritative profile location

Selected source path:

`src/robot_bringup/config/phase1_authority_baseline.rviz`

Reason: `robot_bringup` already uses `config/` as its installed RViz resource
directory, and `src/robot_bringup/CMakeLists.txt:6` installs `launch config`
generically. No new ROS package and no CMake change were required.

Installed validation path:

`/home/dog/phase1_builds/phase1_rviz_profile_validation_20260801_004358/install/share/robot_bringup/config/phase1_authority_baseline.rviz`

## Verified current topic and frame sources

This section records source/configured topic truth only. No runtime topic
publication is claimed.

| Item | Source evidence |
|---|---|
| `map` frame | `src/robot_bringup/launch/rtabmap_bridge.launch.py:32` default `map_frame_id=map`; `:59` passes `publish_tf_map=true`; `src/robot_bringup/config/nav2_common.yaml:5` documents RTAB-Map publishing `/map` and `map -> odom` |
| `odom` frame | `src/FAST_LIO_ROS2/src/laserMapping.cpp:650` sets odometry frame `odom`; `:668` sets TF parent `odom`; `src/robot_bringup/config/nav2_common.yaml:15` uses `/Odometry` |
| `body` frame | `src/FAST_LIO_ROS2/src/laserMapping.cpp:651` sets odometry child frame `body`; `:670` sets TF child `body`; `:580` publishes body-frame cloud with frame `body` |
| `base_footprint` frame | `src/robot_bringup/launch/bringup.launch.py:127-129` configures `base_footprint -> base_link`; `src/robot_bringup/launch/bringup_2d.launch.py:175-177` configures `body -> base_footprint` |
| `base_link` frame | `src/robot_bringup/launch/bringup.launch.py:127-129`; `src/robot_bringup/launch/bringup_2d.launch.py:183-185` |
| `livox_frame` frame | `src/robot_bringup/launch/bringup.launch.py:135-136`; `src/robot_bringup/launch/bringup_2d.launch.py:191-192` |
| `/map` | `src/robot_bringup/config/nav2_common.yaml:5`, `:180`; `src/robot_bringup/launch/rtabmap_bridge.launch.py:59` |
| `/Odometry` | `src/FAST_LIO_ROS2/src/laserMapping.cpp:954`; `src/robot_bringup/launch/rtabmap_bridge.launch.py:33`; `src/robot_bringup/config/nav2_common.yaml:15` |
| `/path` | `src/FAST_LIO_ROS2/src/laserMapping.cpp:955`; enabled by `src/FAST_LIO_ROS2/config/mid360.yaml:40` |
| `/cloud_registered` | `src/FAST_LIO_ROS2/src/laserMapping.cpp:950`; gated by scan publish settings at `:1093` and config `src/FAST_LIO_ROS2/config/mid360.yaml:43` |
| `/cloud_registered_body` | `src/FAST_LIO_ROS2/src/laserMapping.cpp:951`; config `src/FAST_LIO_ROS2/config/mid360.yaml:45`; used by `rtabmap_bridge.launch.py:36` and `nav2_common.yaml:144,192` |
| `/plan_nav` | `plan_nav/core/nav_publisher.py:141`; input to avoidance node in `src/robot_bringup/scripts/plan_nav_laser_avoidance.py:57` |
| `/active_plan` | `src/robot_bringup/scripts/plan_nav_laser_avoidance.py:58,160` |
| `/scan` | `src/robot_bringup/launch/bringup.launch.py:61`; `src/robot_bringup/launch/bringup_2d.launch.py:59`; YDLIDAR is conditional/default-off in `bringup.launch.py:56,90` |
| Nav2 global path `/plan` | existing `src/robot_bringup/config/nav2_navigation.rviz` uses global path display on `/plan` |
| Nav2 local controller path `/transformed_global_plan` | existing `src/robot_bringup/config/nav2_navigation.rviz` uses local path display on `/transformed_global_plan` |
| Nav2 costmaps | `src/robot_bringup/config/nav2_common.yaml:115-192` configures local/global costmaps; existing `nav2_navigation.rviz` uses `/global_costmap/costmap` and `/local_costmap/costmap` |

The profile deliberately does not embed either disputed numeric
`body -> base_footprint` z value. TF will display whichever transforms are
provided at runtime.

## Display inventory

| Display | Class | Topic | Default |
|---|---|---|---|
| Grid | `rviz_default_plugins/Grid` | n/a | enabled |
| TF | `rviz_default_plugins/TF` | TF system | enabled |
| RobotModel | `rviz_default_plugins/RobotModel` | `/robot_description` | enabled |
| Map | `rviz_default_plugins/Map` | `/map` | enabled |
| Odometry | `rviz_default_plugins/Odometry` | `/Odometry` | enabled |
| FAST-LIO Path | `rviz_default_plugins/Path` | `/path` | enabled |
| plan_nav Route | `rviz_default_plugins/Path` | `/plan_nav` | disabled |
| Active Legacy Route | `rviz_default_plugins/Path` | `/active_plan` | disabled |
| Registered Cloud | `rviz_default_plugins/PointCloud2` | `/cloud_registered` | disabled |
| Body-frame Registered Cloud | `rviz_default_plugins/PointCloud2` | `/cloud_registered_body` | disabled |
| LaserScan | `rviz_default_plugins/LaserScan` | `/scan` | disabled |
| Nav2 Global Path | `rviz_default_plugins/Path` | `/plan` | disabled |
| Nav2 Local Controller Path | `rviz_default_plugins/Path` | `/transformed_global_plan` | disabled |
| Global Costmap | `rviz_default_plugins/Map` | `/global_costmap/costmap` | disabled |
| Local Costmap | `rviz_default_plugins/Map` | `/local_costmap/costmap` | disabled |

RobotModel note: no source-owned URDF path or machine-local model file was
embedded. The profile uses the standard Humble RViz RobotModel topic
mechanism, `/robot_description`, so missing robot-description data remains
visible through RViz status reporting.

RViz tools were limited to non-publishing visualization/navigation tools:
Interact, MoveCamera, Select, FocusCamera, and Measure. Goal, initial-pose,
and clicked-point tools were omitted for this visualization-only baseline.

## Basic-health interpretation

The current system does not implement the future consolidated health
supervisor topics. For this Phase 1 profile, basic health means visual RViz
status and data availability checks for:

- TF connectivity and transform status;
- map availability on `/map`;
- odometry freshness on `/Odometry`;
- path availability on `/path` and optional path topics;
- RobotModel transform status from `/robot_description` and TF;
- optional sensor-topic presence for point clouds and `/scan`.

No diagnostic or future health-supervisor topics are represented as
implemented.

## Static validation

| Check | Result |
|---|---|
| YAML parse | PASS |
| Fixed frame | PASS: `map` |
| Duplicate/malformed top-level structure | PASS: none found by YAML parse/tree inspection |
| Absolute `/home/...` path in RViz profile | PASS: none |
| Future unimplemented topics | PASS: none of `/vehicle_cmd_safe`, `/vehicle_state`, semantic-grid, Mission Manager, or Health Supervisor |
| Display class availability | PASS: 21 `rviz_default_plugins/*` classes checked against `/opt/ros/humble/share/rviz_default_plugins/plugins_description.xml` |
| Required enabled states | PASS |
| Optional disabled states | PASS |
| Topic/display type pairing | PASS: Path topics use Path display, map/costmaps use Map display, clouds use PointCloud2, `/scan` uses LaserScan, `/Odometry` uses Odometry |

Source profile checksum:

`d5fe97de0d5ca3bd0d6752f149907b024800e846f174bc8fb6a9bfb69980f780`

## Package installation validation

Validation root:

`/home/dog/phase1_builds/phase1_rviz_profile_validation_20260801_004358`

Build command characteristics:

- `env -i` clean environment;
- `/opt/ros/humble` sourced;
- stable Phase 1 ROS install sourced;
- RTAB-Map stable install provided as a CMake prefix;
- OpenCV 4.5.4d sysroot provided through `OpenCV_DIR`;
- `--packages-select robot_bringup`;
- one colcon worker;
- `BUILD_TESTING=OFF`;
- no runtime launch files started.

Result:

```text
Starting >>> robot_bringup
Finished <<< robot_bringup [1.61s]

Summary: 1 package finished [2.27s]
```

Only `robot_bringup` appeared under the validation build directory.

Installed file:

`/home/dog/phase1_builds/phase1_rviz_profile_validation_20260801_004358/install/share/robot_bringup/config/phase1_authority_baseline.rviz`

Installed checksum:

`d5fe97de0d5ca3bd0d6752f149907b024800e846f174bc8fb6a9bfb69980f780`

The installed checksum matched the source checksum.

## RViz smoke test

Requested smoke command:

`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=194 rviz2 -d <installed profile>`

Result:

`NOT_RUN_NO_DISPLAY`

Observed output:

```text
qt.qpa.xcb: could not connect to display :0
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized.
Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
timeout: the monitored command dumped core
```

Interpretation: a `DISPLAY` value was present, but Qt could not connect to a
usable graphical session. No profile configuration-load or plugin-load result
was produced. Per task constraints, no Xvfb or other software was installed;
the strongest static YAML/plugin/resource/install validation was used instead.

## Limitations

- No runtime publishers were started, so live topic freshness, live TF
  connectivity, and live RobotModel rendering were not asserted.
- Graphical RViz smoke was not completed because no usable display was
  available to Qt from the task environment.
- The profile is a baseline visualization profile, not a launch default and
  not a navigation/control workflow.
- The disputed body-to-base_footprint z conflict was not resolved or encoded.
- Hard-coded-path parameterization remains a separate open Phase 1 closure
  item.

## Integrity confirmation

- No plan_nav source modified.
- No controller or CAN source modified.
- No launch behavior modified.
- No static transforms modified.
- No Nav2, RTAB-Map, FAST-LIO, Collision Monitor, or sensor parameters edited.
- No hardware drivers launched.
- No Nav2 launched.
- No Pure Pursuit or wheelchair controller launched.
- No CAN or UDP transport intentionally opened.
- No movement commands published.
- No packages installed or removed system-wide.
- No tags created, moved, or deleted during validation.
- Stable Phase 1 build roots were used as read-only underlays except for the
  separate validation root created for this task.
