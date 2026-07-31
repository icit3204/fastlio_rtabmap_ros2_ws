# Phase 1 Hard-Coded Path Inventory and Classification

Date: 2026-08-01
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
Source revision audited: `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb`
Authority SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`

## Final audit status

`PHASE1_HARD_CODED_PATH_AUDIT_PASS`

Every material machine-specific, user-specific, workspace-specific, obsolete,
or system absolute path found in current source/configuration, excluded local
source packages, backup copies, generated copies, documentation, and binary
assets was classified and assigned a remediation disposition.

No remediation was performed.

## Executive conclusion

The smallest safe Phase 1 closure set is not “zero grep matches.” It is the
active/reachable path set that can prevent relocation or conflict with the
stable Phase 1 build profile:

1. RTAB-Map database defaults under `/data/maps/...` in active
   `robot_bringup` launch files.
2. Offline path helper defaults under `/data/maps/...` that are launched by
   `offline_avoidance.launch.py`.
3. User-specific Torch library injection under
   `/home/dog/.local/lib/python3.10/site-packages/torch/lib` in active
   `robot_bringup` launches.
4. Default SuperPoint/PyMatcher paths in camera/infra launch defaults, because
   the stable Phase 1 RTAB-Map build has `WITH_TORCH=OFF` and
   `WITH_PYTHON=OFF`.

Replay-only scripts, build helpers, historical backups, generated YDLidar
build files, documentation/evidence paths, intentional `/opt/ros` and `/dev`
platform interfaces, and binary false positives do not belong in the minimum
Phase 1 closure edit set.

## Preflight and integrity

| Check | Result |
|---|---|
| Branch | `main` |
| HEAD | `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb` |
| `origin/main` | `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb` |
| Working tree | clean before audit |
| `phase1_native_build_verified` tag object | `a1dd9aac316d5981aee13904002dfeb6b4052894` |
| `phase1_native_build_verified^{}` peeled commit | `7556f15fb0b8e893ac799330bbf26a62ae19e439` |
| `phase0_verified_baseline` | `ae7517e3fe981bfb3fa148bb48ecc206cf1cdaa0` |
| Authority checksum | matched expected SHA256 |
| Process scan | no active ROS, colcon, compiler, sensor, controller, or CAN process found other than the audit shell/search commands |

## Search scope and method

Searched current workspace source and configuration, including:

- `src/`, including `src/ydlidar_ros2_driver/` and `src/YDLidar-SDK/`;
- `plan_nav/`;
- root scripts and `scripts/`;
- launch files, Python, C++, CMake, package XML, YAML/JSON/RViz, shell,
  Markdown, README, map/database utilities, model/camera/replay helpers;
- backup copy `plan_nav07202900`;
- generated YDLidar build/install copies, classified separately;
- reference Phase 1 documentation and reports.

Patterns included:

`/home/dog/`, `/home/`, `/workspace/`, `/root/`, `/usr/local/`, Windows drive
paths, `file://`, `package://`, `model_path`, `script_path`,
`database_path`, `map_path`, `config_path`, `torch`, `superpoint`,
`pymatcher`, `replay`, `rosbag`, `bag_path`, `os.path`, `pathlib.Path`,
`expanduser`, `get_package_share_directory`, `ament_index_cpp`,
`std::filesystem`, environment variables, launch substitutions, and
`FindPackageShare`.

Large binary assets produced path-like byte false positives; those were
classified as `FALSE_POSITIVE` and excluded from source remediation.

## Occurrence summary

Material finding rows in the matrix: 59.

Classification counts:

| Classification | Count |
|---|---:|
| `ACTIVE_RUNTIME_BLOCKER` | 7 |
| `ACTIVE_BUT_PARAMETER_OVERRIDABLE` | 16 |
| `OPTIONAL_FEATURE_BLOCKER` | 7 |
| `REPLAY_OR_TEST_ONLY` | 13 |
| `BUILD_OR_DEVELOPMENT_ONLY` | 7 |
| `DOCUMENTATION_ONLY` | 1 |
| `GENERATED_OR_INSTALLED_COPY` | 1 |
| `BACKUP_OR_HISTORICAL` | 2 |
| `SYSTEM_PATH_INTENTIONAL` | 4 |
| `FALSE_POSITIVE` | 1 |
| `UNRESOLVED` | 0 |

Minimum closure groups:

| Group | Count |
|---|---:|
| `MUST_FIX_FOR_PHASE1_CLOSURE` | 21 |
| `DEFER_OPTIONAL_OR_LATER_FEATURE` | 19 |
| `NO_CHANGE_REQUIRED` | 19 |

The detailed matrix is:

`/home/dog/phase1_reports/PHASE1_HARD_CODED_PATH_MATRIX.tsv`

## Runtime reachability

### Normal bringup/navigation/mapping launches

`bringup.launch.py`, `fastlio_mapping.launch.py`, `fast_lio2.launch.py`, and
`rtabmap_bridge.launch.py` default `database_path` to
`/data/maps/site_a/rtabmap.db`. The launch arguments are overridable, but the
default is active and site-specific.

The same launch family conditionally appends
`/home/dog/.local/lib/python3.10/site-packages/torch/lib` to
`LD_LIBRARY_PATH` when `COLCON_PREFIX_PATH` exists. That logic is evaluated
during launch description generation, not only when a Torch feature is
selected. This is active and user-specific.

### 2D/camera launches

`bringup_2d.launch.py`, `bringup_2d_infra.launch.py`, and
`fastlio_mapping_infra_2d.launch.py` include default `rtabmap_args` selecting
SuperPoint and PyMatcher paths under `./superpoint_superglue/...`. These are
relative, but still non-reproducible because the referenced model/script tree
is excluded from Git and the stable Phase 1 RTAB-Map build has Torch and
Python support disabled. They are part of the already-documented unsupported
camera defaults and should be made default-safe for Phase 1 closure.

### Offline/replay tooling

`offline_avoidance.launch.py` reaches `path_publisher.py` and
`path_waypoint_sender.py`, so its `/data/maps/db/...` defaults are reachable
when that offline launch is used. Manual replay/extraction scripts use
additional `/home/dog/...` and `/data/...` paths, but those do not affect
normal production startup.

### plan_nav

Current authoritative `plan_nav/main.py` constructs `MainWindow` without an
absolute asset path. GUI settings, datasets, topology files, and model
weights are resolved from the plan_nav project root or user-selected database
path. Operation mode probes `/opt/ros` in `core/pose_receiver.py` and
`core/nav_publisher.py`; this is an intentional ROS installation interface,
not a `/home/dog` relocation blocker.

### Sensor drivers

YDLIDAR `/dev/ttyUSB0` and `/dev/ydlidar` defaults are hardware interface
defaults and are parameter-overridable. They should be robot-profile values
for later live deployment, but are not Phase 1 path-closure blockers.

Livox `/home/livox/livox_test.lvx` defaults are parameterized LVX playback
defaults in Livox source and launch files. The file is missing on Jetson 2.
Because normal MID360 live operation does not require LVX playback, this is
deferred unless a later launch audit proves the driver attempts to open it in
live mode.

## Camera/Torch path findings

Stable Phase 1 RTAB-Map evidence states `WITH_TORCH=OFF` and
`WITH_PYTHON=OFF`. The following are therefore not safe stable defaults:

- seven `robot_bringup` launch files append `/home/dog/.local/.../torch/lib`;
- `bringup_2d.launch.py`, `bringup_2d_infra.launch.py`, and
  `fastlio_mapping_infra_2d.launch.py` pass SuperPoint/PyMatcher paths by
  default.

These are Phase 1 closure items because they are active launch defaults even
though the optional camera/Torch feature itself is deferred.

## Map/database findings

Active/current map database defaults:

- `/data/maps/site_a/rtabmap.db`;
- `/data/maps/db/first_version_0514.db`;
- `/data/maps/db/path_waypoints.yaml`.

Correct replacement depends on intent:

- production site DB: `SITE_PROFILE_PATH` or `ROBOT_PROFILE_PATH`;
- packaged example/offline demo: `LAUNCH_ARGUMENT_WITH_PACKAGE_SHARE_DEFAULT`;
- manual extraction/replay: required `COMMAND_LINE_ARGUMENT`.

No map/database content should be moved in this task or in the first
parameterization edit.

## Replay/test findings

Replay helpers under `scripts/` and root shell files include old workspace and
bag paths under `/home/dog/catkin_byd_1/...` and
`/home/dog/fastlio_rtabmap_ros2_ws/bags/...`. These are not production
startup blockers. The correct remediation is to require arguments or remove
obsolete defaults after preservation policy is approved.

## System path findings

`/opt/ros`, `/usr/local`, `/dev`, `/sys`, and `/proc` occurrences were not
automatically treated as errors.

- `/opt/ros` in plan_nav and build scripts is intentional ROS discovery.
- `/dev/...` in YDLIDAR is a configurable hardware interface.
- `/usr/local` in build scripts and Livox CMake is build/development scope,
  not runtime relocation scope.
- Generated YDLidar build files contain many `/home/dog/...` and
  `/usr/local/...` paths; those are not authoritative source.

## Duplicate and backup findings

`plan_nav07202900` contains backup/historical duplicates of plan_nav and local
Windows adaptation documentation. No current startup path references that
backup. Do not edit backup copies for active authority closure.

## Minimum Phase 1 closure scope

### A. MUST_FIX_FOR_PHASE1_CLOSURE

- HCP-001 through HCP-007;
- HCP-010;
- HCP-023 through HCP-035.

These cover active database defaults, active user-specific Torch LD injection,
and active default unsupported SuperPoint/PyMatcher paths.

### B. DEFER_OPTIONAL_OR_LATER_FEATURE

- HCP-008, HCP-009, HCP-011, HCP-012;
- HCP-013 through HCP-022;
- HCP-036 through HCP-038;
- HCP-043 and HCP-044.

### C. NO_CHANGE_REQUIRED

- HCP-039 through HCP-042;
- HCP-045 through HCP-059.

## Proposed validation

For the later remediation task:

1. Static launch inspection confirms no active Phase 1 launch default embeds
   `/home/dog` or inappropriate `/data` paths.
2. Static search confirms no stable default passes unavailable SuperPoint or
   PyMatcher paths.
3. Launch files parse under a clean environment without opening hardware.
4. Package install validation builds only touched packages.
5. No ROS nodes, hardware drivers, CAN, UDP, or motion processes are started.

## Integrity confirmation

- No source, launch, YAML, scripts, CMake, or documentation files were edited.
- No build was run.
- No ROS node or launch file was run.
- No sensor, CAN, UDP, controller, or hardware transport was accessed.
- No package was installed or removed.
- No symlink or file move was performed.
- No commit, tag, or push was performed.
- Only reports were created under `/home/dog/phase1_reports/`.

Full Phase 1 closure is not declared by this audit.
