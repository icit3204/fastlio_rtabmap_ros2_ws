# Phase 1 Hard-Coded Runtime-Path Remediation Result

Date: 2026-08-01  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Closure status

`PHASE1_HARD_CODED_PATH_REMEDIATION_PASS`

## Source revision

Before edits:

- Branch: `main`
- HEAD: `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb`
- `origin/main`: `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb`
- `phase0_verified_baseline`: `ae7517e3fe981bfb3fa148bb48ecc206cf1cdaa0`
- `phase1_native_build_verified`: `a1dd9aac316d5981aee13904002dfeb6b4052894`
- `phase1_native_build_verified^{}`: `7556f15fb0b8e893ac799330bbf26a62ae19e439`

Authority document SHA256 was confirmed before edits:

`d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`

## Findings resolved

All findings in the approved `MUST_FIX_FOR_PHASE1_CLOSURE` set were remediated:

- HCP-001 through HCP-007
- HCP-010
- HCP-023 through HCP-035

Post-remediation classification summary:

| Status | Count |
|---|---:|
| RESOLVED | 21 |
| DEFERRED_AS_APPROVED | 19 |
| NO_CHANGE_REQUIRED | 19 |
| REGRESSION | 0 |
| UNRESOLVED | 0 |

See: `/home/dog/phase1_reports/PHASE1_HARD_CODED_PATH_POST_REMEDIATION_MATRIX.tsv`

## Files modified

- `src/robot_bringup/launch/rtabmap_bridge.launch.py`
- `src/robot_bringup/launch/bringup.launch.py`
- `src/robot_bringup/launch/fastlio_mapping.launch.py`
- `src/robot_bringup/launch/fast_lio2.launch.py`
- `src/robot_bringup/launch/offline_view.launch.py`
- `src/robot_bringup/launch/offline_avoidance.launch.py`
- `src/robot_bringup/launch/bringup_2d.launch.py`
- `src/robot_bringup/launch/bringup_2d_infra.launch.py`
- `src/robot_bringup/launch/fastlio_mapping_infra_2d.launch.py`
- `src/robot_bringup/scripts/path_publisher.py`
- `src/robot_bringup/scripts/path_waypoint_sender.py`

No CAN source, controller behavior, command topics, TF values, map/database assets, plan_nav files, backups, generated files, or replay-only helpers outside the MUST_FIX set were edited.

## Database and waypoint path behavior

### RTAB-Map database defaults

The active `/data/maps/site_a/rtabmap.db` defaults were removed from:

- `rtabmap_bridge.launch.py`
- `bringup.launch.py`
- `fastlio_mapping.launch.py`
- `fast_lio2.launch.py`
- `offline_view.launch.py`

New behavior:

1. Explicit `database_path:=...` launch arguments remain accepted and take precedence.
2. If no explicit launch argument is supplied, `PARKING_ROBOT_RTABMAP_DATABASE` is used when set.
3. If neither is supplied, the default is an empty string rather than a site-specific absolute path.

### Offline path database

The active `/data/maps/db/first_version_0514.db` defaults were removed from:

- `offline_avoidance.launch.py`
- `path_publisher.py`

New behavior:

1. Explicit launch/ROS parameter values remain accepted.
2. `PARKING_ROBOT_OFFLINE_PATH_DATABASE` is used as an environment fallback.
3. If no database or waypoint YAML is configured, `path_publisher.py` logs a clear error and does not silently open an unrelated database.
4. Missing/non-file database paths are rejected before `sqlite3.connect()`.

### Offline waypoint YAML

The active `/data/maps/db/path_waypoints.yaml` default was removed from:

- `path_waypoint_sender.py`

`offline_avoidance.launch.py` now declares `path_yaml` with an explicit override and `PARKING_ROBOT_WAYPOINTS_FILE` fallback. The empty default is intentional: offline navigation requires an operator-selected waypoint file or environment/site profile.

`path_waypoint_sender.py` now rejects:

- empty input;
- nonexistent input;
- non-file input;
- non-`.yaml`/`.yml` input.

## Torch path behavior

The user-specific Torch library injection:

`/home/dog/.local/lib/python3.10/site-packages/torch/lib`

was removed from all seven affected launch files:

- `rtabmap_bridge.launch.py`
- `bringup.launch.py`
- `fastlio_mapping.launch.py`
- `fast_lio2.launch.py`
- `bringup_2d.launch.py`
- `bringup_2d_infra.launch.py`
- `fastlio_mapping_infra_2d.launch.py`

New behavior:

1. `RTABMAP_TORCH_LIB_DIR` unset: no Torch path is added.
2. `RTABMAP_TORCH_LIB_DIR` set to an existing directory: that directory is appended once.
3. `RTABMAP_TORCH_LIB_DIR` set to a nonexistent directory: it is not injected, and a clear warning is printed.
4. The stable RTAB-Map profile remains no-Torch/no-Python; setting the environment variable does not enable Torch features.

The pre-existing RTAB-Map library workaround remains otherwise unchanged.

## Stable camera/default behavior

The stable defaults no longer select unavailable SuperPoint/PyMatcher behavior in:

- `bringup_2d.launch.py`
- `bringup_2d_infra.launch.py`
- `fastlio_mapping_infra_2d.launch.py`

Removed from stable default `rtabmap_args` where present:

- `Kp/DetectorStrategy 11`
- `SuperPoint/ModelPath`
- `SuperPoint/Cuda`
- `SuperPointRpautrat/Cuda`
- `Vis/FeatureType 11`
- `Vis/CorNNType 6`
- `PyMatcher/Path`
- `PyMatcher/Cuda`
- `PyMatcher/Model outdoor`

Explicit `rtabmap_args:=...` overrides remain supported.

## Static validation

Command:

`python3 -m py_compile` on all 11 modified Python launch/script files.

Result: PASS.

Targeted search result:

- no MUST_FIX file contains `/home/dog/.local/lib/python3.10/site-packages/torch/lib`;
- no MUST_FIX file retains an active `/data/maps/...` default;
- no stable default contains the removed SuperPoint/PyMatcher model/script/default activation settings.

Residual note: `offline_view.launch.py` still contains one `/data/maps/...` usage example in its module docstring. It is not a runtime default and was not part of the approved active runtime-path edit set.

## Launch-description validation

Each modified launch file was imported directly and `generate_launch_description()` was called without launching nodes:

- `rtabmap_bridge.launch.py`: PASS
- `bringup.launch.py`: PASS
- `fastlio_mapping.launch.py`: PASS
- `fast_lio2.launch.py`: PASS
- `offline_view.launch.py`: PASS
- `offline_avoidance.launch.py`: PASS
- `bringup_2d.launch.py`: PASS
- `bringup_2d_infra.launch.py`: PASS
- `fastlio_mapping_infra_2d.launch.py`: PASS

Default launch-argument inspection confirmed:

- affected RTAB-Map `database_path` defaults resolve to empty when the environment fallback is unset;
- `offline_avoidance` `path_yaml` and `database_path` defaults resolve to empty when their environment fallbacks are unset;
- explicit launch arguments remain declared for operator override.

Torch environment checks confirmed:

- unset: no `/home/dog` Torch path and no Torch env path injected;
- valid temporary directory: env directory injected once;
- nonexistent directory: env directory not injected and warning printed.

No ROS graph, node process, hardware driver, CAN, UDP, or sensor process was started.

## Package build/install validation

Validation root:

`/home/dog/phase1_builds/phase1_path_parameterization_validation_20260801_012351/`

Build command characteristics:

- `env -i`
- ROS Humble underlay: `/opt/ros/humble`
- stable ROS install underlay: `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543/install`
- stable RTAB-Map prefix supplied through `CMAKE_PREFIX_PATH`
- OpenCV 4.5.4d sysroot supplied through `OpenCV_DIR`/`CMAKE_PREFIX_PATH`
- package selected: `robot_bringup`
- executor: sequential
- workers: 1
- `BUILD_TESTING=OFF`

Result:

- build succeeded;
- only `robot_bringup` built;
- modified launch files installed under `share/robot_bringup/launch`;
- modified scripts installed under `lib/robot_bringup`;
- installed checksums match source checksums.

Note: the build printed one non-fatal warning that the sysroot `/usr/local` prefix listed in `CMAKE_PREFIX_PATH` did not exist. The package has no compiled targets and the build/install validation still passed.

## Residual deferred findings

The approved deferred/no-change categories remain outside this remediation:

- replay/manual helper paths;
- optional LVX playback paths;
- YDLIDAR device paths;
- historical/backup/generated/documentation paths;
- future physical calibration;
- runtime RViz GUI smoke test;
- live perception and motion validation.

## Integrity confirmation

Confirmed during this task:

- no ROS node launched;
- no message published;
- no CAN, UDP, sensor, controller, or hardware access;
- no RTAB-Map rebuild;
- no system package installation/removal;
- no map/database asset moved, edited, deleted, or staged;
- no static transform value changed;
- no command/CAN/controller behavior changed;
- tags were not moved or recreated.

## Closure decision

`PHASE1_HARD_CODED_PATH_REMEDIATION_PASS`
