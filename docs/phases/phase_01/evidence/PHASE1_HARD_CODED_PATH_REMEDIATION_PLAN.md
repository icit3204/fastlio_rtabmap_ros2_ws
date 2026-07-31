# Phase 1 Hard-Coded Path Remediation Plan

Date: 2026-08-01
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
Source revision audited: `cf33b22d7059a8e8bab1e3ecca96589ba1ed70fb`

## Audit completion status

`PHASE1_HARD_CODED_PATH_AUDIT_PASS`

This is a plan only. No source, launch, YAML, script, CMake, or
documentation file was modified.

## Minimum Phase 1 closure scope

### A. MUST_FIX_FOR_PHASE1_CLOSURE

These are active or realistically reachable defaults that can prevent
relocation to another Jetson/workspace or conflict with the stable Phase 1
Torch/Python-disabled profile.

| Finding IDs | Remediation type | Exact files | Before | After | Compatibility | Validation | Risk |
|---|---|---|---|---|---|---|---|
| HCP-001..HCP-006 | `SITE_PROFILE_PATH` or `LAUNCH_ARGUMENT_WITH_PACKAGE_SHARE_DEFAULT` | `src/robot_bringup/launch/rtabmap_bridge.launch.py`, `bringup.launch.py`, `fastlio_mapping.launch.py`, `fast_lio2.launch.py`, `offline_view.launch.py`, `offline_avoidance.launch.py` | DB defaults point to `/data/maps/...` | Defaults come from a robot/site profile, package-share example, or required launch argument; explicit user override still works | Preserve existing `database_path:=...` overrides | Static launch parse; verify default can resolve; no node launch required for Phase 1 | Medium |
| HCP-007, HCP-010 | `ROS_PARAMETER` with package-share/site-profile default | `src/robot_bringup/scripts/path_publisher.py`, `src/robot_bringup/scripts/path_waypoint_sender.py` | ROS parameter defaults point to `/data/maps/db/...` | Node parameters default to empty/required path or package-share example; offline launch supplies resolved value | Existing explicit ROS parameter overrides unchanged | Unit/static parameter check; optional offline launch argument rendering | Medium |
| HCP-023..HCP-029 | `REMOVE_OBSOLETE_PATH` or guarded `ENVIRONMENT_VARIABLE` | seven `src/robot_bringup/launch/*.launch.py` files that set `_torch_lib` | Launches append `/home/dog/.local/lib/python3.10/site-packages/torch/lib` whenever `COLCON_PREFIX_PATH` is set | Stable profile should not append any user-specific Torch path; optional Torch profile may use `RTABMAP_TORCH_LIB_DIR` only when explicitly set | Do not alter command authority, transforms, CAN, or RTAB-Map stable no-Torch defaults | Static launch parse and environment inspection under clean env; no ROS nodes | Low/Medium |
| HCP-030..HCP-035 | `DEFER_OPTIONAL_FEATURE` plus default-safe launch args | `bringup_2d.launch.py`, `bringup_2d_infra.launch.py`, `fastlio_mapping_infra_2d.launch.py` | Default `rtabmap_args` selects SuperPoint and PyMatcher relative paths excluded from Git | Stable Phase 1 defaults must not select unavailable Torch/Python feature paths; optional camera/Torch profile can restore paths by explicit override | Keep explicit `rtabmap_args:=...` override behavior | Static launch argument inspection; confirm stable build remains WITH_TORCH=OFF/WITH_PYTHON=OFF | Medium |

### B. DEFER_OPTIONAL_OR_LATER_FEATURE

These are real paths but not normal Phase 1 production startup blockers.

| Finding IDs | Reason to defer | Future remediation |
|---|---|---|
| HCP-008, HCP-009, HCP-011, HCP-012 | Manual extraction/record/replay helpers only | Replace defaults with required CLI/ROS parameter, package example path, or output under user-selected directory |
| HCP-013..HCP-022 | Replay/offline shell helpers for old bags/workspaces | Convert to required command-line arguments or remove obsolete helpers after preservation policy |
| HCP-036..HCP-038 | Livox LVX playback path is parameterized and live MID360 use is conditional | Use robot profile/default empty LVX file path if playback not selected |
| HCP-043, HCP-044 | YDLIDAR hardware serial device defaults are configurable and hardware-specific | Use robot profile path or documented launch override for the actual serial device |

### C. NO_CHANGE_REQUIRED

| Finding IDs | Reason |
|---|---|
| HCP-039..HCP-041 | Non-normal HAP/mixed Livox launch defaults; parameterized and not current Phase 1 normal startup |
| HCP-042, HCP-045 | `/dev/...` SDK defaults are intentional Linux device interfaces and already parameter-overridable |
| HCP-046..HCP-051 | Build/development probes or ROS install lookup paths, not runtime relocation blockers |
| HCP-052, HCP-053 | plan_nav `/opt/ros` probing is intentional ROS discovery and fails gracefully |
| HCP-054 | `$HOME`-relative venv path is user-local but not hard-coded to `/home/dog` |
| HCP-055, HCP-056 | local tool settings and backup copies are not authoritative runtime startup inputs |
| HCP-057 | generated YDLidar build/install files are not source authority |
| HCP-058 | documentation/evidence paths document prior machine state |
| HCP-059 | binary false positives |

## Future edit constraints

Any later remediation must preserve:

- current command authority and command-chain behavior;
- current CAN protocol and wheelchair controller behavior;
- current static transform values;
- map/database content and plan_nav data files;
- stable OpenCV 4.5.4d profile;
- stable RTAB-Map no-Torch/no-Python build profile;
- legacy fallback behavior unless the fallback is the direct hard-coded-path issue;
- current launch modes unless a path default is the direct issue.

No future edit may start hardware or motion automatically.

## Proposed validation for future remediation

1. Static grep confirms no Phase 1 closure files retain `/home/dog` or
   inappropriate `/data` defaults.
2. Static launch parse confirms all modified launch files construct without
   evaluating missing local paths at parse time.
3. Package install validation builds only the touched package(s), with
   `BUILD_TESTING=OFF`, one worker, and stable Phase 1 underlays.
4. No ROS node, hardware, controller, CAN, UDP, or motion process is started
   for path-parameterization validation unless explicitly authorized in a
   later live phase.

