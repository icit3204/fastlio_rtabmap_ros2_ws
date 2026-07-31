# Phase 1 — Native Environment and Build Verification

Date: 2026-07-31  
Machine: Jetson 2 (NVIDIA Jetson Orin, aarch64)  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Final status

**`PHASE1_SINGLE_ABI_NATIVE_BUILD_PASS`**

---

## 1. Phase objective

Prove that the authoritative source can be rebuilt natively and reproducibly
on Jetson 2 without relying on copied workspace build/install/log artifacts.
The build must link exclusively against a single OpenCV ABI family (`4.5d`)
matching the ROS Humble system `cv_bridge`.

## 2. Machine and environment

| Attribute | Value |
|---|---|
| OS | Ubuntu 22.04 (Jammy) |
| ROS | ROS 2 Humble (`/opt/ros/humble`) |
| Architecture | aarch64 (ARM64) |
| Kernel | Linux 5.15.185-tegra |
| Compiler | GCC 11.4.0 |
| CMake | 3.22.1 |
| colcon | ROS Humble default |
| JetPack | 6.2.2 (r36.5) |

## 3. Package inventory

From the production ROS build against OpenCV 4.5.4d:

- **Discovered**: 21 packages
- **Installed and verified**: 21
- **Missing**: 0
- **Failed**: 0
- **Incomplete**: 0

All 21 packages built successfully:

```
fast_lio                  livox_ros_driver2         robot_bringup
rtabmap_conversions       rtabmap_costmap_plugins   rtabmap_demos
rtabmap_examples          rtabmap_launch            rtabmap_msgs
rtabmap_odom              rtabmap_python            rtabmap_ros
rtabmap_rviz_plugins      rtabmap_slam              rtabmap_sync
rtabmap_util              rtabmap_viz               rtsp_camera_bridge
wheelchair_controller     ydlidar_ros2_driver       ydlidar_sdk
```

`ydlidar_sdk` is a valid plain CMake package (not an ament package). It is
verified through its colcon hook, static library (`libydlidar_sdk.a`),
CMake package config, headers, and pkg-config metadata. The other 20
packages are ament packages with resource-index entries.

## 4. Stable build architecture

```
OpenCV 4.5.4d isolated sysroot
  └─> RTAB-Map 0.23.4 (Release, no Torch, no Python)
        └─> rtabmap_ros and all workspace packages
              └─> Single OpenCV ABI (4.5d) with ROS Humble cv_bridge
```

The build uses a clean-room method: `env -i` with only ROS Humble sourced,
explicit `OpenCV_DIR`, `RTABMap_DIR`, `CMAKE_PREFIX_PATH`, and
`LD_LIBRARY_PATH`. `/usr/local/lib` is not prepended, preventing accidental
OpenCV 4.10 linkage.

## 5. Stable paths

| Artifact | Path |
|---|---|
| OpenCV 4.5.4d sysroot | `/home/dog/phase1_builds/opencv454d_development_sysroot_20260731_035528/sysroot` |
| RTAB-Map install | `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459/install` |
| ROS install | `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543/install` |

### OpenCV sysroot provenance

The sysroot was recovered from 39 exact-version Ubuntu Jammy `.deb` packages
(`4.5.4+dfsg-9ubuntu4`, arm64). All SHA256 checksums were verified against
apt metadata. The sysroot contains 455 headers, 54 `.so.4.5.4d` runtime
libraries, CMake configuration, pkg-config metadata, and development
symlinks.

See: `evidence/PHASE1_OPENCV45D_SYSROOT_RECOVERY_RESULT.md`

## 6. RTAB-Map configuration

| Setting | Value |
|---|---|
| Build type | Release |
| OpenCV | 4.5.4d (sysroot) |
| `WITH_TORCH` | OFF |
| `WITH_PYTHON` | OFF |
| `BUILD_APP` | OFF |
| `BUILD_TOOLS` | OFF |
| `BUILD_EXAMPLES` | OFF |
| `BUILD_SHARED_LIBS` | ON |

**Retained dependencies**: PCL 1.12.1, VTK, TBB, g2o, GTSAM, OctoMap,
libpointmatcher 1.3.1, SQLite3, TORO, VERTIGO, Madgwick, OpenNI/OpenNI2,
RealSense2 2.58.2, FLANN, Eigen 3.4.0, OpenMP 4.5, Boost 1.74.0.

Configure: exit 0, ~10 seconds.
Build: `cmake --build <build> --parallel 2`, exit 0, ~1727 seconds (~29 min).
Installed size: 25 MiB.

See: `evidence/PHASE1_OPENCV454D_RTABMAP_PRODUCTION_BUILD.md`

## 7. ABI evidence

- OpenCV 4.5.4d only (SONAME `libopencv_*.so.4.5d`)
- No OpenCV 4.8 (no `.so.408`)
- No OpenCV 4.10 (no `.so.410`)
- No Torch, libtorch, or libc10
- No missing libraries

Validated across:
- `libcv_bridge.so` (ROS Humble system)
- `librtabmap_core.so.0.23.4`
- `librtabmap_conversions.so`
- `librtabmap_sync.so`
- `librtabmap_sync_plugins.so`
- `rtabmap_sync/rgb_sync`
- `rtabmap_slam/rtabmap`

All OpenCV DT_NEEDED entries use the `.so.4.5d` SONAME family. The `ldd`
resolution chain completes with zero missing libraries.

See: `evidence/PHASE1_OPENCV454D_STATIC_ABI_VERIFICATION.md`

## 8. Runtime validation

- `rgb_sync` started and exited cleanly (SIGINT, exit code 0)
- `/proc/<PID>/maps` contained one OpenCV ABI family (`.so.4.5.4d`)
- No OpenCV 4.8 or 4.10 library mapped
- No unresolved symbol or segmentation fault
- Synthetic no-cv2 Python publisher sent 12 timestamp-matched
  `sensor_msgs/Image` + `CameraInfo` pairs
- Subscriber received 12 `rtabmap_msgs/RGBDImage` outputs
- All 12 preserved width 4, height 3, encoding `bgr8`, step 12,
  and deterministic byte content

**This is no-hardware compatibility evidence only — it does not constitute
physical camera validation.**

See: `evidence/PHASE1_OPENCV454D_RUNTIME_AND_IMAGE_FLOW.md`

## 9. Build recovery history

The initial parallel `colcon build` encountered RAM/swap exhaustion on this
8 GB Jetson. No source or ABI error was involved. The build was resumed
package-by-package with `CMAKE_BUILD_PARALLEL_LEVEL=1` and `MAKEFLAGS=-j1`.
All 21 packages completed. Power interruption recovery was verified safely:
the `rtabmap_viz` CMake installation was finalized, then all remaining
packages were completed in dependency order. No workspace source was
modified during recovery.

See: `evidence/PHASE1_INTERRUPTED_BUILD_RECOVERY_AUDIT.md`

## 10. Camera and Torch decision

- Camera support has **not** been removed
- Camera topics, recording, replay, RViz, and ordinary OpenCV remain
- Classical RTAB-Map features (ORB, GFTT, etc.) remain available
- Torch SuperPoint and Python matchers are **deferred, not abandoned**
- A separate pinned Torch-enabled profile may be developed later
- Multi-sensor bags remain unchanged and fully useful

See: `evidence/PHASE1_CAMERA_AND_TORCH_DECISION.md`

## 11. Stable camera/default compatibility

Three authoritative launch files previously selected SuperPoint/PyMatcher
defaults unavailable in the stable no-Torch/no-Python build:

- `src/robot_bringup/launch/fastlio_mapping_infra_2d.launch.py`
- `src/robot_bringup/launch/bringup_2d.launch.py`
- `src/robot_bringup/launch/bringup_2d_infra.launch.py`

These stable defaults are now corrected by the hard-coded runtime-path
remediation. The stable profile no longer selects SuperPoint detector
strategy, SuperPoint model paths, PyMatcher scripts/models, or Python/CUDA
matcher settings by default. Explicit `rtabmap_args:=...` overrides remain
supported for a future optional Torch/Python profile. This does not
constitute physical camera validation.

See: `evidence/PHASE1_OPENCV454D_FEATURE_AVAILABILITY_AUDIT.md`
See also: `evidence/PHASE1_HARD_CODED_PATH_REMEDIATION_RESULT.md`

## 12. Preserved experimental profile

The earlier OpenCV 4.10 RTAB-Map and ROS builds remain preserved as
experimental evidence:

- `/home/dog/phase1_builds/rtabmap_0234_opencv410_no_torch_20260731_003847/install`
- `/home/dog/phase1_builds/clean_build_rtabmap_fixed_20260731_020041/install`

They are not the stable ROS Humble profile because they loaded two OpenCV
ABI families (`.so.410` from RTAB-Map and `.so.4.5d` from system cv_bridge)
in a single process.

## 13. Integrity and safety

- No tracked source changed during build validation
- Git remained clean throughout
- Original workspace build/install/log were preserved
- No hardware driver was launched
- No CAN access
- No wheelchair controller
- No Nav2
- No motion command
- No package installation or removal
- `/opt/ros/humble` unchanged
- `/usr/local` unchanged

## 14. Phase 1 acceptance decision

**Phase 1 native environment and single-ABI build verification are complete
and frozen at tag `phase1_native_build_verified`.** Broader V3.1 Phase 1
authority closure remains open for the final independent authority-closure
audit. Static command-authority reconciliation, the initial unified RViz
profile, Collision Monitor Humble capability/schema auditing, and active
runtime hard-coded path parameterization are now recorded as complete Phase 1
closure items. Physical
body-to-base_footprint z-offset validation remains deferred to the
live-sensor phase as required by the authority.

The authoritative source builds natively against a single OpenCV 4.5.4d ABI
matching the ROS Humble system cv_bridge. All 21 packages compile and
install. The runtime probe confirms single-ABI operation in `/proc/<PID>/maps`.
Synthetic image flow through cv_bridge preserves message dimensions,
encoding, and content.

See: `evidence/PHASE1_AUTHORITY_COMPLIANCE_AUDIT.md` for the full V3.1
authority requirement matrix and classification of every Phase 1 task.

## 15. Phase 1 Authority Closure Status

### Completed

- clean native build (21/21 packages);
- RTAB-Map 0.23.4 (Release, OpenCV 4.5.4d, no Torch, no Python);
- 21/21 packages installed and verified with single OpenCV ABI;
- OpenCV 4.5.4d single ABI (no 4.8, no 4.10, no `.so.408`, no `.so.410`);
- synthetic cv_bridge image flow (12/12 RGBDImage outputs verified);
- no hardware or motion activity during build validation;
- module registry created (43 modules, all required fields);
- calibration manifest created with unknown/conflicting values explicit;
- clock/timestamp policy created (derived from V3.1 §10A);
- static command-authority reconciliation complete;
- Collision Monitor Humble capability/schema audit complete;
- governance artifact discovery audit complete;
- unified initial RViz profile complete:
  `src/robot_bringup/config/phase1_authority_baseline.rviz` exists,
  YAML/plugin validation passed, package installation validation passed, and
  the profile contains no machine-specific path;
- hard-coded runtime-path parameterization complete:
  all 21 `MUST_FIX_FOR_PHASE1_CLOSURE` findings are resolved, explicit
  overrides remain functional, stable launch defaults no longer select
  Torch/Python-only features, and package build/install validation passed.

### Still open

- final independent Phase 1 authority-closure audit.

### Deferred by authority

- physical `body → base_footprint` z-offset verification during the live
  sensor phase;
- physical sensor timing/skew measurement;
- physical camera validation;
- optional Torch-enabled visual profile;
- replay/manual helper path cleanup;
- optional LVX playback path cleanup;
- YDLIDAR device-path parameterization beyond current hardware defaults;
- historical, backup, generated, and documentation-only path cleanup;
- future physical calibration;
- runtime RViz GUI smoke test;
- live perception and motion validation.

**Phase 2 must not begin until the non-hardware Phase 1 authority-closure
items are resolved or formally reclassified by an updated authority.**

## 16. Deferred work

- Physical camera validation
- Optional separate Torch-enabled profile
- Build reproducible startup/environment tooling
- Cleanup/archive of failed and experimental build roots (only after
  preservation policy is approved)

---

## Evidence index

See: `evidence/PHASE_01_EVIDENCE_INDEX.md`
