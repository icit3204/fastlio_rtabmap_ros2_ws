# Phase 1 Interrupted Build Recovery Audit

Audit restart time: 2026-07-31T16:13:33+08:00

## Preserved roots

- Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
- RTAB-Map root: `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459`
- Partial ROS root: `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543`
- OpenCV sysroot: `/home/dog/phase1_builds/opencv454d_development_sysroot_20260731_035528/sysroot`

No stale colcon, CMake, make, ninja, or compiler process was present. Git was
clean, 27 GiB was available, the output locations were writable, and the
available kernel log showed no obvious filesystem or I/O error.

## Stable RTAB-Map revalidation

The existing installation was reused without rebuilding. Its package
configuration reports 0.23.4. `librtabmap_core.so.0.23.4` has no missing
dependency and resolves its OpenCV dependencies exclusively to `.so.4.5d`
files in the recovered sysroot. No `.so.408`, `.so.410`, Torch, libtorch, or
libc10 dependency was found.

## Interrupted ROS state

Colcon still discovers 21 packages. The last fully finalized package was
`rtabmap_slam`. The active package at interruption was `rtabmap_viz`.
Its compile and CMake install commands returned zero and installed its ament
index entry and local setup files, but colcon did not reach final package-hook
generation: `install/share/rtabmap_viz/package.sh` was missing. It is therefore
classified as incomplete, not successful.

The following packages had complete install evidence, package hooks, and
ament index entries before recovery:

`fast_lio`, `livox_ros_driver2`, `rtabmap_conversions`,
`rtabmap_costmap_plugins`, `rtabmap_msgs`, `rtabmap_odom`,
`rtabmap_python`, `rtabmap_rviz_plugins`, `rtabmap_slam`, `rtabmap_sync`,
`rtabmap_util`, `rtsp_camera_bridge`, `wheelchair_controller`, and
`ydlidar_ros2_driver`.

`ydlidar_sdk` had completed its standalone CMake package build and had a
colcon-generated package hook. It is not an ament package and therefore does
not install an ament resource-index marker.

The packages not yet started were `rtabmap_ros`, `rtabmap_launch`,
`rtabmap_demos`, `rtabmap_examples`, and `robot_bringup`.

Recovery begins with `rtabmap_viz`, followed by the unstarted packages in
dependency order. No build directory, partial object, or lock file was
deleted.

Raw evidence is retained in the ROS root under:

- `log/resume_preflight_and_rtab_revalidation.txt`
- `log/resume_package_state.tsv`
- `log/resume_package_audit_raw.txt`

During the inventory, three `colcon list` invocations initially used
colcon's default log base and created three new list-log directories plus a
`COLCON_IGNORE` file under the workspace `log` directory. These four
recovery-generated artifacts were identified by their restart-time
timestamps and moved intact to
`<ROS-root>/log/recovered_workspace_log_artifacts`. No pre-existing
workspace log file was removed or changed; final inspection found no
restart-time file remaining in the original workspace log.
