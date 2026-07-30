# Third-Party Versions

**Workspace:** `/home/dog/fastlio_rtabmap_ros2_ws`  
**Date:** 2026-07-30  
**Purpose:** Record exact versions of all third-party dependencies for reproducibility.

---

## ROS2 Packages (under `src/`)

### ydlidar_ros2_driver
- **Status:** Excluded from workspace Git (separate upstream repo)
- **Upstream:** https://github.com/YDLIDAR/ydlidar_ros2_driver.git
- **Current commit:** `6b9f249` — 补充文档内容
- **Package version:** 1.0.1 (from `package.xml`)
- **ROS2 package name:** `ydlidar_ros2_driver`
- **Build type:** `ament_cmake`

### YDLidar-SDK
- **Status:** Excluded from workspace Git (separate upstream repo)
- **Upstream:** https://github.com/YDLIDAR/YDLidar-SDK.git
- **Current commit:** `a5a4743` — 修改通用解析无信号强度校验和问题
- **Build type:** CMake C++ library (not a ROS package)

### FAST_LIO_ROS2
- **Status:** Included in workspace Git (custom config, no upstream repo in tree)
- **Package name:** `fast_lio`
- **Package version:** 0.0.0 (from `package.xml`)
- **Build type:** `ament_cmake`
- **Note:** Modified version of FAST-LIO for Livox MID-360. Custom `config/mid360.yaml` includes project-specific calibration.

### livox_ros_driver2
- **Status:** Included in workspace Git (custom MID360_config.json, no `.git` in tree)
- **Package name:** `livox_ros_driver2`
- **Package version:** 1.0.0
- **License:** MIT
- **Build type:** `ament_cmake_auto`

### rtabmap_ros (ROS2 wrapper)
- **Status:** Included in workspace Git (custom launch/config integration, no `.git` in tree)
- **Package name:** `rtabmap_ros` (metapackage)
- **Package version:** 0.23.4
- **Sub-packages:** rtabmap_msgs, rtabmap_launch, rtabmap_slam, rtabmap_sync, rtabmap_util, rtabmap_odom, rtabmap_viz, rtabmap_conversions, rtabmap_demos, rtabmap_examples, rtabmap_python, rtabmap_costmap_plugins, rtabmap_rviz_plugins, rtsp_camera_bridge

### wheelchair_controller
- **Status:** Included in workspace Git (custom project package)
- **Package name:** `wheelchair_controller`
- **Package version:** 0.0.0
- **Build type:** `ament_cmake`
- **Note:** Custom CAN-based wheelchair controller (SocketCAN, Jetson-native).

### robot_bringup
- **Status:** Included in workspace Git (custom project package)
- **Package name:** `robot_bringup`
- **Package version:** 0.1.0
- **Build type:** `ament_cmake`
- **Note:** Custom launch files, Nav2 params, RViz configs for the integrated system.

---

## Standalone Libraries / Source Trees

### RTAB-Map (standalone C++ library)
- **Status:** Excluded from workspace Git (in `third_party/rtabmap-0.23.4/`)
- **Version:** 0.23.4
- **Build system:** CMake
- **Note:** Built separately. Workspace uses this version via `LD_LIBRARY_PATH` in launch files and `scripts/use_rtabmap_0234_env.sh`.

### SuperPoint (Magic Leap, CVPR 2020)
- **Status:** Excluded from workspace Git (`superpoint_superglue/SuperPointPretrainedNetwork/`)
- **Note:** Has nested `.git` but repository is **corrupted** (packfile errors, `bad object HEAD`). Cannot determine exact commit.
- **Weights:** `superpoint_v1.pth` (1.3 MB), `superpoint_v1.pt` (1.3 MB)
- **Reference:** Research @ Magic Leap (CVPR 2020, Oral)

### SuperGlue (Magic Leap, CVPR 2020)
- **Status:** Excluded from workspace Git (`superpoint_superglue/SuperGluePretrainedNetwork/`)
- **Note:** No `.git` directory present. Source distributed as snapshot.
- **Weights:** `superglue_indoor.pth` (46 MB), `superglue_outdoor.pth` (46 MB), `superpoint_v1.pth` (1.3 MB)
- **Reference:** Research @ Magic Leap (CVPR 2020, Oral)
- **License:** Included in `LICENSE` file in repository

---

## Recovery / Restoration Notes

To restore the full build environment:

1. Clone `ydlidar_ros2_driver` at commit `6b9f249`
2. Clone `YDLidar-SDK` at commit `a5a4743`
3. Build RTAB-Map 0.23.4 from source (see `scripts/build_rtabmap_0234.sh`)
4. Download SuperPoint + SuperGlue weights from Magic Leap repositories
5. Place all under workspace root as per original structure

---

*Generated 2026-07-30. Do not modify third-party directories.*
