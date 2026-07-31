# OpenCV 4.5.4d Clean ROS Build

Status: PASS

- ROS root: `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543`
- RTAB-Map prefix: `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459/install`
- OpenCV config: recovered 4.5.4d sysroot
- Build type: Release; `BUILD_TESTING=OFF`
- Build/install/log/tmp/ccache were all external.

The build environment began with `env -i`, sourced only ROS Humble, and set
`RTABMap_DIR`, `OpenCV_DIR`, `CMAKE_PREFIX_PATH`, `LD_LIBRARY_PATH`,
`CMAKE_INCLUDE_PATH`, and `PKG_CONFIG_PATH` directly. `/usr/local/lib` was
not prepended.

The exact colcon form was:

```text
colcon --log-base <root>/log build
  --base-paths /home/dog/fastlio_rtabmap_ros2_ws/src
  --build-base <root>/build --install-base <root>/install --merge-install
  --executor sequential --packages-select <one-package>
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  -DOpenCV_DIR=<sysroot>/usr/lib/aarch64-linux-gnu/cmake/opencv4
  -DRTABMap_DIR=<rtabmap-install>/lib/rtabmap-0.23
```

Recovery used one selected package and one compiler job at a time with
`CMAKE_BUILD_PARALLEL_LEVEL=1` and `MAKEFLAGS=-j1`. The earlier parallel
attempt had one memory-pressure failure; no source/ABI failure occurred.
The interrupted `rtabmap_viz` CMake installation was finalized, then all
remaining packages were completed in dependency order.

Final discovery: 21. Completed: 21. Failed: 0. Aborted: 0. Skipped/not
processed: 0. Twenty packages are ament packages with resource-index entries.
`ydlidar_sdk` declares plain `cmake`, so its successful installation is
represented by its colcon hook, `libydlidar_sdk.a`, CMake package config,
headers, and pkg-config file rather than an ament marker.

OpenCV 4.10 builds remain preserved as experimental evidence. Torch
SuperPoint and Python matchers remain deferred. Camera hardware, camera
topics, and multi-sensor bag data are not removed.
