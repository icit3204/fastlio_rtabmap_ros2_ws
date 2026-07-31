# OpenCV 4.5.4d RTAB-Map Production Build

Status: PASS

- Source: `/home/dog/fastlio_rtabmap_ros2_ws/third_party/rtabmap-0.23.4`
- External root: `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459`
- Install: `/home/dog/phase1_builds/rtabmap_0234_opencv454d_no_torch_20260731_042459/install`
- OpenCV config: `/home/dog/phase1_builds/opencv454d_development_sysroot_20260731_035528/sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4`
- Configuration: Release, `WITH_TORCH=OFF`, `WITH_PYTHON=OFF`,
  `BUILD_APP=OFF`, `BUILD_TOOLS=OFF`, `BUILD_EXAMPLES=OFF`
- Useful dependencies retained: PCL, VTK, TBB, g2o, GTSAM, OctoMap,
  pointmatcher, SQLite3, TORO, VERTIGO, and Madgwick.
- Configure: exit 0, 10 seconds
- Build: `cmake --build <build> --parallel 2`, exit 0, 1727 seconds
- Install: `cmake --install <build>`, exit 0; installed size 25 MiB

The configure gate proved OpenCV 4.5.4, sysroot-only imported library
locations, and `.so.4.5d` SONAMEs. It found no 4.8 or 4.10 configuration
path. RTAB-Map reports version 0.23.4. `readelf` and `ldd` show no missing
library, `.so.408`, `.so.410`, Torch, libtorch, or libc10 dependency.

The Ubuntu OpenCV set has no xfeatures2d/nonfree component; this is recorded
as unavailable rather than enabled. The OpenCV 4.10 builds remain preserved
unchanged as experimental evidence. OpenCV 4.5.4d is the stable ROS Humble
compatibility candidate.

Complete commands and output are retained in the external root's `log`
directory, including `configure.log`, `build.log`, `install.log`,
`configuration_gate.txt`, and `abi_validation.txt`.
