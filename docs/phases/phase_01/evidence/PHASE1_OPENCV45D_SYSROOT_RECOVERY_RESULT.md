# Phase 1 OpenCV 4.5.4d Isolated Development Sysroot Recovery

Date: 2026-07-31  
Machine: Jetson 2  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Final status

**`PHASE1_OPENCV45D_SYSROOT_RECOVERY_PASS`**

An isolated OpenCV 4.5.4d development sysroot was successfully recovered
from 39 exact-version Ubuntu Jammy `.deb` packages. All packages passed
SHA256 verification. A CMake probe compiled and ran successfully using only
the OpenCV 4.5.4d ABI. RTAB-Map configure-only test passed against the
sysroot with no OpenCV 4.8 or 4.10 contamination.

## Sysroot path

```
/home/dog/phase1_builds/opencv454d_development_sysroot_20260731_035528/
```

## Package recovery summary

| Category | Count | Status |
|---|---|---|
| Development packages | 20 | All SHA256 verified, extracted |
| Runtime packages | 19 | All SHA256 verified, extracted |
| Total .deb files | 39 | 39/39 PASS |
| Extracted files | 627 | Headers, CMake, .so, .a, .pc |

### Package source

- Repository: `http://ports.ubuntu.com/ubuntu-ports jammy/universe`
- Version: `4.5.4+dfsg-9ubuntu4` (all packages)
- Architecture: `arm64`
- Source package: `opencv`
- Origin: Ubuntu
- No NVIDIA, JetPack, or custom packages downloaded

### Development packages (20)

libopencv-dev, libopencv-core-dev, libopencv-imgproc-dev,
libopencv-imgcodecs-dev, libopencv-calib3d-dev, libopencv-contrib-dev,
libopencv-dnn-dev, libopencv-features2d-dev, libopencv-flann-dev,
libopencv-highgui-dev, libopencv-ml-dev, libopencv-objdetect-dev,
libopencv-photo-dev, libopencv-shape-dev, libopencv-stitching-dev,
libopencv-superres-dev, libopencv-video-dev, libopencv-videoio-dev,
libopencv-videostab-dev, libopencv-viz-dev

### Runtime packages (19)

libopencv-calib3d4.5d, libopencv-contrib4.5d, libopencv-core4.5d,
libopencv-dnn4.5d, libopencv-features2d4.5d, libopencv-flann4.5d,
libopencv-highgui4.5d, libopencv-imgcodecs4.5d, libopencv-imgproc4.5d,
libopencv-ml4.5d, libopencv-objdetect4.5d, libopencv-photo4.5d,
libopencv-shape4.5d, libopencv-stitching4.5d, libopencv-superres4.5d,
libopencv-video4.5d, libopencv-videoio4.5d, libopencv-videostab4.5d,
libopencv-viz4.5d

## Extraction method

```bash
dpkg-deb -x <package>.deb sysroot/
```

No `dpkg -i` or `apt install` was used. The dpkg database was not modified.
The system `/usr`, `/usr/local`, and `/opt/ros` directories were unchanged.

## Key sysroot contents

| Item | Path | Status |
|---|---|---|
| Headers | `sysroot/usr/include/opencv4/` | 455 files |
| OpenCVConfig.cmake | `sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4/` | Version 4.5.4 |
| OpenCVModules.cmake | same dir | 54 modules |
| pkg-config | `sysroot/usr/lib/aarch64-linux-gnu/pkgconfig/opencv4.pc` | Version 4.5.4 |
| Shared libs | `sysroot/usr/lib/aarch64-linux-gnu/libopencv_*.so.4.5.4d` | 54 .so files |
| Dev symlinks | `sysroot/usr/lib/aarch64-linux-gnu/libopencv_*.so` | 54 symlinks |
| Static libs | `sysroot/usr/lib/aarch64-linux-gnu/libopencv_*.a` | Present |

## Imported-target path resolution

The Ubuntu OpenCV CMake metadata uses:
```
IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4d"
IMPORTED_SONAME_RELEASE "libopencv_core.so.4.5d"
```

`_IMPORT_PREFIX` is derived from the cmake file directory within the sysroot.
Since the runtime `.so.4.5.4d` files were also extracted into the sysroot,
all imported targets resolve to files within the sysroot. No metadata
patching was required.

## Integrity

- Git: clean (`## main...origin/main`, empty porcelain)
- System OpenCV 4.5.4d libraries: unchanged (mtime 2022-02-02)
- `/usr/local/` OpenCV 4.10: unchanged (mtime 2026-06-18)
- NVIDIA libopencv-dev 4.8: unchanged (mtime 2023-08-29)
- `/opt/ros/humble/lib/libcv_bridge.so`: unchanged
- No package was installed, removed, or registered
- No commit or push occurred
- Existing Phase 1 builds preserved
- Disk free: 19 GiB
