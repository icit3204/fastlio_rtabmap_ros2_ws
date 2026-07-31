# OpenCV 4.5.4d Static ABI Verification

Status: PASS

The complete resolved dependency closures were inspected for:

- `/opt/ros/humble/lib/libcv_bridge.so`
- stable `librtabmap_core.so.0.23.4`
- stable `librtabmap_conversions.so`
- stable `librtabmap_sync.so`
- stable `librtabmap_sync_plugins.so`
- stable `rtabmap_sync/rgb_sync`
- stable `rtabmap_slam/rtabmap`

All OpenCV DT_NEEDED entries use the `.so.4.5d` SONAME family and resolve
to the recovered sysroot. The corresponding real filenames are
`.so.4.5.4d`. There is no OpenCV 4.8, OpenCV 4.10, `.so.408`, `.so.410`,
missing library, or mixed OpenCV ABI family.

The full `readelf` and `ldd` evidence is:

`/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543/log/static_abi_complete.txt`
