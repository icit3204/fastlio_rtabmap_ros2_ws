# OpenCV 4.5.4d Runtime and Synthetic Image Flow

Status: PASS

Environment: clean ROS Humble environment, isolated ROS install, stable
RTAB-Map prefix, recovered OpenCV sysroot, `ROS_LOCALHOST_ONLY=1`, and
`ROS_DOMAIN_ID=193`.

Only the isolated `rtabmap_sync/rgb_sync` executable was started. Inputs and
output were remapped below `/phase1_single_abi_test/`. It remained idle for
approximately 10 seconds, `/proc/<PID>/maps` was captured, and it exited
cleanly on SIGINT with code 0.

The map contains cv_bridge, image transport, RTAB-Map core/conversions/sync
libraries, and OpenCV real files `.so.4.5.4d` from the recovered sysroot.
It contains no OpenCV 4.8 or 4.10 library. There was no unresolved symbol or
segmentation fault. Sandbox-only UDP errors from an earlier restricted run
were excluded from the definitive localhost flow run.

For the message test, a temporary no-cv2 Python publisher constructed
`sensor_msgs/Image` and `CameraInfo` directly. It sent 12 matching timestamp
pairs. The subscriber received 12 `rtabmap_msgs/RGBDImage` outputs; all 12
preserved width 4, height 3, encoding `bgr8`, step 12, and deterministic byte
content. `rgb_sync` then exited cleanly. No physical camera, bag, driver,
navigation node, controller, CAN interface, or motion topic was used.

This proves synthetic no-hardware message compatibility only, not physical
camera validation.

Evidence is under the ROS root in `log/runtime_idle` and
`log/synthetic_flow`.
