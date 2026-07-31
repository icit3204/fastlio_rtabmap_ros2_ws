# Phase 1 Camera and Torch Decision

## Stable Phase 1 profile

The stable Phase 1 RTAB-Map build uses:

- OpenCV 4.10
- `WITH_TORCH=OFF`
- `WITH_PYTHON=OFF`

## What remains supported

This decision does **not** disable or remove:

- camera hardware;
- RealSense or stereo ROS drivers;
- camera image topics;
- camera recording in rosbags;
- camera replay;
- RViz image viewing;
- ordinary OpenCV image processing;
- traditional RTAB-Map visual features such as ORB and GFTT when configured appropriately.

Camera support has not been removed.

## What is temporarily disabled

Only these feature strategies are disabled in this stable RTAB-Map prefix:

- Torch SuperPoint integration inside RTAB-Map;
- PyMatcher, PyDetector, and Python feature strategies.

The decision is temporary and reversible. SuperPoint and Python visual matching are **DEFERRED, NOT ABANDONED**.

## Later Torch-enabled profile

Later work should create a separate RTAB-Map installation with:

- a pinned compatible Torch version;
- matching CUDA and c10 ABI;
- no replacement of the stable LiDAR-centered RTAB-Map prefix.

The later camera validation sequence is:

1. Verify camera topics independently.
2. Test classical OpenCV visual features.
3. Measure CPU, GPU, RAM, latency, and freezing.
4. Build and compare a separate Torch-enabled profile only afterward.

## Multi-sensor data

Multi-sensor bags containing Livox LiDAR, IMU, camera, and single-line LiDAR data remain unchanged and fully usable. Tests may consume only selected topics without deleting, converting, or otherwise altering the other topics.

This decision is recorded in the Phase 1 reports only. No tracked project documentation was modified.
