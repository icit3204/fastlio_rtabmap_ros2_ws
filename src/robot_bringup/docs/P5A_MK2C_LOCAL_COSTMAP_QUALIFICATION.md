# P5A-MK2C stationary actual-local-costmap procedure

This procedure is intentionally inactive until an operator declares hardware
ready. It launches the production local-costmap parameter block directly from
`robot_bringup/config/nav2_common.yaml`, with no navigation or motion
authority.

## Safe stack

After the hardware checkpoint only, start:

```bash
ros2 launch robot_bringup mk2c_local_costmap_observation.launch.py start_stack:=true
```

It starts MID-360, freshness boundary, FAST-LIO, TF, `nav2_costmap_2d`, and a
lifecycle manager for `local_costmap` only. It starts no controller server,
planner server, BT Navigator, velocity smoother, Collision Monitor, vehicle
adapter, CAN writer, or Twist command publisher.

## Observation evidence

Visual evidence: in RViz add a **Map** display for
`/costmap/costmap` (the raw companion is `/costmap/costmap_raw`), set
its fixed frame to `body`, and add `/cloud_registered_body` as a PointCloud2
display. Keep the robot stationary.

Numerical evidence: after determining the expected box region in the costmap
`body` frame from the live TF transform, run the read-only observer:

```bash
ros2 run robot_bringup mk2c_costmap_observer.py --ros-args \
  -p region_x_min_m:=XMIN -p region_x_max_m:=XMAX \
  -p region_y_min_m:=YMIN -p region_y_max_m:=YMAX
```

It reports received cloud frame/size and occupied (`cost >= 100`) cells in the
specified region. Establish a clear-room baseline first; compare the box test
against that baseline so robot/self artifacts cannot be mistaken for the box.

For floor evidence, run the separate read-only observer only with the clear
floor and a healthy live cloud:

```bash
ros2 run robot_bringup mk2c_floor_return_observer.py --ros-args \
  -p output_prefix:=/tmp/mk2c_floor_return
```

It saves CSV and JSON and classifies the result strictly as
`PROJECT_CLOUD_PRACTICAL_NEAREST_FLOOR_RETURN`, not a universal sensor FOV
result. Its fixed method is: base-footprint TF transform; central corridor
`|y| <= 0.30 m`; x range `0.40..4.00 m`; 0.10 m x bins; robust horizontal
plane fit; `|z-plane| <= 0.05 m`; at least 30 aggregate points in at least
50% of 20 frames.
