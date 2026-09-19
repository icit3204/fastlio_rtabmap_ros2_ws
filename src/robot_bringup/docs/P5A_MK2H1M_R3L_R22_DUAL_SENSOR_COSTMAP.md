# R22 dual-sensor Nav2 perception contract

R22 adds MID-360 information to Nav2 without changing FAST-LIO input,
Collision Monitor geometry, the T-mini source, the robot footprint, or the
1.75 m Smac/MPPI turning-radius authority.

## Data flow

```text
/cloud_registered_body (frame: body)
  +-- unchanged --> FAST-LIO/other existing consumers
  +-- TEMPORARY_MKMINI_SELF_BODY_CROP
      /cloud_registered_rtabmap (frame preserved: body)
        +-- unchanged --> RTAB localization/mapping
        +-- MID Nav2 filter (classification in base_footprint)
            +-- /cloud_registered_nav2_obstacles --> Nav2 decaying marking + Collision Monitor
            +-- /cloud_registered_nav2_clearing  --> diagnostic output (not consumed by Nav2)

/scan_raw (T-mini driver)
  +-- project-owned no-return normalizer (invalid/zero -> +Inf)
      /scan
        +-- local Nav2 marking and raytrace clearing
        +-- global Nav2 marking and raytrace clearing
        +-- Collision Monitor
```

The MID obstacle topic contains only finite returns at base-frame height
0.15–1.60 m and planar range 0.10–3.00 m. Its input has already passed the
qualified temporary MK-mini self-body crop. The broader MID clearing topic
contains finite observed returns at height -0.20–2.00 m and range
0.10–3.50 m, but remains diagnostic only. A SpatioTemporalVoxelLayer owns
MID marks and expires unrefreshed voxels after one second. Continuously seen
obstacles refresh at approximately 10 Hz; removed objects disappear without
broad floor/background rays erasing valid marks.

Both outputs preserve the source `body` frame and timestamp. The filter uses
the timestamped `base_footprint <- body` transform only for classification.
Keeping `body` on the output ensures Nav2 ray tracing begins at the real
MID/FAST-LIO sensor origin rather than at the rear-axle ground projection.
Missing TF, unexpected frame, malformed cloud, invalid configuration, and
nonfinite points fail closed on the new Nav2 branch only.

## Costmap contract

Local plugins:

1. `tmini_obstacle_layer` (normal scan marking/raytrace clearing)
2. `mid360_obstacle_layer` (one-second linear voxel decay)
3. `inflation_layer`

Global plugins:

1. `static_layer`
2. `tmini_obstacle_layer` (normal scan marking/raytrace clearing)
3. `mid360_obstacle_layer` (one-second linear voxel decay)
4. `inflation_layer`

Each source owns its dynamic cells: T-mini clears by normal scan raytracing,
while filtered MID cells decay when no longer observed. No manual costmap
clearing service is part of the qualification mechanism.

The normalizer does not change valid finite T-mini ranges, timestamps, or
frames. It makes the driver's zero-valued no-return bins explicit as `+Inf`,
which Nav2 accepts as valid open rays (`inf_is_valid: true`). This behavior is
owned by `robot_bringup`; no modified vendor-driver behavior is required.

Collision Monitor continues to own both physical sensors. Its MID topic is the
same filtered obstacle topic Nav2 consumes; R22 does not alter its polygons,
point thresholds, or action policies.

## Platform boundary

The upstream `TEMPORARY_MKMINI_SELF_BODY_CROP` remains qualified only for the
current MK-mini test platform. It must not transfer automatically to a final
robot.
