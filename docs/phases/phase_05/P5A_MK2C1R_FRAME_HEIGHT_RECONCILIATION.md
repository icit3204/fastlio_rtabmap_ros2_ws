# P5A-MK2C1R frame and height reconciliation

## Disposition before live retest

Classification: `B — PRODUCTION_COSTMAP_ITSELF_USES_BODY` (pre-repair).

The production file `src/robot_bringup/config/nav2_common.yaml` configured the
local rolling costmap with `global_frame: body`. The MK2C1 standalone launch
did not override that value: it extracted the complete production
`local_costmap.local_costmap.ros__parameters` mapping and placed the same
mapping under ROS's wildcard node key for the Humble standalone costmap child.
That is exactly why `/costmap/costmap.header.frame_id` was `body` in MK2C1.

Production launch paths `bringup.launch.py`, `bringup_2d.launch.py`, and
`bringup_2d_infra.launch.py` pass `nav2_common.yaml` to Nav2's
`navigation_launch.py`. Their launch-time dictionary changes namespace,
simulation time, autostart, composition, respawn, and log level; it contains no
costmap frame or VoxelLayer override.

## Exact pre-repair production local costmap

- global frame: `body`
- robot base frame: `base_footprint`
- rolling window: `true`
- size: `8 x 8 m`
- resolution: `0.05 m`
- plugins: `nav2_costmap_2d::VoxelLayer`, then
  `nav2_costmap_2d::InflationLayer`
- source name: `lidar_cloud`
- source topic: `/cloud_registered_body`
- explicit sensor frame: none; Nav2 uses `PointCloud2.header.frame_id`
- source type: `PointCloud2`; marking and clearing both `true`
- source height band: `-0.9 .. +0.1 m`
- obstacle range: `0.0 .. 3.0 m`
- raytrace range: `0.0 .. 3.5 m`
- voxel volume: origin Z `-0.9 m`, resolution `0.15 m`, 14 voxels,
  layer maximum obstacle height `+0.1 m`, mark threshold 0
- footprint: `[[0.25,0.10],[0.25,-0.40],[-0.35,-0.40],[-0.35,0.10]]`
- inflation: radius `0.65 m`, cost scaling factor `3.0`

## FAST-LIO body semantics and calibrated TF

FAST-LIO's `body` is its built-in IMU/state frame. It is not a chassis-level
navigation frame. `publish_frame_body()` transforms undistorted LiDAR points
through the configured LiDAR-to-IMU extrinsic and labels the output `body`.
FAST-LIO publishes dynamic `odom -> body` and `/Odometry` with child `body`.

The qualified internal transform is native LiDAR to body/IMU with translation
`[-0.011,-0.02329,+0.04412] m` and identity rotation. The frozen MK1D chassis
calibration establishes `base_link -> livox_frame` as translation
`[+0.128236856109,-0.003242408904,+0.739281661466] m` and RPY
`[+1.109672868,+31.394688552,-112.640194589] deg`.

Their exact recomposition produces the static ROS `body -> base_footprint`
transform `[+0.476208541216,-0.156607656246,-0.662756522534] m`, RPY
`[-29.008923663,+12.613005834,+109.676720772] deg`; MK1D measured a maximum
matrix recomposition error of `1.11e-16`. The large rotation is therefore
expected and is substantially the inverse chassis mounting orientation needed
to convert the IMU/state frame to the REP-103 chassis frame. The calibrated TF
is unchanged by MK2C1R.

## Humble VoxelLayer height semantics

Installed package: `ros-humble-nav2-costmap-2d 1.1.20-1jammy.20260804.213630`.
Its installed `observation_buffer.hpp` describes `global_frame` as the frame
into which PointClouds are transformed. The matching upstream 1.1.20
`ObservationBuffer::bufferCloud()` first calls the TF buffer to transform the
whole cloud into `global_frame_`; it then iterates over Z in that transformed
cloud and retains values between the source's minimum and maximum obstacle
heights. VoxelLayer subsequently receives that already transformed/filtered
observation and applies its layer-level Z bounds in the same costmap-global
frame.

Authoritative implementation:
https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/observation_buffer.cpp

Therefore height limits are costmap-global-frame heights, not untransformed
source-cloud Z heights.

## MK2C1 evidence audit

MK2C1's costmap-cell windows were expressed in the actual `body` costmap frame
by transforming the known base-frame box corners into `body`; they were valid
for the costmap actually recorded. The zero costmap delta relative to clear is
preserved as an observation of that tilted-frame costmap.

The `cloud_points_in_same_xy_and_voxel_height_band` calculation also used raw
`body` points and a `-0.9 .. +0.1 m` Z band. Because the actual MK2C1
costmap-global frame and source frame were both `body`, no point transform was
required in that particular run. It is therefore **not** classified
`MEASUREMENT_FRAME_SEMANTICS_INVALID`. It was valid for the erroneous
body-frame configuration, but it must not be reused after changing the
costmap to `odom_chassis`.

## Minimal repair and retest observer

The first software repair changed the local rolling costmap from `body` to
`odom`. Live preflight then proved that this was insufficient: the observed
`odom -> base_footprint` RPY remained approximately
`[-29.1,+12.7,+109.5] deg`. FAST-LIO's fixed `odom` axes are aligned to its
initial sensor/IMU body, not to the chassis. Calling that frame horizontal
would therefore be incorrect.

No existing horizontal FAST-LIO bridge was present. The minimal production
repair adds a fixed sibling frame, `odom_chassis`, under `odom`. Its frozen
transform is the same qualified initial-body-to-chassis transform used for
`body -> base_footprint`. This is a coordinate-basis change only: it does not
modify FAST-LIO, `/Odometry`, `odom -> body`, or any calibrated sensor edge.
At initialization, `odom_chassis -> base_footprint` is identity apart from
small live estimator initialization offsets; planar chassis motion remains
planar in this basis.

The production local costmap now uses `global_frame: odom_chassis` and retains
`base_footprint` as robot base. Its ground-relative obstacle band is
`+0.10 .. +1.60 m`, with voxel origin `0.0 m`, resolution `0.1 m`, and 16
voxels. Ranges, inflation, source topic, global-costmap
configuration, and all qualified FAST-LIO/Livox calibration are unchanged.

The first live costmap also exposed that its inherited local footprint was a
stale asymmetric non-MK-mini polygon (`x=-0.35..+0.25 m`,
`y=-0.40..+0.10 m`). It left much of the physical chassis outside the
footprint and created extensive self-marking. The local footprint is repaired
to the frozen MK-mini envelope: `x=-0.145..+0.755 m`,
`y=-0.300..+0.300 m`, with the existing `0.03 m` padding retained. The
global-costmap footprint is outside this local-costmap qualification scope and
is unchanged.

`mk2c1r_reconciliation_observer.py` is read-only. For the known 0.595 m and
0.295 m physical-front placements it transforms `/cloud_registered_body` into
the actual `odom_chassis` costmap frame before applying the same Z band, then reports counts
in physical windows alongside lethal cells from the actual Nav2 OccupancyGrid.
It publishes nothing and does not decide whether an obstacle exists; the
qualification authority remains attributable same-pose costmap delta.

## Safe live procedure prepared

Use `mk2c_local_costmap_observation.launch.py start_stack:=true` for the
production-derived standalone VoxelLayer. It launches the calibrated sensor
chain, the real standalone Nav2 costmap, and its lifecycle manager only. It
does not launch controller, planner, BT Navigator, velocity smoother,
Collision Monitor, vehicle adapter, CAN writer, or Twist publisher.

Record one clear baseline with both physical windows, then the upright 430 mm
box at 0.595 m, then at 0.295 m ahead of the physical front edge. Use the new
observer for transformed input/height evidence and actual costmap cells; also
record the actual cloud, costmap, TF, and costmap updates for independent
replay. No 0.15 m placement is planned.
