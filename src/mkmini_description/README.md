# mkmini_description

Project-owned MK-mini frame authority, superseded for current MID-360 yaw by
P5A-MK2C1R3.

This package intentionally contains only the rear-axle-based project frames:

- `base_footprint`: rear-axle midpoint projected onto the ground;
- `base_link`: rear-axle midpoint at nominal axle-centre height;
- `rear_axle`: coincident with `base_link`;
- `front_axle`: nominal wheelbase offset of `0.600 m`;
- `livox_frame`: current native MID-360 frame fixed to `base_link`.

It has no chassis visual mesh, collision footprint, T-mini, stereo, CAN, or
runtime sensor implementation. The profile records the operator worksheet,
manufacturer values, the historical C2R record, and the current-hardware
MK2C1R3 yaw authority with explicit authority/status labels.

The operator worksheet used is the supplied v3 revision:
`/home/dog/Downloads/PHASE5A_MK1B_CLEAN_MEASUREMENT_WORKSHEET_20260830_v3.md`.
It is recorded as the input because the exact v2 filename was not present and
the v3 document identifies itself as task P5A-MK1B.

The single active numeric source is `config/mkmini_platform_profile.yaml`.
The robot description is rendered from `urdf/mkmini_frames.urdf.template`,
which intentionally contains no MID-360 numeric transform. The launch derives
body/base and chassis-odom transforms from the same profile. The old
-112.640195-degree yaw is historical only. FAST-LIO's internal sensor
extrinsic remains unchanged and is cross-checked against the profile contract.

The final operational footprint and Collision Monitor polygons remain pending.
Manufacturer nominal `900 x 600 mm` dimensions are metadata, not a footprint.
Brake/encoder authority remains a separate blocker for real CAN and wheel
odometry only. The calibrated TF does not authorize CAN or motion.
