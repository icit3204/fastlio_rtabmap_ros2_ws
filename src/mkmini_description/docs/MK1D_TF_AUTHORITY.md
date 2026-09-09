# MK1D MID-360 TF authority (historical)

Status: `SUPERSEDED_BY_P5A_MK2C1R3_FOR_CURRENT_HARDWARE`

The former MK1D fixed edge was:

```text
base_link -> livox_frame
translation:  0.128236856109 -0.003242408904 0.739281661466 m
RPY:          0.019367445165 0.547940682865 -1.965942265662 rad
```

Its yaw is retained for historical provenance only and has status
`RETIRED_FOR_CURRENT_HARDWARE_FRAME_CONTRACT`. Current physical-front target
recordings contradict it. The active authority is
`config/mkmini_platform_profile.yaml`, status
`MKMINI_MID360_CURRENT_YAW_AUTHORITY_MK2C1R3`.

The rotation is `Rz(yaw) * Ry(pitch) * Rx(roll)`. Translation X/Y derives
from the operator-confirmed mast/sensor centerline and the official 47 mm
native-origin offset. Translation Z uses the supervisor-accepted midpoint of
two stationary floor captures: 0.855073 m and 0.863490323 m from the native
origin to the floor, giving 0.859281661 m above floor and 0.739281661 m above
`base_link`.

The bounded engineering uncertainty is +/-0.004209 m for the floor midpoint,
approximately +/-0.001 m for the lateral physical reference, and approximately
+/-0.84 degrees plane-fit plus +/-1.0 degree operator target alignment. This
is not represented as a formal covariance.

FAST-LIO owns dynamic `odom -> body`. Its unchanged
`extrinsic_T=[-0.011,-0.02329,+0.04412]` maps native LiDAR coordinates to the
IMU/state `body` frame. The MK-mini localization launch now derives static
`body -> base_footprint` and `odom -> odom_chassis` from the active profile and
renders the robot-state-publisher description from a nonnumeric template. No
CAN, T-mini, stereo, Nav2, Gate, or operational footprint is enabled by this
profile.
