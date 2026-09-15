# Temporary MK-mini RTAB self-body crop

This feature is qualified only for the current MK-mini research/test platform.
It removes a measured rigid cooler/collar/cover volume from RTAB's dedicated
mapping cloud. It does not modify FAST-LIO or safety perception clouds.

Runtime bounds in `base_footprint` are maintained in
`config/mkmini_rtabmap_self_body_crop.yaml`:

- X: +0.540 to +0.670 m
- Y: -0.140 to +0.150 m
- Z: +0.530 to +0.580 m

The generic RTAB bridge defaults the filter to disabled. Current MK-mini
bringup explicitly opts in. These bounds must never be inherited automatically
by a final robot platform. On platform replacement:

1. disable the crop;
2. inspect and measure final-platform LiDAR self-visibility;
3. leave it disabled if no self-return exists;
4. otherwise qualify a new rigid self-volume and dedicated configuration.

The crop is not a general obstacle-noise filter and must not be expanded into
external free space.
