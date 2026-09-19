# R23 motion-aware Collision Monitor geometry prototype

The installed ROS 2 Humble Collision Monitor supports curvature-aware
`approach` prediction, but not modern `velocity_polygon` zones. Its
`footprint_topic` is accepted only for `approach`; STOP/SLOW polygon point
arrays are fixed at configuration time. The Humble dynamic-parameter callback
changes only `enabled`, not polygon points. Therefore stock Humble cannot
express separate curvature-aware STOP and SLOW swept envelopes.

R23 adds a project-owned **shadow-only geometry publisher**. It computes and
publishes review polygons but neither reads obstacles nor modifies commands.
The qualified R22 fixed Collision Monitor remains the sole active safety
filter in both modes.

Modes in the R3H launch:

- `collision_monitor_mode:=fixed_qualified` (default): R22 behavior; the R23
  publisher is absent.
- `collision_monitor_mode:=motion_aware_experimental`: the same fixed R22
  Collision Monitor remains active and the R23 shadow polygons are published
  for sensor-side comparison. This is not authorization for physical motion.

For a fresh, finite, forward command, the generator samples the rear-axle
Ackermann arc using `curvature = angular.z / linear.x`, bounded by the
qualified 1.75 m radius. At every sample it places the padded MK-mini
footprint. The convex hull of those footprints is the zone.

- STOP distance: `min(0.45, 0.10 + |v| * 2.0)` m, with 0.02 m beyond the
  already-qualified 0.03 m footprint padding.
- SLOW distance: `min(0.80, 0.15 + |v| * 5.0)` m, with 0.12 m beyond footprint
  padding.

The SLOW envelope therefore contains STOP and extends farther/laterally.
Stale, absent, nonfinite, zero, reverse, or over-curvature commands select a
conservative fixed fallback: the padded body union the original R22 zones.

Before physical activation, a future task must feed real R22 sensor points to
an independently reviewed enforcement implementation, validate command timing
and stale transitions, and prove STOP/SLOW behavior with MockTransport and
stationary live sensors. R23 itself does not place this prototype in the
command chain.
