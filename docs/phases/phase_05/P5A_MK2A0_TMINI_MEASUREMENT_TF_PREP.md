# P5A-MK2A0R T-MINI final measurement and TF freeze

**Status:** `P5A_MK2A0R_TMINI_FINAL_TF_MEASUREMENT_FREEZE_PASS`

This record uses the MK-mini frame authority:

- `base_footprint`: rear axle midpoint projected to ground;
- `base_link`: rear axle midpoint at axle-center height;
- `base_footprint -> base_link`: `[0, 0, +0.120] m`.

## Frozen measurements

| Field | Value | Status |
|---|---:|---|
| `TMINI_CENTERLINE_X_FROM_REAR_AXLE_M` | `+0.703` m | measured/frozen |
| `TMINI_CENTERLINE_Y_FROM_ROBOT_CENTER_M` | `0.000` m | measured/frozen |
| `TMINI_FORWARD_OF_FRONT_AXLE_M` | `+0.103` m | measured/frozen |
| `TMINI_BEHIND_FRONT_EDGE_M` | `0.052` m | measured/frozen |
| `REAR_AXLE_TO_FRONT_EDGE_M` | `0.755` m | measured/frozen |
| `TMINI_HOUSING_BOTTOM_HEIGHT_FROM_FLOOR_M` | `0.286` m | measured; housing bottom only |
| `TMINI_SCAN_PLANE_HEIGHT_FROM_FLOOR_M` | `0.3123` m | measured/frozen |
| `BASE_LINK_TO_TMINI_SCAN_PLANE_Z_M` | `+0.1923` m | measured/frozen |

The manufacturer optical drawing places the laser/optical centerline
`0.0263 m` above the housing bottom: `0.2860 + 0.0263 = 0.3123 m`.
Relative to `base_link` at `0.1200 m`, the frozen TF Z is
`0.3123 - 0.1200 = +0.1923 m`. No diagonal measurement is inferred.

## Cross-checks

- `0.600 + 0.103 = 0.703` — **PASS**
- `0.703 + 0.052 = 0.755` — **PASS**
- `0.2860 + 0.0263 = 0.3123` — **PASS**
- `0.3123 - 0.1200 = 0.1923` — **PASS**

## Prepared, inactive interface

Candidate transform:

```text
base_link -> laser_frame
x = +0.703 m
y =  0.000 m
z = +0.1923 m
roll/pitch/yaw = 0 / 0 / 0 deg nominal
```

The manufacturer defines zero-angle as the top-arrow direction viewed from
above. The operator aligned that mark with robot forward `+X`, so
`TMINI_ZERO_MARK_DIRECTION=ROBOT_FORWARD_PLUS_X` and
`TMINI_NOMINAL_YAW_DEG=0.0`. The manufacturer individual zero-angle
tolerance is approximately `+/-3 degrees`; it is recorded as a tolerance,
not converted into a TF correction. Actual `/scan` orientation remains a
future live verification.

The expected future message type is `sensor_msgs/msg/LaserScan` and the
conceptual frame is `laser_frame`. TF preparation is disabled by default and
the preparation launch contains no T-MINI driver or serial-device access.

Before any live qualification, resolve a stable USB identity through
`/dev/serial/by-id` or equivalent; no `ttyUSB0` assumption is authorized.

Mount observations: rigid `YES`; front obstruction `NO`; left obstruction
`NO`; right obstruction `NO`; rear obstruction `YES_COOLER_BODY`; wheels in
scan plane `NO`; cable secured `YES`. T-MINI USB remains disconnected.
