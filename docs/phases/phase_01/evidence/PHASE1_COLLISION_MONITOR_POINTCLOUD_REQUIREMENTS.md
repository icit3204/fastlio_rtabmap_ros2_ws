# Phase 1 Collision Monitor PointCloud2 Requirements

Date: 2026-07-31
Machine: Jetson 2

## V3.1 Authority PointCloud2 requirement

The V3.1 authority (Section 9.6, Phase 5-6) specifies a temporary
raw-MID-360 Collision Monitor profile using PointCloud2 as an observation
source. It requires verifying that the installed Humble version supports
the `PointCloud2` source type and the exact required configuration schema.

## PointCloud2 support: CONFIRMED

| Check | Result |
|---|---|
| `pointcloud.hpp` exists | YES (`/opt/ros/humble/include/nav2_collision_monitor/pointcloud.hpp`) |
| Class `PointCloud : public Source` | YES |
| Subscribes `sensor_msgs::msg::PointCloud2` | YES |
| Height filtering (`min_height_`, `max_height_`) | YES |
| Default YAML includes `pointcloud` example | YES (`type: "pointcloud"`) |

## PointCloud2 parameter requirements for project use

### Required by installed schema

| Parameter | Required | Project value (bringup.launch.py) | Notes |
|---|---|---|---|
| `type` | YES | `"pointcloud"` | Must be exactly `"pointcloud"` |
| `topic` | YES | `"/cloud_registered_body"` | Must be valid PointCloud2 topic |
| `enabled` | NO | `true` | Source active |

### Optional by installed schema

| Parameter | Project value | Purpose |
|---|---|---|
| `min_height` | `0.05` | Filter floor points below 5cm from base_footprint |
| `max_height` | `1.80` | Filter overhead points above 1.8m |

### Inherited from node-level

| Parameter | Project value |
|---|---|
| `base_frame_id` | `"base_footprint"` |
| `odom_frame_id` | `"odom"` |
| `transform_tolerance` | `0.3` |
| `source_timeout` | `1.0` |
| `base_shift_correction` | `true` |
| `observation_sources` | `["pointcloud"]` |

## TF requirements for `/cloud_registered_body`

The PointCloud2 source transforms incoming points from their frame to
`base_frame_id` using the TF buffer. This requires:

1. `/cloud_registered_body` messages must have a valid `header.frame_id`
   (expected: `"body"`)
2. TF tree must contain `body → base_footprint` transform
3. Transform must arrive within `transform_tolerance: 0.3` seconds
4. `odom_frame_id: "odom"` is used as the global frame for time interpolation

## V3.1 authority PointCloud2 profile comparison

The authority specifies Phase 5-6 temporary profile with:
- Source: MID-360 raw `/cloud_registered_body`
- Collision Monitor configured with PointCloud2 observation source

The current `bringup.launch.py` configuration matches this requirement:
- `observation_sources: ["pointcloud"]`
- `pointcloud.type: "pointcloud"`
- `pointcloud.topic: "/cloud_registered_body"`

The `bringup_2d.launch.py` has a DIFFERENT PointCloud2 intent (floor-self
filtering with `min_height: -1.1, max_height: 0.1`) but it is COMMENTED OUT.

## PointCloud2 feature support matrix

| Feature | Supported in Humble 1.1.20? | Evidence |
|---|---|---|
| PointCloud2 subscription | YES | `pointcloud.hpp:data_sub_` |
| TF transformation to base_frame | YES | `source.hpp` (inherited) |
| min_height filtering | YES | `pointcloud.hpp:min_height_` |
| max_height filtering | YES | `pointcloud.hpp:max_height_` |
| source_timeout staleness | YES | `source.hpp:sourceValid()` |
| base_shift_correction | YES | `source.hpp:base_shift_correction_` |
| Obstacle point-count threshold (polygon) | YES | `polygon.hpp:max_points` |
| Multiple simultaneous PointCloud2 sources | YES | `observation_sources` list |
| PointCloud2 QoS configuration | Limited | Uses default subscriber QoS |
| min_points per polygon | Partial | Not in Humble 1.1.20 — only `max_points` |
| Obstacle radius/dilation | NO | Not in Humble 1.1.20 |
| Cloud voxel downsampling | NO | Not in Humble 1.1.20 |
| PointCloud2 intensity filtering | NO | Not in Humble 1.1.20 |

## Required runtime validation (deferred to later phase)

Per V3.1 authority Phase 5-6 requirements, the following must be validated
at runtime (no-motion test) before physical navigation:

1. PointCloud2 data arrives at expected rate from `/cloud_registered_body`
2. `body → base_footprint` TF is published at correct rate
3. Height filtering with `min_height: 0.05` does not create false stops
4. Robot-body self-points do not trigger unintended stops
5. `source_timeout: 1.0s` is adequate for MID-360 cloud rate
6. Collision Monitor polygon STOP/SLOWDOWN behavior under real data
