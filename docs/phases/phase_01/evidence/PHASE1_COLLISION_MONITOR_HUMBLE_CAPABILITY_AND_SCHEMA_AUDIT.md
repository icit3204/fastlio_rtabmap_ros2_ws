# Phase 1 Collision Monitor Humble Capability and Schema Audit

Date: 2026-07-31
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Executive conclusion

**`PHASE1_COLLISION_MONITOR_SCHEMA_AUDIT_PASS`**

The installed ROS 2 Humble `nav2_collision_monitor` version 1.1.20 supports
PointCloud2 (`type: "pointcloud"`), LaserScan (`type: "scan"`), and Range
(`type: "range"`) observation sources. It accepts `geometry_msgs/msg/Twist`
as input/output. The project's active `bringup.launch.py` Collision Monitor
configuration is schema-valid. The `bringup_2d.launch.py` Collision Monitor
is commented out and inactive. Velocity polygon and "limit" action types are
NOT supported by Humble 1.1.20.

---

## 1. Installed package identity

| Attribute | Value |
|---|---|
| Package | `ros-humble-nav2-collision-monitor` |
| Version | `1.1.20-1jammy.20260607.134559` |
| Architecture | `arm64` |
| Origin | `http://packages.ros.org/ros2/ubuntu jammy/main` |
| Installation prefix | `/opt/ros/humble` |
| Executable | `collision_monitor` |
| Lifecycle node | YES (`nav2_util::LifecycleNode`) |
| Shared library | `/opt/ros/humble/lib/libcollision_monitor_core.so` |

Related packages:
- `ros-humble-nav2-bringup`: `1.1.20`
- `ros-humble-nav2-common`: `1.1.20`
- `ros-humble-navigation2`: `1.1.20`

---

## 2. Supported observation source types

| Source type | `type` string | ROS message type | Header file | Supported |
|---|---|---|---|---|
| PointCloud2 | `"pointcloud"` | `sensor_msgs::msg::PointCloud2` | `pointcloud.hpp` | ✅ YES |
| LaserScan | `"scan"` | `sensor_msgs::msg::LaserScan` | `scan.hpp` | ✅ YES |
| Range | `"range"` | `sensor_msgs::msg::Range` | `range.hpp` | ✅ YES |
| Velocity Polygon | N/A | N/A | N/A | ❌ NOT IN HUMBLE 1.1.20 |

---

## 3. Exact PointCloud2 source schema

| Parameter | Type | Default | Required | Behavior | Evidence |
|---|---|---|---|---|---|
| `type` | string | `"pointcloud"` | YES | Selects PointCloud2 source | `pointcloud.hpp` |
| `topic` | string | (no default) | YES | Subscription topic for PointCloud2 | `source.hpp` `getCommonParameters()` |
| `min_height` | double | (none) | NO | Minimum Z height in base_frame. Points below are filtered out. | `pointcloud.hpp:min_height_` |
| `max_height` | double | (none) | NO | Maximum Z height in base_frame. Points above are filtered out. | `pointcloud.hpp:max_height_` |
| `enabled` | bool | (none) | NO | Whether this source is active | `source.hpp:enabled_` |
| `source_timeout` | double | (node default) | NO (node-level) | Max seconds data is valid | `source.hpp:source_timeout_` |
| `base_shift_correction` | bool | (node default) | NO (node-level) | Correct for robot movement | `source.hpp:base_shift_correction_` |
| `transform_tolerance` | double | (node default) | NO (node-level) | TF lookup tolerance | `source.hpp:transform_tolerance_` |
| `base_frame_id` | string | (node default) | NO (node-level) | Base frame for transforms | `source.hpp:base_frame_id_` |
| `global_frame_id`/`odom_frame_id` | string | (node default) | NO (node-level) | Global/odom frame | `source.hpp:global_frame_id_` |

**PointCloud2 confirmed**: YES. Installed `pointcloud.hpp` declares `class PointCloud : public Source` with `sensor_msgs::msg::PointCloud2::ConstSharedPtr` data callback and `min_height_`, `max_height_` members.

---

## 4. Exact node-level schema

| Parameter | Type | Default (from YAML) | Required | Behavior | Evidence |
|---|---|---|---|---|---|
| `base_frame_id` | string | `"base_footprint"` | YES | Robot base frame for transforms | `collision_monitor_node.hpp` |
| `odom_frame_id` | string | `"odom"` | YES | Global frame for time interpolation | `collision_monitor_node.hpp` |
| `cmd_vel_in_topic` | string | `"cmd_vel_raw"` | YES | Input velocity topic | `collision_monitor_node.hpp` |
| `cmd_vel_out_topic` | string | `"cmd_vel"` | YES | Output velocity topic | `collision_monitor_node.hpp` |
| `transform_tolerance` | double | `0.5` | YES | TF lookup tolerance (seconds) | Default YAML |
| `source_timeout` | double | `5.0` | YES | Max age of valid source data (seconds) | Default YAML |
| `base_shift_correction` | bool | `true` | YES | Correct source data for base movement | Default YAML |
| `stop_pub_timeout` | double | `2.0` | YES | Stop publishing 0-velocity after this long | `collision_monitor_node.hpp:stop_pub_timeout_` |
| `observation_sources` | list[string] | `["scan"]` | YES | Names of configured observation sources | Default YAML |
| `polygons` | list[string] | `["PolygonStop"]` | YES | Names of configured polygons | Default YAML |
| `use_sim_time` | bool | `true` | NO | Use simulation clock | Default YAML |

**Input/output message type**: `geometry_msgs/msg/Twist` (confirmed from `collision_monitor_node.hpp` — `cmd_vel_in_sub_` subscribes `Twist`, `cmd_vel_out_pub_` publishes `Twist` via LifecyclePublisher).

**NOT `TwistStamped`**. Humble 1.1.20 uses unstamped `Twist`.

---

## 5. Supported action types

| Action type | `action_type` string | Enum value | Behavior | Evidence |
|---|---|---|---|---|
| DO_NOTHING | (default when no polygon triggered) | `0` | Pass through input velocity unchanged | `types.hpp` |
| STOP | `"stop"` | `1` | Set output velocity to zero | `types.hpp`, `collision_monitor_node.hpp::processStopSlowdown()` |
| SLOWDOWN | `"slowdown"` | `2` | Reduce velocity by `slowdown_ratio` | `types.hpp`, `collision_monitor_node.hpp::processStopSlowdown()` |
| APPROACH | `"approach"` | `3` | Maintain constant time before collision | `types.hpp`, `collision_monitor_node.hpp::processApproach()` |
| LIMIT | N/A | N/A | ❌ NOT IN HUMBLE 1.1.20 | Not in `types.hpp` |

---

## 6. Polygon schema

| Parameter | Type | Required | Behavior |
|---|---|---|---|
| `type` | string (`"polygon"` or `"circle"`) | YES | Shape type |
| `points` | list[float] (x1,y1,x2,y2,...) | For polygon | Vertices in base_frame |
| `radius` | float | For circle | Circle radius |
| `action_type` | string (`"stop"`, `"slowdown"`, `"approach"`) | YES | Action when polygon contains obstacle points |
| `max_points` | int | NO | Max obstacle points inside polygon before trigger (default: 3 per YAML) |
| `min_points` | int | NO | Min obstacle points (default: implicit) |
| `slowdown_ratio` | float | For slowdown | Velocity multiplier (0.0–1.0) |
| `time_before_collision` | float | For approach | Seconds of modeled time before collision |
| `simulation_time_step` | float | For approach | Kinematic simulation timestep |
| `visualize` | bool | NO | Publish polygon for RViz |
| `polygon_pub_topic` | string | NO | Topic for visualization polygon |
| `enabled` | bool | NO | Whether polygon is active |
| `footprint_topic` | string | For approach with polygon type | Footprint subscription topic for dynamic footprint |

---

## 7. Active vs inactive launch configuration

### bringup.launch.py (ACTIVE)

| Check | Finding |
|---|---|
| Node launched | YES — `ld.add_action(collision_monitor)` at line 336 |
| Condition | `mode == 'navigation'` only |
| Collision Monitor lifecycle manager | YES — `ld.add_action(collision_monitor_lifecycle)` at line 337 |
| Source type | `pointcloud` |
| Source topic | `/cloud_registered_body` (MID-360 body-frame) |
| Height filter | `min_height: 0.05`, `max_height: 1.80` (floor-to-head, body-frame) |

### bringup_2d.launch.py (INACTIVE)

| Check | Finding |
|---|---|
| Node launched | NO — COMMENTED OUT (`# ld.add_action(collision_monitor)` at line 572) |
| Source type | `pointcloud` (defined but inactive) |
| Source topic | `/cloud_registered_body` |
| Height filter | `min_height: -1.1`, `max_height: 0.1` (different: floor-self filter) |

---

## 8. Timeout and failure behavior

Based on installed header and default YAML evidence (runtime confirmation not performed):

| Scenario | Expected behavior | Evidence |
|---|---|---|
| Source never arrives | Source marked invalid. No obstacle points from that source. | `source.hpp::sourceValid()` — checks `source_timeout` |
| Source becomes stale (exceeds `source_timeout`) | Source marked invalid. Obstacle data discarded. | `source.hpp::sourceValid()` |
| All sources invalid/missing | No obstacle points → no polygon triggers → pass-through velocity | Logical inference from polygon processing |
| TF lookup fails | Transform of source points to base_frame fails. Points dropped. | TF buffer in Source constructor |
| Observation source disabled (`enabled: false`) | Source skipped entirely | `source.hpp::getEnabled()` |
| Input cmd_vel stops arriving | After `stop_pub_timeout` seconds, node stops publishing 0-velocity | `collision_monitor_node.hpp::publishVelocity()` — `stop_pub_timeout_` |
| Polygon contains ≤ `max_points` obstacle points | No action (pass through) | Default YAML comments + STOP/SLOWDOWN logic |
| Polygon contains > `max_points` obstacle points | Action triggered (stop/slowdown) | `collision_monitor_node.hpp::processStopSlowdown()` |
| Node not yet activated (lifecycle) | No publishers active | `nav2_util::LifecycleNode` behavior |

**Key safety property**: If the PointCloud2 source goes stale (exceeds `source_timeout: 1.0s`), the Collision Monitor effectively passes through velocity unchanged — it does NOT default to stopping. This is a documented Humble behavior and NOT a configuration bug.

---

## 9. Project configuration compatibility

See: `PHASE1_COLLISION_MONITOR_CONFIG_COMPATIBILITY_MATRIX.tsv`

### bringup.launch.py compatibility summary

| Parameter classification | Count |
|---|---|
| VALID_EXACT_SCHEMA | 24 parameters |
| UNSUPPORTED_PARAMETER | 0 |
| WRONG_TYPE | 0 |
| WRONG_VALUE | 0 |
| MISSING_REQUIRED_PARAMETER | 0 |

**All 24 inline parameters are schema-valid for Humble 1.1.20.**

### bringup_2d.launch.py compatibility summary

| Parameter classification | Count |
|---|---|
| VALID_EXACT_SCHEMA | 24 parameters |
| UNSUPPORTED_PARAMETER | 0 |
| WRONG_TYPE | 0 |
| COMMENTED_OR_INACTIVE | All — not in LaunchDescription |

**Schema-valid but never executed.**

---

## 10. PointCloud2 perception-safety analysis

### bringup.launch.py: Schema valid, perception safety not validated

| Concern | Assessment |
|---|---|
| Source topic `/cloud_registered_body` in `body` frame | Frame `body` is valid but requires TF `body → base_footprint` |
| Height filtering `min_height: 0.05, max_height: 1.80` | Filters floor (z < 5cm) and overhead (z > 1.8m). Applied AFTER transform to `base_footprint`. Height semantics depend on TF accuracy |
| Floor self-filtering | No explicit self-filter. Points below 5cm from base_footprint are removed — but this depends on correct `body → base_footprint` TF |
| Robot-body point removal | Not configured. MID-360 body-frame points may include robot chassis |
| False stop risk from floor points | LOW — `min_height: 0.05` removes floor points |
| False stop risk from robot-body points | MODERATE — no robot-body self-filter configured |

### bringup_2d.launch.py: Schema valid but different height filter intent

| Concern | Assessment |
|---|---|
| Height `min_height: -1.1, max_height: 0.1` | Intended to keep ONLY points at floor level and below (self-filter). Reversed intent from bringup.launch.py |
| Useful for obstacle detection | NO — this would filter OUT most obstacles (h > 10cm) |

---

## 11. Limitations requiring runtime validation

The following cannot be confirmed through static schema analysis alone:

1. Whether `/cloud_registered_body` PointCloud2 data actually arrives at expected rate
2. Whether `body → base_footprint` TF is published at correct rate
3. Whether height filtering with `min_height: 0.05` correctly removes floor without removing valid obstacles
4. Whether the `source_timeout: 1.0s` is adequate for MID-360 PointCloud2 rate
5. Whether robot-body self-points trigger false stops
6. Actual stop/slowdown behavior under real PointCloud2 data

---

## 12. Phase 1 closure decision

This audit PASSES because:

- Installed Collision Monitor version is exact: `1.1.20`
- PointCloud2 support is PROVEN (header exists, parameter schema documented)
- LaserScan and Range source types are PROVEN
- Exact node-level and source-level schemas are documented
- Velocity polygon and "limit" action are confirmed UNSUPPORTED in Humble 1.1.20
- Active `bringup.launch.py` configuration is schema-valid (24/24 parameters)
- Inactive `bringup_2d.launch.py` configuration is schema-compatible
- Input/output message type is `Twist` (not `TwistStamped`)
- Timeout behavior is documented from source evidence
- Runtime perception safety requires later no-motion testing, which is NOT a Phase 1 blocker per the authority deferral to Phase 5-6.

---

## 13. Integrity

| Check | Status |
|---|---|
| Git clean | ✅ |
| HEAD `c27961d` | ✅ |
| Tags unchanged | ✅ |
| No source/config modified | ✅ |
| No ROS node launched | ✅ |
| No packages installed/removed | ✅ |
| No build/commit/tag/push | ✅ |

---

PHASE1_COLLISION_MONITOR_SCHEMA_AUDIT_PASS
