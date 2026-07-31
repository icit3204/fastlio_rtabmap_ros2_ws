# Phase 1 Clock and Timestamp Policy

Date: 2026-07-31
Authority: V3.1 FINAL / PHASE-0 AUTHORITY §10A
Authority SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`
Status: PHASE_01_CLOCK_TIMESTAMP_POLICY_CREATED

## 1. Purpose and scope

This policy defines the clock domains, timestamp rules, and synchronization
requirements for the Jetson 2 autonomy stack. It covers live hardware
operation, rosbag replay, and simulation modes. It is derived from V3.1
authority §10A and applies to all ROS 2 Humble nodes in the workspace.

## 2. Clock domains

| Domain | Source | Use |
|---|---|---|
| ROS time (`/clock`) | Simulation or bag replay | Message timestamps, TF lookups, sensor data timing |
| System time (wall clock) | Jetson system clock (`CLOCK_REALTIME`) | Logging, diagnostics, human-readable timestamps |
| Steady time | `std::chrono::steady_clock` / `CLOCK_MONOTONIC` | Watchdogs, deadmen, receipt-time measurements |
| Sensor timestamps | LiDAR/IMU/camera hardware | Message `header.stamp` in live mode |

**Rule**: ROS time and system time are distinct domains. Never assume they
are synchronized. Use ROS time for sensor data and TF; use steady time for
safety-critical timing; use system time for logging only.

## 3. Live hardware mode

### Configuration

```yaml
use_sim_time: false
```

### Rules

1. **`use_sim_time` MUST be `false`** for all nodes in live mode.
2. **Preserve sensor-generated message stamps** when the sensor provides
   valid hardware timestamps. Do not replace sensor stamps with callback
   receipt time.
3. **Use the disciplined Jetson host clock** as the system-time source.
   The clock discipline method must be identified (see §13 Current unknowns).
4. **Do not substitute `ros::Time::now()` for sensor-generated stamps**
   at the driver level unless the sensor has no onboard clock.
5. **Record the timestamp source** for each sensor in the sensor inventory
   (§8).

### Current launch-file status

All bringup launch files declare `use_sim_time` as a launch argument
defaulting to `false`. This is consistent with the live-mode policy.

- `bringup.launch.py`: use_sim_time default `false`
- `bringup_2d.launch.py`: use_sim_time default `false`
- `fast_lio2.launch.py`: use_sim_time default `false`

## 4. Rosbag replay mode

### Configuration

```yaml
use_sim_time: true
```

### Rules

1. **`use_sim_time` MUST be `true`** for ALL participating nodes.
2. **Use `/clock`** published by `ros2 bag play --clock`.
3. **Never mix wall-time freshness assumptions with simulated ROS time.**
   Watchdogs that check wall-time freshness must be disabled or use only
   the bag's `/clock` progression.
4. **Record playback rate, pause state, and loop behavior** in the test log.
5. **Paused bags**: When playback is paused, `/clock` stops advancing.
   All nodes will perceive frozen time. Ensure no safety-critical watchdog
   falsely triggers on paused bag time.

### Launch procedure

```bash
ros2 bag play <bag_file> --clock
# In a separate terminal, launch nodes with use_sim_time:=true
```

## 5. Simulation mode

### Configuration

```yaml
use_sim_time: true
```

### Rules

1. **All participating nodes MUST use the same simulated clock** (`/clock`
   from Gazebo or equivalent).
2. **No live hardware watchdog may rely solely on paused simulation time.**
   A separate wall-time monitor may be needed for simulation-safety
   interlock.
3. **Gazebo publishes `/clock`** automatically. Ensure all nodes are
   configured with `use_sim_time:=true`.
4. **Simulated sensor data** must carry timestamps consistent with the
   simulated `/clock`.

## 6. Watchdog and deadman timing

### Rules

1. **Use `std::chrono::steady_clock` (C++) or `time.monotonic()` (Python)**
   for all watchdog/deadman timing decisions.
2. **Do not use adjustable wall time** (`CLOCK_REALTIME` / `system_clock`)
   for deadman decisions. System time can jump (NTP correction, manual
   adjustment).
3. **Do not rely only on ROS time** for deadman decisions. ROS time may
   jump or pause (bag replay, simulation pause, `/clock` reset).
4. **Receipt-time monitoring**: Record the steady-time receipt timestamp
   alongside the message stamp. Compare the message stamp age against
   `source_timeout` using ROS time; compare receipt intervals using
   steady time.
5. **Zero-velocity heartbeat**: The wheelchair_controller_node sends
   repeated-zero CAN frames at 20ms intervals when no command is received.
   The Generic Command Safety Gate (when implemented) must publish
   repeated-zero `/vehicle_cmd_safe` heartbeats using steady-time
   intervals.

### Example pattern

```cpp
auto receipt_time = std::chrono::steady_clock::now();
auto msg_age = node->now() - msg->header.stamp;  // ROS time age
auto receipt_interval = receipt_time - last_receipt_time_;  // steady interval

if (msg_age > source_timeout_) {
  // Source data is stale by ROS time
}
if (receipt_interval > max_interval_) {
  // Messages stopped arriving by steady time
}
```

## 7. TF and message-time rules

### Rules

1. **Use ROS message timestamps** for TF lookups.
2. **Perform TF lookup for the relevant message time**, not for `now()`.
3. **Log both the message stamp and the host receipt time** where
   end-to-end latency is measured. This enables sensor-to-output latency
   calculation.
4. **TF timeout**: Use `transform_tolerance` from node configuration
   (e.g., Collision Monitor `transform_tolerance: 0.3`). Do not hard-code
   TF timeout values in policy.

### Example

```cpp
tf_buffer_->transform(
  cloud_in, cloud_out, target_frame,
  msg->header.stamp,  // Use message time, not now()
  tf2::durationFromSec(transform_tolerance_));
```

## 8. Sensor timestamp inventory

| Sensor | Topic | Timestamp source | Status |
|---|---|---|---|
| MID-360 LiDAR | `/livox/lidar` | Livox driver (device clock or host arrival) | UNKNOWN — requires confirmation |
| MID-360 IMU | `/livox/imu` | Livox driver (device clock or host arrival) | UNKNOWN — requires confirmation |
| Stereo camera | RTSP bridge topic | Camera hardware or RTSP driver | UNKNOWN — deferred to camera phase |
| YDLIDAR | `/scan` | YDLIDAR driver | UNKNOWN — requires confirmation |
| FAST-LIO odometry | `/Odometry` | Derived from LiDAR+IMU stamps | Depends on LiDAR/IMU source |
| RTAB-Map TF | `map→odom` | RTAB-Map internal | Depends on odometry stamps |

## 9. Synchronization and skew limits

### Current state

All sensor-to-sensor and sensor-to-host timing offsets are **UNKNOWN**.
The following measurements are required before asserting acceptable skew:

| Measurement | Status | Required for |
|---|---|---|
| MID-360 device-to-host timestamp offset | UNKNOWN | LiDAR data age accuracy |
| IMU-to-LiDAR timestamp offset | UNKNOWN | FAST-LIO sensor fusion quality |
| Camera-to-LiDAR timestamp offset | UNKNOWN | Camera-LiDAR data association |
| Host clock discipline method | UNKNOWN | System-time accuracy |
| End-to-end sensing-to-command latency | UNKNOWN | Control loop stability |

### Meeting-derived target

The approximately 100 ms end-to-end latency figure discussed in the
July 24 meeting is recorded as:

**MEETING_DERIVED_TARGET_REQUIRES_CONFIRMATION**

It is NOT a formal acceptance threshold. Do not use it as a pass/fail
criterion until measured latency data confirms feasibility.

### Maximum acceptable skew

No numeric skew limits are defined. They must be derived from:
- FAST-LIO odometry rate and accuracy requirements;
- Collision Monitor `source_timeout` constraints;
- Nav2 control loop period;
- TF `transform_tolerance` settings.

## 10. Logging and traceability

### Rules

1. **Log message stamps and receipt times** for critical topics
   (`/Odometry`, `/livox/lidar`, `/cloud_registered_body`, `/cmd_vel_nav`,
   `/vehicle_cmd_safe` when implemented).
2. **Tag logs with clock domain**: ROS time for sensor data age, steady
   time for receipt intervals, system time for human correlation.
3. **Bag metadata**: Record `use_sim_time`, playback rate, and clock
   source in bag metadata or accompanying test log.

## 11. Future multi-computer policy

### Reserved requirements

If future robots distribute sensing and control across multiple computers:

1. **Select one synchronization mechanism** (NTP, PTP/IEEE 1588, or
   equivalent) per robot profile.
2. **Measure and document** the steady-state clock offset and drift
   between computers.
3. **Define acceptance limits** for maximum inter-computer clock skew.
4. **Publish `/clock` from one designated master** if using distributed
   simulation.

### Current applicability

Jetson 2 is a single-computer system. Multi-computer synchronization is
**NOT currently required** but is reserved for future chassis.

## 12. Verification checklist

| # | Check | Phase | Method | Status |
|---|---|---|---|---|
| C01 | `use_sim_time=false` in live launches | Phase 1 | Source inspection | PASS — all bringup launch files default to false |
| C02 | `use_sim_time` consistency across nodes | Phase 5 | Runtime inspection (`ros2 param get`) | DEFERRED |
| C03 | MID-360 LiDAR timestamp source confirmed | Phase 5 | Livox driver documentation or runtime test | DEFERRED |
| C04 | IMU timestamp source confirmed | Phase 5 | Livox driver documentation or runtime test | DEFERRED |
| C05 | Camera timestamp source confirmed | Camera phase | RTSP driver documentation | DEFERRED |
| C06 | Watchdog uses steady time | Phase 4 | Source inspection of safety gate | DEFERRED |
| C07 | Deadman uses steady time | Phase 4 | Source inspection of command adapter | DEFERRED |
| C08 | TF lookups use message time | Phase 5 | Runtime inspection or log analysis | DEFERRED |
| C09 | Sensor-to-host latency measured | Phase 5 | End-to-end timing test | DEFERRED |
| C10 | Bag replay with `/clock` tested | Phase 5 | Bag playback test with use_sim_time | DEFERRED |
| C11 | Host clock discipline method documented | Phase 5 | System configuration inspection | DEFERRED |
| C12 | Policy tracked in repository | Phase 1 | This document | PASS |

## 13. Current unknowns

The following items are explicitly unknown and must be resolved before
physical navigation validation:

1. **Exact MID-360 device/host timestamp mode** — whether the Livox driver
   uses hardware timestamps from MID-360 packets or assigns host arrival
   time.
2. **Exact IMU timestamp source** — whether the IMU data shares the
   LiDAR timestamp source or uses an independent clock.
3. **Camera timestamp source** — RTSP stream timestamp semantics.
4. **Measured LiDAR-to-host offset** — network and driver latency from
   MID-360 photon event to ROS message publication.
5. **Measured camera-to-LiDAR skew** — time offset between camera frame
   capture and corresponding LiDAR scan.
6. **Jetson clock-discipline method** — whether `systemd-timesyncd`,
   `chrony`, or `ntpd` is active; whether the clock is disciplined from
   a reliable NTP source.
7. **Final acceptable skew limits** — per-pipeline maximum timestamp
   skew values (not yet derived from component requirements).

## 14. Change-control rules

1. **Policy version**: Increment when clock-domain rules or sensor
   timestamp sources change.
2. **Review triggers**: After any sensor addition/change, after physical
   timing measurement, after bag-replay behavior change.
3. **Authority alignment**: Any change to clock-domain rules must be
   consistent with V3.1 §10A or must update the authority with a
   documented revision.
4. **Measurement evidence**: All resolved UNKNOWN items must reference
   the measurement report, date, and operator.
