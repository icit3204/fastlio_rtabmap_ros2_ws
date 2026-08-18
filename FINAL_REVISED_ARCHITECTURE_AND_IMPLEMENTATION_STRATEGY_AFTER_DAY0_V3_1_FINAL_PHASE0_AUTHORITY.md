# Final Revised Architecture and Implementation Strategy

## Project

**Scalable Parking/Charging-Robot Autonomy Platform**

**Current development chassis:** CAN-controlled electric wheelchair  
**Future target:** company parking/charging robots with replaceable chassis and site-specific configurations  
**Autonomy stack:** MID-360 semantic perception, FAST-LIO odometry, RTAB-Map mapping/localization, Nav2 planning/control, generic command-safety gating, and chassis-specific vehicle adaptation  
**Strategy status:** V3.1 FINAL / PHASE-0 AUTHORITY after current-source audits, recovered-work review, day-zero requirement review, sequencing decision, July 24 meeting review, V3 consistency review, and final four-item cleanup  
**Target environment:** new native Jetson, Ubuntu 22.04 / ROS 2 Humble  
**Primary current workspace reference:** `/home/dog/fastlio_rtabmap_ros2_ws`  
**Recovered development references:** `competition_practice_ws`, recovered FAST-LIO/Livox workspaces, and associated milestone evidence  
**Revision date:** 2026-07-30 — Revision 3.1 FINAL / PHASE-0 AUTHORITY


### Revision 3.1 correction log

Revision 3.1 is a consistency and completeness correction. It does not redesign the selected autonomy chain.

Corrections:

- separated mandatory localization-valid checks from configurable map-consistency checks;
- defined controlled handling of RTAB-Map loop-closure/relocalization pose jumps;
- replaced the weak geometry-only mission contract with typed route and mission-state interfaces;
- retained `nav_msgs/Path` only as a temporary compatibility input;
- added a supervised Phase 5B reproduction of the known working legacy physical baseline before new Nav2-to-CAN testing;
- added a generic `/vehicle_state` feedback boundary without inventing unsupported wheelchair feedback;
- defined the exact `/vehicle_cmd_safe` coordinate, unit, numerical and unsupported-motion contract;
- removed the duplicate repeated-zero requirement;
- added timestamp/clock-synchronization policy;
- added calibration-manifest versioning;
- clarified that module statuses are assigned in the Phase 0 module registry;
- added product security and software-supply-chain requirements as `PRODUCTIZATION_LATER`;
- updated package structure, diagnostics, phases, acceptance criteria and immediate actions consistently;
- changed `VehicleState` status fields from plain booleans to explicit tri-state values with a validity mask;
- clarified that unavailable lower-controller feedback does not automatically block the supervised low-speed Phase 6 interface test, while remaining a production limitation;
- defined `RouteMission` array, frame, direction and validation invariants;
- separated Navigation Constraint YAML overlays from RViz profile files.

### Revision 3 change log retained

Revision 3 preserves the Revision 2 command architecture and adds the justified changes from the July 24 meeting review and the final technical corrections:

- Preserved the reusable command boundary `/vehicle_cmd_safe`.
- Clarified `TwistStamped` timestamp, frame and watchdog semantics.
- Added independent stale-input/deadman behavior to the Wheelchair Command Adapter.
- Added the complete current wheelchair-message contract and unit-test requirement.
- Made map-bounds and occupancy-based localization checks configurable.
- Added continuous command-authority monitoring while armed.
- Added configurable zero-command heartbeat requirements.
- Added explicit verification of the installed ROS 2 Humble Collision Monitor source schema.
- Removed editing-history commentary from the authoritative strategy.
- Added MPPI-versus-DWB reference benchmarking and AMCL contingency criteria.
- Added a versioned map-editing workflow.
- Added a unified RViz visualization/debug deliverable.
- Added a generic mission state machine, blocked classification and future help escalation.
- Added an unambiguous topological-route identity contract.
- Added Gazebo as a supporting validation track, not a second architecture.
- Added heartbeat/freshness/freeze supervision and controlled restart rules.
- Added full Jetson system-image preservation in Phase 0.
- Added a provisional approximately 100 ms meeting-derived latency target requiring confirmation.
- Added an explicit non-goal for full free-space 3D navigation.
- Added collaborator/company ownership and interface boundaries.
- Added event-triggered visual-assisted relocalization as the preferred later camera mode.

### Scope boundary

This document defines the reusable autonomy, perception, localization, navigation, safety-command, and vehicle-adapter platform.

Charging-task functions such as charger discovery, docking alignment, manipulator control, charging communication, customer scheduling, and fleet dispatch are not yet specified in the available day-zero documents. They are therefore represented as future mission/application interfaces rather than invented requirements.

The current wheelchair is a development and validation chassis. The autonomy architecture must not depend permanently on wheelchair-specific lower-control details.

**Current navigation scope:** a ground vehicle using 3D perception and mapping to produce 2D/2.5D navigation constraints and costmaps. Full free-space 3D navigation is outside the current contract unless separately specified.
---

## 1. Executive decision

The project will preserve the parts that already work, restore Nav2 as the intended navigation authority, and enforce a clean boundary between reusable command safety and chassis-specific command conversion.

### Preserve

- Livox MID-360 driver and IMU input.
- FAST-LIO odometry and registered point clouds.
- RTAB-Map mapping, database, loop closure, and saved-map localization.
- The authoritative root `plan_nav/` GUI, topology editor, Dijkstra route selection, and route display.
- The existing `wheelchair_controller_node` and its working SocketCAN encoder.
- The current plan_nav/Pure Pursuit/CAN path as a separately launchable legacy baseline and rollback mode.
- Existing maps, databases, topology files, calibration, source audits, and validated recovered work.

### Replace in the new autonomous mode

```text
Current physical authority:
plan_nav path
-> custom Pure Pursuit
-> /wheelchair_control_command
-> existing CAN driver
```

becomes:

```text
mission source
-> mission manager
-> Nav2 goal
-> Nav2 global planner
-> MPPI
-> Collision Monitor
-> Generic Command Safety Gate
-> /vehicle_cmd_safe  (TwistStamped)
-> Chassis-Specific Command Adapter
-> current wheelchair CAN driver or future robot controller
```

### Primary perception path

```text
MID-360
-> FAST-LIO
-> semantic_grid_tools
-> /obstacle_cloud
-> Nav2 local/global costmaps
```

### Product-scale command boundary

Everything above `/vehicle_cmd_safe` is reusable autonomy and generic command safety. Everything below it is chassis-specific.

```text
Reusable autonomy core
-> Collision Monitor
-> Generic Command Safety Gate
-> /vehicle_cmd_safe
   |-> Wheelchair Command Adapter
   |    -> /wheelchair_control_command
   |    -> existing wheelchair_controller_node
   |    -> SocketCAN
   |
   +-> Future Charging-Robot Command Adapter
        -> future lower-controller protocol
```

The generic safety gate must not contain wheelchair-specific radius calculations, millimetre units, CAN conventions, wheelbase values, or `Float32MultiArray` formatting.

### Early localization rule

Advanced relocalization remains a later work package, but a minimum binary localization-valid gate is required before any Phase 6 physical Nav2 test.

```text
Odometry freshness
+ map -> odom freshness
+ complete map -> base_footprint TF
+ finite pose and valid quaternion
+ pose-jump check
+ stability window
        |
        v
LOCALIZATION_VALID / LOCALIZATION_INVALID
```

Configurable map-consistency checks may additionally verify map bounds, occupied/invalid cells and unknown-space policy when valid map metadata is available.

Invalid localization must refuse real arming and force zero command. A loop-closure or relocalization jump must pause the mission, mark localization unstable, wait for the configured stability window and require controlled resume/re-arming; it is not automatically a permanent failure.

### Temporary Phase 6 obstacle source

Before semantic perception is ported, Phase 6 will use a separate temporary raw-MID-360 Collision Monitor profile:

```text
/cloud_registered_body
-> temporary verified Collision Monitor profile
-> /cmd_vel_nav_safe
```

This profile is only for a supervised clear-area command-interface test. It does not claim semantic obstacle avoidance or final collision-performance validation.

### Deferred requirements remain visible

The following remain part of the final architecture but are disabled or postponed until their activation gates are satisfied:

- Navigation Constraint Layer: keepout zones, virtual walls, speed zones, temporary restrictions, and site masks.
- Stereo-camera assistance for loop closure, relocalization, and mapping robustness.
- YDLIDAR as an optional independent close-range stop/slow source.
- Ultrasonic or other near-field sensors when actual hardware is available.
- Site-specific charging/parking rules.
- Fleet deployment, configuration distribution, telemetry, and rollout controls.

### Core authority rules

- Only one publisher may own `/vehicle_cmd_safe`.
- Only one chassis adapter may publish the current chassis command topic.
- Only one process may write commands to the physical lower controller.
- Legacy Pure Pursuit and the new Nav2 command chain must never have simultaneous physical authority.

### Meeting-derived architecture clarifications

The July 24 meeting confirms several requirements around the selected core architecture:

- normal navigation should remain LiDAR-dominant;
- vision should mainly support mapping, loop closure and relocalization without slowing normal navigation;
- the final product receives tasks from a company platform rather than relying on manual start/goal selection;
- the system needs one coordinated mission state machine;
- local avoidance must distinguish passable, temporarily blocked and unpassable situations;
- important project data must be visible in one reusable RViz interface;
- simulation and recorded-data replay should precede difficult physical tests;
- important nodes require health/freshness supervision and later controlled restart;
- a known-working Jetson image must be recoverable;
- the system must remain suitable for large-scale future deployment.

These clarifications add mission, observability and supervision planes. They do not replace the selected perception/control command chain.

### Cross-cutting planes

```text
MISSION / STATE PLANE
company platform later
-> platform adapter
-> Mission Manager state machine
-> Nav2 actions
-> mission outcome/status
-> platform adapter later

OBSERVABILITY PLANE
important topics + diagnostics
-> standard RViz/debug profiles
-> developer/operator

SUPERVISION PLANE
heartbeats + freshness + lifecycle + latency
-> Health Supervisor
-> pause / stop / controlled restart / help escalation
```

## 2. Evidence and current confirmed baseline

The strategy is based on the following confirmed findings from the project audits and recovered work:

1. The authoritative `plan_nav` copy is the root `plan_nav/`; backup copies under `plan_nav07202900/` are not referenced by current startup.
2. FAST-LIO publishes `odom -> body`, not `odom -> base_footprint`.
3. The live TF chain is intended to be:

```text
map -> odom -> body -> base_footprint -> base_link -> livox_frame
```

4. Current default physical control is:

```text
/plan_nav
-> plan_nav_laser_avoidance
-> /active_plan
-> pure_pursuit
-> /wheelchair_control_command_raw
-> laser_command_safety_filter
-> /wheelchair_control_command
-> wheelchair_controller_node
-> CAN
```

5. Nav2 MPPI publishes `/cmd_vel_nav`, but this output does not currently control CAN.
6. Collision Monitor exists in source/configuration but is commented out in the current 2D live launch and does not protect the current wheelchair command topic.
7. The existing CAN node uses `can0`, extended CAN ID `0x801400`, DLC 8, int16 little-endian wheel fields, a 20 ms transmit timer, and a 500 ms stale-command timeout.
8. The wheelchair has physically followed plan_nav routes through the current CAN path, as confirmed by the user.
9. The manual joystick always overrides autonomous commands, as confirmed by the user.
10. Recovered `semantic_grid_tools` successfully used FAST-LIO `/cloud_registered` in `odom`, published `/ground_cloud`, `/obstacle_cloud`, and `/semantic_grid`, and fed Nav2 costmaps.
11. Recovered offline tests demonstrated semantic-costmap planning and fake closed-loop obstacle avoidance to successful Nav2 goals.
12. A guarded Nav2 Twist-to-wheelchair conversion path was previously tested safely using a mock wheelchair command topic.

---

## 3. Project objectives

### 3.1 Immediate engineering objective

Create a stable ROS 2 Humble system on the new Jetson that can:

- build and launch reproducibly without relying on stale generated folders;
- load or create an RTAB-Map map;
- localize using FAST-LIO and RTAB-Map;
- accept a start pose and a simple navigation goal;
- generate a global path with NavFn A*;
- follow it with MPPI in fake/mock mode;
- pass only Collision-Monitor-filtered Twist commands through a generic safety gate and a mock chassis adapter;
- port the recovered semantic pipeline;
- place semantic obstacles into Nav2 costmaps;
- validate obstacle avoidance offline and with live sensors but no physical output;
- preserve the existing SocketCAN driver for later controlled physical transition.

### 3.2 Current-platform objective

On the wheelchair development chassis:

- preserve the known working legacy plan_nav/Pure Pursuit/CAN baseline;
- prove the new Nav2-to-existing-CAN interface at very low speed in a clear area;
- validate semantic obstacle avoidance and final software stop/slow behavior;
- retain manual joystick override as the operator's highest physical authority during supervised development.

### 3.3 Long-term product objective

Support many future parking/charging robots without rewriting the autonomy core.

Mission sources may include:

```text
plan_nav GUI
company task scheduler
parking/charging-site mission
GPS/RTK route
predefined route
remote/API command
fleet manager
```

All feed one mission interface. Perception, localization, Nav2, health monitoring, Collision Monitor, and the generic command-safety gate remain common. Only the robot model, calibration, footprint, kinematic limits, sensor profile, site profile, chassis command adapter, and lower-controller driver vary.

### 3.4 Day-zero requirement reconciliation

The day-zero company-style plan requires:

- RTAB-Map mapping and map loading;
- Nav2 global and local navigation;
- relocalization and localization recovery;
- keepout zones, virtual walls, temporary restrictions, speed-related constraints, and map modification;
- LiDAR and stereo-camera evaluation;
- optional single-line LiDAR and ultrasonic near-field sensing;
- dynamic-obstacle testing;
- underground-garage and outdoor-parking validation.

These requirements remain in the final architecture, but their implementation order is revised to reduce debugging complexity.

### 3.5 Module status classes

Every tracked module shall receive one of these statuses in the Phase 0 requirement and module registry:

| Status | Meaning |
|---|---|
| `CORE_NOW` | required for the current milestone and enabled |
| `CORE_LATER_GATE` | required for final architecture but activated only after a named pass gate |
| `OPTIONAL_EVALUATE` | retained only if testing shows measurable benefit |
| `HARDWARE_FUTURE` | interface reserved; hardware or protocol not currently available |
| `LEGACY_BASELINE` | preserved for comparison and rollback, not part of new authority |
| `SITE_PROFILE` | configured only when actual parking/charging-site rules are known |
| `PRODUCTIZATION_LATER` | required for fleet deployment, not for first robot navigation |

### 3.6 Non-goals for the first implementation

Do not initially:

- rewrite the lower CAN protocol;
- modify current YDLIDAR avoidance code;
- add ultrasonic software without hardware;
- require the stereo camera for basic navigation;
- enable keepout/speed filters in the core baseline;
- rely on raw GPS for precise local navigation;
- replace RTAB-Map;
- add TEB or a neural semantic model before the selected baseline is stable;
- invent charging/docking behavior not present in the available specifications;
- remove the working legacy baseline;
- enable two physical controllers simultaneously;
- implement full free-space 3D navigation;
- invent the company platform message protocol before the specification exists.


## 3.7 Revision changes from the previous strategy

This revision adds or changes:

- explicit charging-robot/fleet product vision;
- chassis-independent autonomy and a chassis-specific adapter boundary;
- day-zero requirement status tracking;
- deferred Navigation Constraint Layer with activation gates;
- delayed stereo-camera evaluation;
- future ultrasonic interface;
- revised core-first implementation sequence;
- distinction between early clear-path CAN interface validation and final semantic physical authority;
- relocalization and localization-health phase;
- site-profile and robot-model configuration;
- fleet/productization requirements without blocking current development.


## 3.8 Revision 2 focused corrections retained in Revision 3

Revision 3 retains the following Revision 2 corrections:

1. Reusable `Generic Command Safety Gate` separated from wheelchair-specific command conversion.
2. `/cloud_registered_body` limited to a temporary, no-motion-validated Phase 6 Collision Monitor profile.
3. A minimum `LOCALIZATION_VALID / LOCALIZATION_INVALID` gate required before any physical Nav2 arming.

## 3.9 Revision 3 meeting-derived additions

Revision 3 additionally introduces:

- unified RViz visualization/debug profiles as a formal deliverable;
- a generic Mission Manager state machine;
- passable/temporarily-blocked/unpassable navigation outcomes;
- generic `HELP_REQUIRED` escalation without inventing company messages;
- route identity and direction metadata requirements;
- Gazebo as a compatible validation track;
- early freshness/freeze supervision and later controlled restart;
- full Jetson image backup;
- provisional latency measurement and reporting;
- explicit ownership and interface boundaries;
- event-triggered camera-assisted relocalization;
- DWB reference benchmarking, AMCL contingency, and versioned map editing.

## 3.10 Revision 3.1 consistency corrections

Revision 3.1 adds no competing navigation design. It makes the selected architecture implementable and internally consistent by defining:

- mandatory versus configurable localization checks;
- typed route, mission-state and vehicle-state interfaces;
- a legacy physical-baseline reproduction gate before new command authority;
- the full generic vehicle-command contract;
- clock/timestamp and calibration-manifest policies;
- product security and software-supply-chain requirements for later deployment.

# Part I - Existing architecture

## 4. Existing sensor and odometry pipeline

```text
Livox MID-360
-> livox_ros_driver2
-> /livox/lidar        livox_ros_driver2/msg/CustomMsg
-> /livox/imu          sensor_msgs/msg/Imu
-> FAST-LIO
-> /Odometry           nav_msgs/msg/Odometry
-> TF odom -> body
-> /cloud_registered   PointCloud2 in odom
-> /cloud_registered_body PointCloud2 in body
-> /path               nav_msgs/msg/Path in odom
```

### Current responsibility

FAST-LIO provides local motion estimation and registered clouds. It does not provide the global map correction.

## 5. Existing mapping and localization pipeline

```text
FAST-LIO /Odometry + registered cloud
+ optional camera data
-> RTAB-Map
-> RTAB-Map database
-> /map
-> TF map -> odom
-> saved-map localization
```

### Current responsibility

RTAB-Map provides:

- map memory;
- graph optimization;
- loop closure;
- saved database loading;
- localization against the saved map;
- global `map -> odom` correction.

The primary current database is `map/rtabmap_2d.db`, subject to checksum verification after transfer.

## 6. Existing plan_nav mission and path pipeline

```text
plan_nav GUI
-> user selects start/end or route nodes
-> topology graph
-> Dijkstra route selection
-> dense trajectory generation
-> /plan_nav nav_msgs/Path in map
```

### Current responsibility

`plan_nav` currently combines:

- mission selection;
- topological route choice;
- route visualization;
- direct path publication for the physical controller.

It does not currently send Nav2 goals.

## 7. Existing navigation and control pipeline

### Current default mode with YDLIDAR avoidance

```text
/plan_nav
+ /scan
-> plan_nav_laser_avoidance
-> /active_plan
-> custom pure_pursuit_controller_node
-> /wheelchair_control_command_raw
+ /scan
-> laser_command_safety_filter
-> /wheelchair_control_command
-> wheelchair_controller_node
-> CAN
```

### Parallel Nav2 branch

```text
/map + odometry + point cloud
-> Nav2 global/local costmaps
-> NavFn planner
-> MPPI controller
-> /cmd_vel_nav
```

### Existing central limitation

The Nav2 branch can calculate obstacle-aware paths and Twist commands, but those commands do not govern the CAN output. Physical authority remains with custom Pure Pursuit.

## 8. Existing physical control pipeline

```text
/wheelchair_control_command
-> wheelchair_controller_node
-> radius/velocity to wheel-value conversion
-> SocketCAN raw frame
-> can0
-> lower controller
-> wheelchair
```

This lower hardware layer is preserved in the new architecture.

---

# Part II - Selected new architecture

## 9. Full selected architecture

### 9.1 Reusable product-level architecture

```text
                           APPLICATION / MISSION SOURCES
        +----------------------+----------------------+------------------+
        |                      |                      |                  |
   plan_nav GUI         charging task later       GPS/RTK later    remote/fleet API
        |                      |                      |                  |
        +----------------------> Mission Manager <----------------------+
                                      |
                            standard Nav2 mission goals
                                      |
                                      v
                              Nav2 BT Navigator
                                      |
                                      v
                         Global Planner: NavFn A*
                                      |
                              global metric path
                                      |
                                      v
                              MPPI Controller
                                      |
                           /cmd_vel_nav_raw
                                      |
                                      v
                             Collision Monitor
                                      |
                           /cmd_vel_nav_safe
                                      |
                                      v
                      Generic Command Safety Gate
                   arming, freshness, generic limits, faults
                                      |
                  /vehicle_cmd_safe  geometry_msgs/TwistStamped
                                      |
                  +-------------------+--------------------+
                  |                                        |
                  v                                        v
       Wheelchair Command Adapter              Future Robot Command Adapter
       Twist -> radius/velocity                Twist -> future command format
                  |                                        |
       /wheelchair_control_command             future robot command topic
                  |                                        |
       existing wheelchair_controller_node     future lower-controller driver
                  |                                        |
              SocketCAN                              future protocol
                  |                                        |
                  +----------------> robot chassis <--------+
```

### 9.2 Localization and perception

```text
MID-360 + IMU
-> Livox driver
-> FAST-LIO
   |-> /Odometry and odom -> body
   |-> /cloud_registered in odom
   |-> /cloud_registered_body in body
   |
   +-> RTAB-Map
   |    |-> map database
   |    |-> /map
   |    |-> map -> odom
   |    +-> localization / loop closure / relocalization
   |
   +-> semantic_grid_tools
        |-> /ground_cloud
        |-> /obstacle_cloud
        +-> /semantic_grid
                 |
                 v
        Nav2 local/global costmaps
```

### 9.3 Basic localization-valid gate before physical testing

Mandatory checks:

```text
/Odometry freshness
+ map -> odom freshness
+ complete map -> base_footprint transform
+ finite pose and normalized quaternion
+ translation/rotation jump threshold
+ configurable stability window
        |
        v
Basic Localization Validity Monitor
        |
        +-> /system/localization_valid
        +-> /system/localization_reason
        +-> /system/pose_age
        +-> /system/tf_age
        +-> /system/pose_jump_detected
        |
        +-> Mission Manager permission
        +-> Generic Command Safety Gate arming permission
```

Configurable map-consistency checks, enabled only when valid map metadata and a selected site policy are available:

- pose inside map bounds;
- pose not inside an occupied/invalid cell;
- unknown-space acceptance or rejection;
- required clearance from lethal/inflated cells.

A detected jump caused by loop closure or relocalization must:

```text
pause mission
-> mark localization unstable
-> wait for stable TF/pose
-> require controlled resume or re-arming
```

It must not be classified automatically as permanent localization failure.

This early gate is intentionally binary. Advanced `LOCALIZING / DEGRADED / LOST / RELOCALIZING / RECOVERED` logic remains in Phase 12.

### 9.4 Required-deferred Navigation Constraint Layer

Initially disabled:

```text
site map / constraint masks
-> Keepout Filter
-> virtual-wall mask
-> Speed Filter
-> temporary-restriction mask
-> site-specific no-go and slow zones
```

These become active only after the core navigation and semantic obstacle-avoidance gates pass.

### 9.5 Optional/future sensor branches

```text
Stereo camera
-> later RTAB-Map mapping / loop-closure / relocalization evaluation
-> optional depth obstacle source only if measured useful

YDLIDAR /scan
-> preserve current code
-> later optional independent stop/slow source

Ultrasonic / Range sensors
-> future hardware interface
-> near-field blind-zone compensation
```

### 9.6 Collision Monitor source progression

```text
Phase 4:
synthetic observation source in offline/mock tests

Phase 5-6 temporary profile:
/cloud_registered_body
-> verified raw MID-360 filters
-> Collision Monitor

Phase 8 onward final profile:
/obstacle_cloud
-> Collision Monitor
```

The temporary raw-cloud profile must be retired from the final semantic mode unless a controlled comparison justifies retaining it.

### 9.7 Mission, observability and supervision architecture

```text
Company platform later
-> Platform Adapter
-> Generic Mission Command
-> Mission Manager
-> Nav2 action
-> Nav2 outcome
-> Mission Manager state
-> Platform Adapter later
```

```text
TF + map + pose + paths + clouds + costmaps + zones + health
-> standard RViz profiles
-> development validation and presentation
```

```text
node/process presence
+ topic/message freshness
+ TF age
+ lifecycle state
+ controller/latency health
-> Navigation Health Supervisor
-> pause mission
-> zero vehicle command
-> controlled subsystem recovery later
```

### 9.8 Basic mission state machine

The Mission Manager shall implement a generic internal state machine independent of the future company protocol:

```text
IDLE
RECEIVED
VALIDATING
PLANNING
NAVIGATING
PAUSED
CANCELLING
CANCELLED
SUCCEEDED
TEMPORARILY_BLOCKED
BLOCKED
FAILED
HELP_REQUIRED
```

`HELP_REQUIRED` is an internal generic status. Mapping it to a company message is deferred until the company protocol is provided.

### 9.9 Navigation outcome policy

The navigation system shall not repeatedly advance experimentally toward an obstacle.

```text
PASSABLE
-> safe path/control progress exists
-> continue

TEMPORARILY_BLOCKED
-> stop
-> wait/replan within configured limits

UNPASSABLE / BLOCKED
-> stop
-> terminate or pause mission
-> report BLOCKED
-> optionally escalate HELP_REQUIRED
```

Classification may use planner results, progress checking, controller output, costmap state, Collision Monitor state, recovery attempts and time without progress.

### 9.10 Topological route identity contract

The plan_nav-to-Mission-Manager interface must eventually carry unambiguous route identity, not geometry alone.

Recommended metadata:

```text
route_id
edge_id or ordered edge IDs
direction
ordered node IDs
ordered poses
topology/map version
```

The full plan_nav GUI solution is collaborator-owned. The Mission Manager must reject ambiguous routes rather than guessing from nearest geometry.

## 10. New frame architecture

### Selected TF ownership

```text
RTAB-Map:          map -> odom
FAST-LIO:          odom -> body
Static transform:  body -> base_footprint
Static transform:  base_footprint -> base_link
Static transform:  base_link -> livox_frame
Static/driver TF:  base_link -> camera and optional laser frames
```

### Selected Nav2 frames

| Component | Selected frame |
|---|---|
| Global map | `map` |
| Global costmap | `map` |
| Local costmap global frame | `odom` |
| Robot base frame | `base_footprint` |
| FAST-LIO child | `body` |
| Semantic V1 fixed frame | `odom` |
| Mission goals | `map` |

### Required verification

The current static `body -> base_footprint` z offset must be physically verified. No new live navigation phase may proceed if the TF tree has duplicate publishers or conflicting values.

---

## 10A. Timestamp and clock-synchronization policy

Freshness, synchronization and latency measurements must use an explicit clock policy.

Record and verify:

- ROS time versus system time;
- whether `use_sim_time` is enabled;
- MID-360 LiDAR and IMU timestamp source;
- stereo-camera timestamp source;
- Jetson system-clock source;
- measured camera-to-LiDAR and sensor-to-host offsets;
- maximum acceptable timestamp skew for each synchronized pipeline;
- bag-playback clock behavior;
- steady/monotonic receipt time for deadman/watchdog decisions.

Rules:

```text
live mode:
use real sensor stamps and a disciplined host clock

bag/simulation mode:
use /clock consistently and never mix stale wall-time assumptions

watchdogs:
use steady/monotonic receipt time

traceability and TF lookup:
use ROS message timestamps
```

If future robots distribute sensing and control across computers, the robot profile must define the selected NTP/PTP or equivalent synchronization method and acceptance limits.

## 11. Semantic perception architecture

### 11.1 Selected Version 1 input

```text
/cloud_registered
frame: odom
```

Reason:

- it matches the recovered implementation;
- the recovered tests already validated this interface;
- it avoids raw body-frame multi-scan accumulation;
- it provides a fixed odometry frame for the initial grid.

### 11.2 Outputs

```text
/ground_cloud
sensor_msgs/msg/PointCloud2
frame: odom

/obstacle_cloud
sensor_msgs/msg/PointCloud2
frame: odom

/semantic_grid
nav_msgs/msg/OccupancyGrid
frame: odom
```

### 11.3 Selected initial algorithm

- input filtering;
- range filtering;
- voxel/downsampling where required;
- RANSAC ground plane estimation;
- ground distance classification;
- obstacle height classification;
- ground and obstacle vote accumulation;
- bounded grid publication;
- PointCloud2 ground and obstacle debug outputs.

### 11.4 Required improvements before live authority

The recovered node must be audited or modified for:

- timestamp preservation;
- correct QoS for sensor streams;
- bounded memory;
- vote decay or grid reset policy;
- stale-cloud detection;
- moving-obstacle clearing;
- configurable grid origin and dimensions;
- proper TF handling if a non-odom input is later used;
- diagnostic output for processing rate, latency, and point counts.

### 11.5 Version 2 option

After Version 1 is stable, compare:

```text
/cloud_registered
```

with:

```text
/cloud_registered_body
-> timestamped TF transform to odom
-> semantic processing
```

Raw body-frame clouds must never be accumulated directly into a persistent fixed grid.

---

## 12. Nav2 costmap architecture

### 12.1 Global costmap

```text
Frame: map
Robot base: base_footprint
Layers:
- static layer from RTAB-Map/map server
- semantic obstacle observation source
- inflation layer
```

Selected semantic source:

```text
/obstacle_cloud
PointCloud2
marking: true
clearing: configured and tested
```

### 12.2 Local costmap

```text
Frame: odom
Robot base: base_footprint
Rolling window: true
Layers:
- semantic obstacle observation source
- inflation layer
- optional voxel layer only if measured useful
```

### 12.3 Ground output

`/ground_cloud` is not used as an obstacle marking source. It remains available for:

- RViz;
- algorithm validation;
- traversability analysis;
- later terrain/slope extension.

### 12.4 Duplicate-source policy

In the semantic mode, do not simultaneously mark the same raw MID-360 points from both:

```text
/cloud_registered_body
```

and:

```text
/obstacle_cloud
```

Start with semantic `/obstacle_cloud` as the main MID-360 obstacle source. Compare with the raw source in controlled tests before deciding whether any raw source remains.

### 12.5 Initial clearing policy

Because the recovered semantic obstacle output may accumulate votes, the first costmap profile must be conservative and test:

- observation persistence;
- expected update rate;
- ray tracing or clearing capability;
- stale data timeout;
- whether previously occupied cells disappear after obstacles move.

---

## 13. Mission and plan_nav architecture

### 13.1 Final role of plan_nav

Keep:

- GUI;
- map/database visualization;
- topological graph;
- route editing;
- Dijkstra route selection;
- start and destination selection;
- ordered semantic waypoint generation;
- route and progress display.

Remove from new physical authority:

- velocity generation;
- Pure Pursuit control;
- obstacle detour generation as the primary local planner;
- final wheelchair command publication;
- direct CAN authority.

### 13.2 Selected Version 1 mission behavior

```text
plan_nav Dijkstra route
-> sparse ordered waypoints
-> sequential NavigateToPose goals
-> Nav2 plans and controls each segment
```

### 13.3 Mission manager design

Recommended new ROS package:

```text
parking_robot_mission_manager
```

Recommended node:

```text
mission_manager_node
```

Responsibilities:

- receive an ordered waypoint route from plan_nav;
- validate frame is `map`;
- remove duplicate or extremely close points;
- send one `NavigateToPose` goal at a time;
- wait for result before advancing;
- report current waypoint and route progress;
- support cancel and pause;
- stop on rejected or aborted waypoint;
- never silently skip a failed waypoint;
- never publish `/initialpose` automatically in live mode;
- never publish motor commands;
- own the generic mission state machine;
- classify temporary blockage versus terminal blockage;
- enforce bounded recovery attempts and progress timeouts;
- expose generic `BLOCKED`, `FAILED`, `SUCCEEDED`, and `HELP_REQUIRED` outcomes;
- reject ambiguous route identity or direction.

### 13.4 Selected typed interfaces

Create a small generic interface package:

```text
parking_robot_interfaces
```

Primary mission input:

```text
/mission/route
parking_robot_interfaces/msg/RouteMission
```

Minimum conceptual fields:

```text
std_msgs/Header header
string mission_id
string route_id
string topology_version
string[] node_ids
string[] edge_ids
int8[] edge_directions
geometry_msgs/PoseStamped[] poses
```

Required `RouteMission` invariants:

```text
header.frame_id:
normally map for topological navigation missions

mission_id:
non-empty and unique within the active mission scope

route_id:
non-empty for plan_nav/topological missions

topology_version:
must match the topology version loaded by the Mission Manager

poses.size():
must equal node_ids.size()

edge_ids.size():
normally poses.size() - 1

edge_directions.size():
must equal edge_ids.size()

edge direction values:
-1 = reverse
 0 = unspecified
+1 = forward
```

Validation rules:

- every pose must resolve to the declared route frame;
- pose positions must be finite and quaternions valid/normalized;
- consecutive duplicate or near-duplicate poses must be rejected or removed deterministically;
- inconsistent array lengths must reject the mission;
- unsupported direction values must reject the mission;
- a topology-version mismatch must reject the mission;
- geometry-only compatibility input must never invent route identity, edge identity or direction.

Primary mission state:

```text
/mission/state
parking_robot_interfaces/msg/MissionState
```

The state message must use typed constants/enums for:

```text
IDLE
RECEIVED
VALIDATING
PLANNING
NAVIGATING
PAUSED
CANCELLING
CANCELLED
SUCCEEDED
TEMPORARILY_BLOCKED
BLOCKED
FAILED
HELP_REQUIRED
```

Supporting control and diagnostic interfaces:

```text
/mission/start         std_srvs/srv/Trigger
/mission/cancel        std_srvs/srv/Trigger
/mission/pause         std_srvs/srv/SetBool
/mission/status        diagnostic_msgs/msg/DiagnosticStatus
/mission/block_reason  diagnostic_msgs/msg/DiagnosticStatus
```

Temporary compatibility input during early implementation:

```text
/mission_waypoints_compat
nav_msgs/msg/Path
```

The compatibility adapter may convert an unambiguous ordered path into `RouteMission`, but it cannot represent route ID, edge identity, direction or topology version. It must therefore never guess when geometry is ambiguous.

Plan_nav may continue publishing `/plan_nav` for display and comparison, but `/plan_nav` must be non-authoritative in the new mode.

### 13.5 Long-term evolution

After sequential goals are stable, evaluate:

```text
NavigateThroughPoses
```

using only sparse semantic route waypoints, not every dense trajectory sample.

The future company-platform adapter must translate company task, cancellation, return, and help/status messages into the generic Mission Manager interface. It is not implemented until the company supplies the formal protocol.

---

## 14. Nav2 planner architecture

### Selected initial global planner

```text
NavFnPlanner
use_astar: true
```

Reason:

- already configured and previously tested;
- sufficient for initial 2D occupancy/costmap navigation;
- easier to reproduce than changing planner and controller simultaneously.

### Planner responsibility

- compute a metric path between the current pose and the current semantic waypoint;
- avoid static and currently marked global-costmap obstacles;
- replan when the environment or goal changes;
- produce a Nav2 path, not a physical command.

### Later alternatives

Evaluate Smac 2D or another planner only after the full initial pipeline is stable and a specific deficiency is measured.

### Planner outcome requirement

Repeated path-planning failure must be reported to the Mission Manager as a structured blocked/failure reason rather than causing uncontrolled retries.

---

## 15. Nav2 controller architecture

### Selected controller

```text
MPPI Controller
DiffDrive motion model
```

### Responsibility

- track the Nav2 path;
- sample possible control sequences;
- score future trajectories;
- avoid local costmap obstacles;
- obey velocity and acceleration constraints;
- output raw Twist commands.

### Output topic

```text
/cmd_vel_nav_raw
geometry_msgs/msg/Twist
```

### Initial constraints

Start from conservative speeds validated in the recovered fake closed-loop work. Exact values should be tuned on the new Jetson and later calibrated physically.

Initial physical-mode policy:

- low maximum linear speed;
- low angular speed;
- reverse disabled initially;
- in-place rotation disabled or blocked until the CAN mapping is verified;
- acceleration limits enabled;
- no automatic high-speed recovery behavior.

### Controller comparison policy

MPPI remains the selected primary controller.

DWB is retained only as a later `OPTIONAL_EVALUATE` reference benchmark after MPPI is stable. Compare:

- path-completion rate;
- minimum obstacle clearance;
- oscillation;
- narrow-passage success;
- command smoothness;
- CPU load;
- controller-loop overruns;
- recovery count.

TEB is not part of the current implementation unless a measured requirement and a compatible maintained implementation justify it.

---

## 16. Collision Monitor architecture

### Position in command chain

```text
MPPI /cmd_vel_nav_raw
-> Collision Monitor
-> /cmd_vel_nav_safe
```

### Observation sources by phase

#### Phase 4 — offline/mock

Use a synthetic or replayed observation source to prove topic routing, lifecycle behavior, stop zones, slowdown zones, and timeout handling.

#### Phase 5-6 — temporary raw MID-360 profile

```text
/cloud_registered_body
sensor_msgs/msg/PointCloud2
frame: body
-> collision_monitor_phase6_raw_mid360.yaml
```

Before Phase 6, first verify that the installed ROS 2 Humble Collision Monitor version supports the intended `PointCloud2` source type and the exact required configuration schema. Do not copy a profile from another Nav2 version without validation.

Then verify with no physical output:

- stable cloud rate and timestamps;
- correct `body` frame and complete transform to `base_footprint`;
- floor points do not trigger false stops;
- wheelchair/self points are removed or excluded;
- obstacle-height and range filters are correct;
- source timeout produces a safe state;
- a stationary object triggers slowdown and stop zones;
- removal of the object clears the condition;
- latency and CPU use are acceptable.

This source exists only to support a limited clear-path command-interface test. It is not the final semantic obstacle source.

#### Phase 8 onward — final semantic source

```text
/obstacle_cloud
sensor_msgs/msg/PointCloud2
-> collision_monitor_semantic.yaml
```

### Optional later source

```text
/scan from YDLIDAR
```

### Responsibility

Collision Monitor is not the route planner. It is the final independent software stop/slow layer.

Initial zones should include:

- stop zone around the robot footprint;
- slowdown zone outside the stop zone;
- approach/time-to-collision behavior where stable;
- source timeout behavior that fails safely.

### Required authority rule

Only `/cmd_vel_nav_safe` may enter the Generic Command Safety Gate.

No chassis adapter may subscribe directly to `/cmd_vel_nav_raw`, and no node may bypass Collision Monitor in the new physical mode.

## 17. Generic Command Safety Gate

### Recommended package

```text
vehicle_cmd_safety
```

### Recommended node

```text
guarded_vehicle_cmd_gate
```

### Input

```text
/cmd_vel_nav_safe
geometry_msgs/msg/Twist
```

### Output

```text
/vehicle_cmd_safe
geometry_msgs/msg/TwistStamped
```

`TwistStamped` is selected so that downstream consumers can inspect the command publication time and configured command frame.

Because the Humble input is `geometry_msgs/msg/Twist`, the gate creates:

```text
header.stamp:
the ROS time when the gate accepted and published the command

header.frame_id:
configured command frame, normally base_footprint
```

Deadman and freshness decisions must use steady/monotonic receipt time, not only ROS time, because ROS time may pause or jump during bag playback or simulation.

If a future stamped velocity interface is adopted end-to-end, preserve the original source stamp and separately retain steady-clock receipt timing for watchdogs.

### Generic `/vehicle_cmd_safe` contract

`/vehicle_cmd_safe` uses ROS REP-103 conventions and a right-handed coordinate system.

```text
linear.x:
forward vehicle velocity in metres/second

angular.z:
yaw rate in radians/second

linear.y, linear.z:
must be zero for the selected ground-vehicle profile

angular.x, angular.y:
must be zero

header.frame_id:
base_footprint unless the active robot profile explicitly defines another base command frame
```

Rules:

- NaN and infinity are rejected.
- Unsupported lateral, vertical, roll or pitch commands are rejected.
- Reverse may be blocked by policy, but the gate must not silently change the sign.
- The chassis adapter must not silently reinterpret coordinate signs or units.
- Generic limits are expressed in SI units.
- Wheelchair-specific radius, millimetre and sign conventions exist only below this boundary.

### Generic responsibilities

- subscribe only to Collision Monitor's safe output;
- require explicit arming;
- require basic localization validity;
- require fresh command input;
- apply generic linear/angular speed limits;
- apply generic acceleration/slew limits where appropriate;
- reject NaN, infinity, malformed or stale commands;
- publish repeated zero command while disarmed, stale or faulted;
- publish startup and shutdown zero commands;
- expose clear state and fault diagnostics;
- refuse arming if command authority is ambiguous;
- publish at a configurable fixed heartbeat rate;
- monitor command authority continuously while armed, not only at startup.

### Required states

1. `MOCK`
2. `DISARMED`
3. `ARMED`
4. `FAULT`
5. `MANUAL_OVERRIDE` where detectable

### Explicit non-responsibilities

This node must not know:

- wheelchair turning-radius format;
- millimetre units;
- `Float32MultiArray` layout;
- current CAN ID or payload;
- negative-forward wheel convention;
- wheelchair minimum-radius encoding;
- future charging-robot protocol.

### Required gates

- `/system/localization_valid` is true and stable;
- Collision Monitor output is fresh;
- Nav2/controller state is valid;
- exactly one publisher owns `/vehicle_cmd_safe`;
- no invalid numerical values;
- explicit runtime arming;
- real-output mode is selected deliberately;
- publisher/writer ownership remains unique continuously while armed.

### Mock validation

During offline phases, either publish `/vehicle_cmd_safe` with no chassis adapter connected or connect only a mock adapter. No real chassis topic may be produced.

## 17A. Wheelchair Command Adapter

### Recommended package

```text
wheelchair_cmd_adapter
```

### Recommended node

```text
wheelchair_cmd_adapter_node
```

### Input

```text
/vehicle_cmd_safe
geometry_msgs/msg/TwistStamped
```

### Mock output

```text
/wheelchair_control_command_mock
std_msgs/msg/Float32MultiArray
```

### Armed real output

```text
/wheelchair_control_command
std_msgs/msg/Float32MultiArray
```

### Wheelchair-specific conversion

For ordinary curved motion:

```text
velocity_mm_s = linear.x * 1000
radius_mm = (linear.x / angular.z) * 1000
```

For near-straight motion:

```text
radius_mm = configured wheelchair straight value
velocity_mm_s = linear.x * 1000
```

For in-place rotation:

- default: reject and output stop until the current wheelchair mapping is verified;
- later: map to the existing radius-zero spin behavior only after a controlled test.

### Complete current wheelchair-message contract

Before real output, the adapter must reproduce the exact source-verified `/wheelchair_control_command` contract:

- required array length;
- meaning and units of `data[0]`;
- meaning and units of `data[1]`;
- meaning/default of `data[2]`;
- straight-motion encoding;
- turning-radius encoding;
- in-place rotation encoding;
- sign conventions;
- valid numeric ranges;
- stop encoding.

Required unit tests:

```text
stop
straight forward
gentle left
gentle right
maximum allowed command
reverse blocked
stale input
invalid NaN/Inf input
in-place turn rejected
```

### Independent adapter deadman

The Wheelchair Command Adapter must independently monitor `/vehicle_cmd_safe` using steady-clock receipt time.

If `/vehicle_cmd_safe` becomes stale, the adapter must repeatedly publish the wheelchair-specific stop command at a configured rate. It must never continue using the last nonzero command.

### Wheelchair-specific responsibilities

- metres/second to millimetres/second conversion;
- radius calculation;
- wheelchair-specific minimum-radius policy;
- current-chassis reverse and in-place-turn policy;
- current-chassis output topic selection;
- current-chassis command diagnostics;
- exactly one publisher on `/wheelchair_control_command`;
- independent input timeout/deadman;
- repeated stop heartbeat compatible with the existing controller timeout;
- continuous command-authority monitoring while real output is active.

### Reuse decision

Reuse the recovered conversion logic as evidence and test reference, but separate generic safety logic from wheelchair-specific formatting. Rewrite in C++ only if measured timing or reliability requires it.

## 17B. Future Charging-Robot Command Adapter

```text
/vehicle_cmd_safe
-> charging_robot_cmd_adapter
-> future command topic/protocol
-> future lower controller
```

This adapter is not implemented until the company provides the real chassis kinematics, command units, lower-controller protocol, watchdog behavior, and physical safety requirements.

## 17C. Generic Vehicle State and Feedback Interface

The reusable architecture needs a reverse information boundary in addition to `/vehicle_cmd_safe`.

Recommended topic:

```text
/vehicle_state
parking_robot_interfaces/msg/VehicleState
```

A plain boolean cannot distinguish `FALSE` from `UNKNOWN`. Therefore, status fields shall use explicit tri-state values and a validity mask.

Conceptual fields:

```text
std_msgs/Header header
string robot_id

uint8 STATE_UNKNOWN=0
uint8 STATE_FALSE=1
uint8 STATE_TRUE=2

uint64 valid_fields

uint8 controller_connected
uint8 drive_enabled
uint8 manual_override_active
uint8 emergency_stop_active
uint8 command_acknowledged

float32 measured_linear_velocity_mps
float32 measured_angular_velocity_radps
float32 battery_percent

uint32 fault_code
string fault_text
string interface_state
```

The interface package shall define validity-bit constants for all optional status and numeric fields. Consumers must use `valid_fields` before interpreting optional values.

Rules:

- unsupported or unavailable information remains `STATE_UNKNOWN` or invalid in `valid_fields`;
- `STATE_FALSE` may be used only when a real source explicitly confirms false;
- transmit success must not be reported as lower-controller command acknowledgement;
- numeric feedback must not be interpreted unless its validity bit is set;
- unavailable information must never be fabricated from assumptions.

Current wheelchair policy:

- the existing source appears transmit-oriented and may not provide lower-controller feedback;
- unsupported fields remain explicitly unknown;
- CAN-interface-open state may be reported separately from confirmed lower-controller connectivity;
- absence of lower-controller feedback does not block Phase 0–5 software work;
- absence of lower-controller feedback does not automatically block the supervised, low-speed Phase 6 command-interface test;
- Phase 6 must record unavailable feedback explicitly and rely on the confirmed joystick override, physical emergency stop, localization gate, Collision Monitor, both software deadmen, repeated-zero behavior, single-authority enforcement and direct operator supervision;
- missing feedback remains a production-readiness limitation;
- future receive/CAN-status integration is a `CORE_LATER_GATE`.

Future charging-robot policy:

- a product chassis adapter must publish the available generic vehicle state;
- Mission Manager and Health Supervisor consume generic state, not chassis-private messages;
- company-specific feedback stays below the adapter boundary;
- product release requires a formally defined feedback, fault and acknowledgement contract.

## 18. Existing CAN layer

### Preserve unchanged initially

```text
/wheelchair_control_command
-> wheelchair_controller_node
-> SocketCAN can0
-> lower controller
```

### Existing key protocol

- output transport defaults to CAN;
- interface `can0`;
- extended ID `0x801400`;
- DLC 8;
- int16 little-endian fields;
- 20 ms timer;
- 500 ms stale command timeout;
- negative wheel values used for forward in current convention;
- startup paused unless explicitly auto-started.

### Initial new-mode policy

- do not modify frame layout;
- do not change CAN settings;
- do not change sign conventions;
- do not enable auto-start;
- launch the CAN node separately from perception/navigation;
- retain joystick override as the primary human intervention mechanism during supervised testing.

---

## 19. YDLIDAR policy

### Initial implementation

- preserve current source and launch files;
- do not modify its path-avoidance or command-filter code;
- do not launch those nodes in the new Nav2 semantic mode;
- keep them available only in the legacy baseline mode.

### Later evaluation

After MID-360 semantic/Nav2 control is stable, evaluate YDLIDAR as:

```text
/scan
-> Collision Monitor secondary source
```

or:

```text
/scan
-> independent close-range stop/slow gate
```

Do not initially use it to generate detour paths that compete with Nav2.

---

## 20. Stereo-camera policy

### Initial decision

The stereo/RealSense camera is not required for the first simple Nav2 navigation baseline and is not required for the first MID-360 semantic obstacle-avoidance milestone.

Keep the camera disabled while proving:

```text
map loading
-> localization
-> start pose
-> goal submission
-> NavFn path
-> MPPI control
-> mock command
```

### Primary later role

The preferred later operating concept is:

```text
normal navigation:
LiDAR-dominant FAST-LIO + RTAB-Map localization

localization degraded/lost:
activate or increase visual-assisted relocalization processing

stable localization recovered:
return to normal LiDAR-dominant navigation
```

Continuous visual processing may still be used during mapping if required, but it must not block or starve navigation control.

The camera should first be evaluated for:

- visual loop closure;
- relocalization in repetitive or drift-prone areas;
- map completeness;
- robustness when LiDAR geometry is weak;
- comparison of LiDAR-only versus LiDAR-plus-stereo RTAB-Map;
- recovery after localization degradation.

### Is the camera required for navigation?

No. Nav2 navigation can operate using the map, localization, MID-360-derived costmaps, and MPPI without the camera.

The camera may indirectly improve navigation by improving localization and map quality, but it must not become a hidden dependency of basic path following.

### Is the camera required for obstacle avoidance?

No for the initial system. The MID-360 plus semantic processing is the primary obstacle source.

Later, stereo depth or visual detection may be added as another costmap observation source for specific blind spots or object classes, but only after:

- calibration is verified;
- timing and TF are correct;
- compute cost is acceptable;
- false positives and stale depth are controlled;
- measured benefit is greater than complexity.

### Activation sequence

1. Core navigation without camera.
2. Semantic MID-360 obstacle avoidance without camera.
3. LiDAR-only mapping/localization benchmark.
4. Camera-assisted mapping/relocalization benchmark.
5. Keep camera only if it measurably improves robustness.
6. Evaluate camera obstacle input only as a separate later experiment.

## 20A. Ultrasonic and future near-field sensor policy

No ultrasonic hardware is confirmed in the current setup.

Therefore:

- do not implement or simulate a fake production dependency now;
- reserve a standard `sensor_msgs/msg/Range` interface;
- allow a future Range Sensor Layer or safety-supervisor input;
- keep ultrasonic calibration, mounting, field of view, and timeout rules in a future robot-model profile;
- activate only when actual charging-robot hardware is available.

Status:

```text
HARDWARE_FUTURE
```

## 20B. Navigation Constraint Layer policy

### Final requirement

The final product architecture includes:

- keepout zones;
- virtual walls;
- temporary restricted areas;
- speed-limit zones;
- map masks;
- site-specific parking/charging rules.

### Initial development state

```text
required in final architecture
documented now
implemented later
disabled by default in core baseline
```

### Activation gate

Begin constraint-layer implementation only after:

- simple localization and Nav2 goal navigation pass;
- MPPI mock command path passes;
- guarded adapter mock tests pass;
- semantic costmaps and offline obstacle avoidance pass;
- at least one clear-path physical Nav2-to-current-CAN interface test passes;
- physical semantic obstacle avoidance is stable enough to separate constraint errors from perception/control errors.

### Test order

1. Synthetic rectangular keepout zone.
2. Virtual wall.
3. Speed-limit zone.
4. Temporary restriction.
5. Multiple overlapping filters.
6. Actual parking/charging-site masks and operating rules.

### Configuration separation

Navigation Constraint and site-policy overlays:

```text
nav2_core.yaml
nav2_semantic.yaml
nav2_constraints_test.yaml
nav2_site_<site_id>.yaml
```

RViz profiles are maintained separately under the visualization package, as defined in Section 20D and the package/file structure.

Do not put half-configured future filters into the core YAML.

## 20C. Chassis independence and fleet-scale policy

The current wheelchair is one hardware profile, not the permanent product architecture.

### Reusable layers

- mission manager;
- FAST-LIO/RTAB-Map integration;
- semantic perception;
- Nav2 planners/controllers;
- Collision Monitor;
- health monitoring;
- logging and validation;
- constraint-layer interfaces.

### Per-robot or per-model profiles

- footprint;
- wheelbase/track;
- motion model;
- maximum speed and acceleration;
- sensor extrinsics;
- camera and LiDAR availability;
- vehicle-command conversion;
- lower-controller protocol;
- stop/arm behavior.

### Calibration manifest

Each robot must have a versioned calibration manifest identifying:

- `body -> base_footprint`;
- `base_footprint/base_link -> livox_frame`;
- camera intrinsics and extrinsics;
- optional YDLIDAR and ultrasonic extrinsics;
- robot footprint;
- wheelbase/track and kinematic constants;
- calibration method, date and operator;
- source files, version and checksum;
- robot serial/model applicability.

A software or site release must record the calibration-manifest version it expects.

### Per-site profiles

- map/database;
- keepout mask;
- speed mask;
- charging locations;
- route policies;
- restricted areas;
- localization initialization data.

### Productization requirements represented now, implemented later

- no hard-coded absolute user paths;
- configuration version IDs;
- robot-model and site-profile selection;
- diagnostics and health state;
- reproducible build/release;
- rollback;
- log and bag metadata;
- future fleet-manager interface;
- staged rollout and canary testing before deploying to hundreds or thousands of robots;
- robot identity and authenticated platform communication;
- access control and credential/certificate management;
- signed or otherwise verified software/configuration releases;
- secure update and rollback;
- DDS/network exposure policy;
- software bill of materials and open-source license review;
- dependency/version pinning;
- security-event logging;
- camera and operational-data retention policy.


## 20D. Unified RViz visualization and debug deliverable

The project shall provide one reusable visualization system through multiple RViz profiles.

Recommended files:

```text
parking_robot_minimal.rviz
parking_robot_navigation_debug.rviz
parking_robot_perception_debug.rviz
parking_robot_full_presentation.rviz
```

Expected content, enabled according to profile:

- RobotModel and TF;
- map and robot pose;
- FAST-LIO odometry/path;
- raw registered clouds;
- `/ground_cloud`, `/obstacle_cloud`, `/semantic_grid`;
- global/local costmaps;
- global plan and local/controller trajectory where available;
- mission goals and topological waypoints;
- footprint;
- Collision Monitor zones;
- localization/health state;
- later keepout and speed masks.

A sensor or subsystem is not considered integrated merely because its node starts. Its required data must be available, correctly framed, visualized and validated.

Expensive raw/debug displays must remain disabled in the minimal runtime profile.

## 20E. Gazebo and validation-track policy

Gazebo is a supporting validation environment, not a second autonomy architecture.

The same interfaces must be reused:

```text
Mission Manager
Nav2
MPPI
Collision Monitor
/vehicle_cmd_safe
```

Recommended validation ladder:

1. unit tests;
2. fake closed loop;
3. rosbag replay;
4. minimal Gazebo closed loop;
5. live sensors with no physical commands;
6. supervised physical robot.

The initial Gazebo model may use an approximate differential-drive robot with measured footprint and approximate kinematics. It should improve only when a simulation deficiency blocks useful testing.

Gazebo does not replace bag replay or physical testing.

## 20F. Runtime supervision and controlled recovery

### Early required supervision

Before physical autonomy:

- process/node presence;
- expected topic/message freshness;
- TF age;
- lifecycle state;
- odometry/localization freshness;
- command freshness;
- controller and Collision Monitor activity;
- latency budget reporting.

On fault:

```text
pause mission
-> zero /vehicle_cmd_safe
-> report fault
```

### Later controlled restart

After the core system is stable:

- freeze detection;
- dependency-aware restart order;
- restart limits;
- cooldown;
- localization revalidation;
- mission resume only after stability;
- escalation to `HELP_REQUIRED` after repeated failure.

A critical node must never be restarted while movement continues.

## 20G. Meeting-derived performance target

The meeting mentioned an approximately 100 ms navigation/safety responsiveness target. Because the transcript is not a formal specification, record this as:

```text
MEETING-DERIVED TARGET — REQUIRES COMPANY/PROFESSOR CONFIRMATION
```

Measure separately:

- sensor timestamp to semantic obstacle output;
- obstacle output to costmap;
- costmap/control update to MPPI response;
- MPPI to Collision Monitor output;
- safe Twist to `/vehicle_cmd_safe`;
- `/vehicle_cmd_safe` to chassis command.

The final acceptance budget must be confirmed before product release.

## 20H. Ownership and interface boundaries

| Area | Primary owner | Our integration responsibility |
|---|---|---|
| plan_nav GUI/topology and overlapping-route selection | plan_nav collaborator | define and validate unambiguous mission contract |
| LiDAR–vision fusion and visual relocalization implementation | vision/localization collaborator | preserve interfaces, monitor health/load, integrate status |
| unified RViz/debug interface | user/current workstream | implement and maintain profiles |
| semantic/Nav2/Collision Monitor integration | current autonomy workstream | implement and validate |
| company task/state protocol | company/specification owner | provide generic platform-adapter boundary |
| future chassis protocol | company/hardware team | preserve `/vehicle_cmd_safe` adapter boundary |
| charging/docking workflow | future application team/specification | keep mission interface compatible |

## 20I. Map-editing and versioning policy

Prefer non-destructive overlays before modifying the base map.

Order:

1. keepout/speed/temporary masks;
2. preserve original map and database;
3. edit the static map only when the map itself is wrong;
4. version every edited map;
5. record tool, operator, date, reason and checksum;
6. validate navigation after editing;
7. support rollback to the previous map.

# Part III - Old versus new comparison

## 21. Architecture comparison table

| Section | Existing architecture | Selected revised architecture | Status |
|---|---|---|---|
| Mission source | plan_nav GUI | plan_nav first; charging task/GPS/API/fleet later | `CORE_NOW` + interfaces |
| Route selection | plan_nav Dijkstra | retained as topological mission planning | `CORE_NOW` |
| Mission execution | dense `/plan_nav` to Pure Pursuit | mission manager sends standard Nav2 goals | `CORE_NOW` |
| Global planner | NavFn configured but bypassed physically | NavFn A* authoritative | `CORE_NOW` |
| Local controller | custom Pure Pursuit authoritative | MPPI authoritative in new mode | `CORE_NOW` |
| Legacy fallback | current plan_nav/Pure Pursuit/CAN | preserved separately | `LEGACY_BASELINE` |
| Primary odometry | FAST-LIO | retained | `CORE_NOW` |
| Mapping/localization | RTAB-Map | retained and formalized | `CORE_NOW` |
| Basic localization-valid gate | informal verification | binary validity gate required before Phase 6 | `CORE_NOW` |
| Advanced relocalization health | incomplete | recovery/state subsystem and stereo comparison | `CORE_LATER_GATE` |
| Main obstacle input | raw cloud parallel branch; YDLIDAR custom avoidance | MID-360 semantic `/obstacle_cloud` | `CORE_LATER_GATE` |
| Final stop/slow | current YDLIDAR filter in legacy path | Collision Monitor authoritative | `CORE_NOW` before real arming |
| Generic command safety | absent as a reusable boundary | `guarded_vehicle_cmd_gate` publishes `/vehicle_cmd_safe` | `CORE_NOW` |
| Wheelchair command conversion | custom Pure Pursuit output | separate `wheelchair_cmd_adapter` | `CORE_NOW` |
| Current CAN driver | wheelchair-specific SocketCAN | retained as current hardware adapter | `CORE_NOW` |
| Future chassis | not abstracted | replaceable charging-robot adapter | `HARDWARE_FUTURE` |
| Navigation constraints | not central | keepout, virtual wall, speed and site masks | `CORE_LATER_GATE` |
| Stereo camera | integrated/optional, benefit unclear | later mapping/relocalization evaluation | `OPTIONAL_EVALUATE` |
| Camera obstacle use | unclear | optional later depth/vision source | `OPTIONAL_EVALUATE` |
| YDLIDAR | current custom detour/filter | preserve; optional later stop/slow | `OPTIONAL_EVALUATE` |
| Ultrasonic | day-zero requirement only | standard future Range interface | `HARDWARE_FUTURE` |
| Charging functions | not specified | future application/mission layer | `HARDWARE_FUTURE` |
| Fleet deployment | absent | profiles, diagnostics, release and rollback interfaces | `PRODUCTIZATION_LATER` |
| Unified RViz/debug | informal/scattered | reusable minimal/debug/presentation profiles | `CORE_NOW` |
| Mission state machine | incomplete/distributed | one generic Mission Manager state machine | `CORE_NOW` |
| Blocked/help outcome | not formalized | bounded recovery, `BLOCKED`, future `HELP_REQUIRED` | `CORE_LATER_GATE` |
| Route identity | geometry/path only may be ambiguous | explicit route/edge/direction metadata | collaborator dependency |
| Gazebo | not established | compatible supporting validation track | `OPTIONAL_EVALUATE` |
| Runtime supervision | logs/diagnostics only | freshness/freeze detection; later controlled restart | `CORE_NOW` + later gate |
| System image recovery | not formalized | versioned full Jetson image and restore procedure | `CORE_NOW` |
| 3D navigation | potentially ambiguous scope | explicit non-goal; use 3D perception for 2D/2.5D ground navigation | scope boundary |
| Typed mission contract | path/string interfaces are insufficient | `RouteMission` and typed `MissionState` | `CORE_NOW` |
| Vehicle feedback boundary | absent/inconsistent | generic `/vehicle_state`, unsupported fields explicit | `CORE_LATER_GATE` |
| Clock/timestamp policy | implicit | explicit live/bag/watchdog synchronization policy | `CORE_NOW` |
| Calibration manifest | scattered parameters | versioned per-robot calibration artifact | `CORE_NOW` |
| Product security | not represented | identity, authentication, secure release, SBOM and logging | `PRODUCTIZATION_LATER` |


# Part IV - Software organization

## 22. Recommended package and file structure

```text
fastlio_rtabmap_ros2_ws/
├── src/
│   ├── FAST_LIO_ROS2/                    # retain
│   ├── livox_ros_driver2/                # retain
│   ├── rtabmap_ros/                      # retain
│   ├── robot_bringup/                    # retain; legacy launch preserved
│   ├── parking_robot_bringup/            # new generic new-mode launches/profiles
│   ├── parking_robot_interfaces/         # RouteMission, MissionState, VehicleState
│   ├── wheelchair_controller/            # retain current chassis CAN adapter
│   ├── semantic_grid_tools/              # port recovered package
│   ├── parking_robot_mission_manager/    # new generic mission layer
│   ├── vehicle_cmd_safety/               # new generic safety gate
│   ├── wheelchair_cmd_adapter/           # new current-chassis conversion
│   ├── navigation_health_monitor/        # freshness, freeze and recovery supervision
│   ├── platform_adapter/                 # future company protocol adapter
│   ├── parking_robot_description/        # URDF/xacro and simulation model
│   ├── parking_robot_visualization/      # RViz profiles
│   ├── navigation_constraints/           # deferred filters/masks/profile tools
│   ├── parking_robot_gazebo/              # minimal compatible simulation track
│   ├── ydlidar_ros2_driver/              # preserve
│   └── YDLidar-SDK/                      # preserve
├── plan_nav/                             # authoritative current GUI/topology
├── config/
│   ├── robot_models/
│   │   ├── wheelchair_dev.yaml
│   │   └── future_charging_robot.yaml.example
│   ├── calibration_manifests/
│   │   └── wheelchair_dev_calibration.yaml
│   ├── sensor_profiles/
│   ├── nav_profiles/
│   └── site_profiles/
├── map/
├── maps/
├── scripts/
├── docs/
└── tests/
```

### New launch files

```text
parking_robot_bringup/launch/core_nav2_offline.launch.py
parking_robot_bringup/launch/core_nav2_live_no_motion.launch.py
parking_robot_bringup/launch/semantic_nav2_offline.launch.py
parking_robot_bringup/launch/semantic_nav2_live_no_motion.launch.py
parking_robot_bringup/launch/nav2_can_interface_test.launch.py
parking_robot_bringup/launch/semantic_nav2_real.launch.py
parking_robot_bringup/launch/navigation_constraints_test.launch.py
parking_robot_gazebo/launch/minimal_nav2_sim.launch.py
```

### Configuration overlays

```text
nav2_core.yaml
nav2_semantic.yaml
collision_monitor_phase6_raw_mid360.yaml
collision_monitor_semantic.yaml
vehicle_cmd_gate_mock.yaml
vehicle_cmd_gate_real.yaml
wheelchair_cmd_adapter_mock.yaml
wheelchair_cmd_adapter_real.yaml
clock_sync_policy.yaml
vehicle_state_profile.yaml
nav2_constraints_test.yaml
site_<site_id>.yaml
rviz/parking_robot_minimal.rviz
rviz/parking_robot_navigation_debug.rviz
rviz/parking_robot_perception_debug.rviz
rviz/parking_robot_full_presentation.rviz
```

### Naming rule

Use generic names for newly created autonomy packages. Keep wheelchair-specific naming only for the existing current hardware driver and current-chassis calibration.

### Preserve current baseline

Do not overwrite `bringup_2d.launch.py` or the legacy plan_nav/Pure Pursuit path at the beginning. Document it as a baseline and rollback mode.


## 23. Launch modes

### Mode A — Legacy current-chassis baseline

```text
LEGACY_PLAN_NAV_CAN
```

Current plan_nav, current Pure Pursuit, current optional YDLIDAR path/filter, current wheelchair controller and CAN.

### Mode B — Core Nav2 offline

```text
CORE_NAV2_OFFLINE
```

Map/localization substitute or fake odometry, simple Nav2 goals, NavFn, MPPI, mock command. No semantic perception, constraints, live sensors or CAN.

### Mode C — Core live sensors, no motion

```text
CORE_NAV2_LIVE_NO_MOTION
```

MID-360, FAST-LIO, RTAB-Map, the basic localization-valid gate, Nav2, Mission Manager, the temporary raw-MID-360 Collision Monitor profile, Generic Command Safety Gate, and mock Wheelchair Command Adapter. No wheelchair controller.

### Mode D — Semantic Nav2 offline

```text
SEMANTIC_NAV2_OFFLINE
```

Recorded clouds, semantic node, semantic costmaps, Nav2 fake closed loop, Collision Monitor, Generic Command Safety Gate, and mock Wheelchair Command Adapter.

### Mode E — Semantic live sensors, no motion

```text
SEMANTIC_NAV2_LIVE_NO_MOTION
```

Complete perception/localization/navigation chain through the Generic Command Safety Gate and mock Wheelchair Command Adapter; mock output only.

### Mode F — Limited clear-path CAN interface test

```text
NAV2_CAN_INTERFACE_TEST
```

Very low speed, no complex obstacle test, explicit arming, one publisher, current CAN driver, supervised clear area.

### Mode G — Guarded semantic physical mode

```text
SEMANTIC_NAV2_REAL
```

Validated semantic costmaps, MPPI, semantic Collision Monitor profile, Generic Command Safety Gate, Wheelchair Command Adapter, and current wheelchair hardware driver.

### Mode H — Navigation constraints test

```text
NAVIGATION_CONSTRAINTS_TEST
```

Synthetic/existing maps first; keepout, virtual walls and speed filters. Physical use only after offline pass.

### Mode I — Site profile mode

```text
SITE_<SITE_ID>_REAL
```

Actual parking/charging-site map, constraint masks, robot model, sensor profile and operating rules. Created only when real site specifications exist.


### Mode J — Minimal Gazebo validation

```text
GAZEBO_NAV2_VALIDATION
```

Approximate differential-drive robot, simulated LiDAR, static/dynamic obstacles, same Mission Manager, Nav2, MPPI, Collision Monitor and `/vehicle_cmd_safe` interfaces.

### Mode K — Future company-platform integration

```text
PLATFORM_ADAPTER_INTEGRATION
```

Company protocol adapter, generic mission command/state interface and mock autonomy stack. Created only after formal company message definitions are available.

# Part V - Revised implementation strategy

## 24. Phase 0 — Preservation, day-zero reconciliation, and source control

### Tasks

1. Preserve the current project and previous strategy unchanged.
2. Save day-zero PDF/DOCX/README as historical requirements, not source authority.
3. Create a requirement matrix: original requirement, current evidence, selected status, activation gate, acceptance criterion.
4. Create checksums for source archive, maps, databases, topology and calibration.
5. Initialize Git on the new Jetson and commit the untouched baseline.
6. Mark authoritative source versus backup/legacy/generated data.
7. Create robot-model and site-profile templates without inventing unknown values.
8. Create a full recoverable image of the known-working Jetson.
9. Record Jetson model, JetPack/OS/ROS versions, partition/image method, image checksum, storage location and restore procedure.
10. Create an ownership/interface matrix and open-requirement register from the meeting.
11. Create the module registry: module, owner, status, evidence, dependency, activation gate and acceptance criterion.
12. Create the first versioned calibration manifest without inventing unknown measurements.
13. Record the live/bag/simulation clock and timestamp policy.
14. Record product-security and software-supply-chain items as `PRODUCTIZATION_LATER`.
15. Record full free-space 3D navigation as outside the current scope.

### Pass criteria

- immutable baseline archived;
- historical requirements indexed;
- current-source facts take precedence over outdated README statements;
- first Git commit and checksum manifest exist;
- no implementation changes mixed into preservation;
- known-working Jetson system image exists;
- image checksum and restore procedure are documented;
- ownership and open-interface requirements are indexed;
- complete module registry exists;
- calibration manifest exists and unknown fields are explicit;
- clock/timestamp policy is documented;
- product-security requirements are registered without blocking current navigation.

## 25. Phase 1 — Native Humble build and authoritative baseline

### Tasks

- rebuild without stale `build/`, `install/`, cache or old logs;
- parameterize hard-coded paths;
- verify packages, maps, RTAB-Map database, plan_nav and TF configuration;
- verify current frame truth `map -> odom -> body -> base_footprint`;
- verify all possible command publishers statically;
- keep sensors and CAN disconnected or command nodes absent;
- create the initial RViz profile for TF, robot model, map, pose and basic health;
- verify Collision Monitor plugin/source capabilities for the installed Humble version.

### Pass criteria

- clean build;
- launch inspection succeeds;
- no duplicate package;
- no physical command publisher;
- map/database and plan_nav assets readable.

## 26. Phase 2 — Core simple Nav2 navigation baseline

### Scope

No semantic perception, no constraint filters, no camera, no YDLIDAR, no ultrasonic, no real CAN.

```text
map / fake localization
-> initial pose
-> simple goal
-> NavFn A*
-> MPPI
-> mock Twist
-> fake odometry
```

### Tests

- start pose;
- one simple goal;
- sequential simple goals;
- goal cancel;
- planner failure;
- controller timeout;
- TF failure;
- fake closed-loop goal completion;
- visualize goals, global path, robot pose, commands and basic health;
- exercise basic mission states and cancellation.

### Pass criteria

- Nav2 reaches simple goals reproducibly;
- global path and MPPI commands are sensible;
- failures stop or abort cleanly;
- no real command topic exists.

## 27. Phase 3 — Mission manager and plan_nav decoupling

### Tasks

- keep plan_nav topology and Dijkstra;
- add sparse ordered mission-waypoint output;
- create `parking_robot_interfaces` and `parking_robot_mission_manager`;
- implement typed `RouteMission` and `MissionState`;
- keep `nav_msgs/Path` only through a temporary compatibility adapter;
- send sequential `NavigateToPose` goals;
- implement pause, cancel, failure and progress;
- keep dense `/plan_nav` display-only in new mode;
- define the route identity/direction contract;
- reject ambiguous route selections;
- keep full GUI ambiguity resolution collaborator-owned.

### Pass criteria

- plan_nav no longer controls velocity in new mode;
- typed unambiguous map-frame route reaches Nav2;
- geometry-only compatibility input is rejected when ambiguous;
- failed waypoint does not silently skip;
- mission cancel stops command generation.

## 28. Phase 4 — MPPI authority, Collision Monitor, Generic Command Safety Gate, and mock chassis adapter

### Tasks

```text
MPPI /cmd_vel_nav_raw
-> Collision Monitor
-> /cmd_vel_nav_safe
-> guarded_vehicle_cmd_gate
-> /vehicle_cmd_safe
-> mock wheelchair_cmd_adapter
-> /wheelchair_control_command_mock
```

- separate raw, Collision-Monitor-safe, generic-safe, and chassis-mock topics;
- use synthetic or replayed obstacle zones;
- implement generic arming, freshness, numerical validation, generic limits and repeated zero output;
- implement wheelchair-specific conversion only in the mock wheelchair adapter;
- block in-place rotation until current-chassis mapping is verified;
- enforce one publisher for each authority topic;
- verify no real chassis topic exists;
- implement the basic generic mission state machine;
- implement bounded progress timeout and blocked-state reporting;
- start a minimal Gazebo validation track only after fake closed-loop success.

### Pass criteria

- Collision Monitor stops/slows correctly;
- `/vehicle_cmd_safe` contains only valid, filtered commands;
- wheelchair-specific conversion occurs only downstream of `/vehicle_cmd_safe`;
- real output is impossible in default mode;
- stale, invalid, disarmed or fault state produces repeated zero command;
- the mock wheelchair output matches expected radius/velocity conversion.

## 29. Phase 5 — Core live sensors, basic localization validity, and temporary Collision Monitor source; no motion

### Tasks

- connect MID-360 and IMU;
- run FAST-LIO and RTAB-Map;
- verify TF, timestamps, topic rates and localization;
- implement the minimum binary localization-valid gate;
- validate `/cloud_registered_body` as the temporary Phase 6 Collision Monitor source;
- run simple Nav2 goals in observation/mock mode;
- run the full command chain through `/vehicle_cmd_safe` and the mock Wheelchair Command Adapter;
- measure CPU, memory, thermal behavior and latency;
- keep camera, YDLIDAR and real CAN output disabled;
- activate early node/topic freshness supervision;
- record per-stage latency and compare against the provisional meeting-derived target;
- extend the unified live RViz profile.

### Minimum localization-valid checks

Mandatory:

- `/Odometry` age below threshold;
- `map -> odom` exists and is fresh;
- complete `map -> base_footprint` transform resolves at the command time;
- finite position and valid normalized quaternion;
- translation and rotation jump below configured thresholds;
- validity remains stable for a configured arming window.

Configurable when valid map metadata/site policy is available:

- robot pose inside map bounds;
- pose not clearly inside an occupied/invalid region;
- unknown-space policy;
- minimum clearance from lethal/inflated cells.

A loop-closure or relocalization jump pauses the mission and clears validity until the stability window passes. Controlled resume/re-arming is required.

Output:

```text
/system/localization_valid
/system/localization_reason
/system/pose_age
/system/tf_age
/system/pose_jump_detected
```

Invalid state must pause the mission, refuse real arming and force `/vehicle_cmd_safe` to zero.

### Temporary raw-MID-360 Collision Monitor validation

Use:

```text
/cloud_registered_body
-> collision_monitor_phase6_raw_mid360.yaml
```

Confirm:

- floor and self points do not create persistent false stops;
- height/range filters are correct;
- obstacle insertion and removal behave predictably;
- stale cloud causes safe state;
- source rate, transform availability, latency and CPU are acceptable.

### Pass criteria

- stable `odom -> body` and `map -> odom`;
- `/system/localization_valid` switches correctly under induced faults;
- mock commands are zero whenever localization is invalid;
- temporary Collision Monitor source passes no-motion object tests;
- mock paths and commands remain reasonable;
- no thermal or timing failure;
- no CAN writer exists.

## 29A. Phase 5B — Reproduce the current physical legacy baseline

### Purpose

Verify that the transferred/new Jetson, current project assets, current CAN driver and lower controller still reproduce the user-confirmed working plan_nav/Pure Pursuit behavior before introducing new Nav2 physical authority.

This phase does not approve the legacy obstacle-avoidance behavior as the final solution.

### Preconditions

- Phases 0–5 pass;
- exact legacy launch mode is isolated from all new command nodes;
- joystick override and physical emergency stop are available;
- one `/wheelchair_control_command` publisher and one CAN writer;
- low-speed supervised test area;
- exact map/database, topology and controller configuration recorded.

### Record before launch

- source commit/checksums;
- map and RTAB-Map database;
- plan_nav topology and route;
- launch commands and order;
- whether YDLIDAR avoidance is enabled;
- Pure Pursuit parameters;
- Wheelchair Controller parameters;
- speed limits and arm/start procedure.

### Tests

- startup and stop;
- short straight motion;
- gentle left and right;
- route start-to-end with known intermediate waypoints;
- cancel/stop;
- joystick override;
- shutdown;
- no competing new-mode publishers.

### Pass criteria

- known legacy direction and scaling are reproduced;
- the selected known route is repeatable at low speed;
- stop, cancel, joystick override and shutdown work;
- exact evidence and configuration are archived;
- result is tagged `legacy_physical_baseline_pass`.

If this phase fails, diagnose transfer/CAN/lower-controller/baseline issues before blaming the new Nav2 adapter.

## 30. Phase 6 — Limited low-speed Nav2-to-current-CAN interface validation

### Purpose

Validate command authority, generic/chassis adapter separation, and current wheelchair response. This is not final semantic autonomy or final collision-performance validation.

### Temporary observation source

```text
/cloud_registered_body
-> collision_monitor_phase6_raw_mid360.yaml
-> /cmd_vel_nav_safe
```

This source must have passed the Phase 5 no-motion floor/self-filter, timeout, object-trigger and clearing checks.

If the raw cloud cannot be made reliable enough for this limited test, reduce Phase 6 to stop-only and very short command-interface checks, or postpone movement until the semantic source is available. Do not claim obstacle protection from an unvalidated source.

### Command chain

```text
MPPI
-> Collision Monitor
-> /cmd_vel_nav_safe
-> Generic Command Safety Gate
-> /vehicle_cmd_safe
-> Wheelchair Command Adapter
-> /wheelchair_control_command
-> existing wheelchair_controller_node
-> CAN
```

### Preconditions

- Phases 0–5 and Phase 5B pass;
- `/system/localization_valid` is true for the configured stability window;
- unavailable lower-controller feedback is documented explicitly and is not misrepresented as healthy feedback;
- missing feedback is accepted only for this supervised, low-speed development test and is not treated as product readiness;
- joystick override confirmed;
- explicit low-speed profile;
- exactly one publisher on `/vehicle_cmd_safe`;
- exactly one publisher on `/wheelchair_control_command`;
- exactly one CAN-writing process;
- temporary Collision Monitor profile active and fresh;
- supervisor and clear area.

### Tests

- stop only;
- short straight motion;
- stop;
- gentle left/right;
- cancel;
- deadman;
- shutdown;
- forced localization-invalid test while stationary or under the safest feasible condition;
- joystick override;
- one short clear-path Nav2 goal.

### Pass criteria

- direction and scaling are correct;
- all stop paths work;
- localization invalidity refuses or removes real command authority;
- no legacy publisher competes;
- clear-path goal is stable;
- result is documented as command-interface validation only;
- unavailable lower-controller feedback is recorded as a known limitation;
- no claim is made for product-ready feedback, command acknowledgement, semantic obstacle avoidance or final collision performance.

## 31. Phase 7 — Port semantic_grid_tools in isolation

### Tasks

- port recovered package;
- use `/cloud_registered` in `odom`;
- preserve timestamps and frames;
- add bounded memory, stale data diagnostics, reset/decay policy and processing metrics;
- replay known bags;
- compare with recovered reference evidence.

### Pass criteria

- ground/obstacle/grid outputs are correct;
- no frame relabeling without transform;
- no unbounded growth;
- long replay remains stable;
- compute load measured.

## 32. Phase 8 — Semantic costmaps and offline obstacle avoidance

### Tasks

- add semantic `/obstacle_cloud` to local costmap first;
- validate marking, clearing, inflation and stale behavior;
- add global costmap after local pass;
- compare semantic versus raw cloud;
- run fake closed-loop static, narrow-passage and sudden-obstacle tests;
- use MPPI and Collision Monitor;
- keep real CAN absent;
- test `PASSABLE`, `TEMPORARILY_BLOCKED`, `BLOCKED`, and `HELP_REQUIRED` internal outcomes;
- enforce bounded recovery attempts and progress timeout.

### Pass criteria

- ground is not broadly marked;
- obstacles appear in correct place;
- moved obstacles clear;
- fake robot avoids obstacles or stops safely;
- resource use remains acceptable.

## 33. Phase 9 — Semantic live sensors, no motion

### Tasks

- run live MID-360, FAST-LIO, RTAB-Map, semantic node, Nav2, MPPI, Collision Monitor and mock adapter;
- move manually where appropriate;
- record end-to-end obstacle-to-safe-command latency;
- test sensor loss, stale cloud, localization degradation and restart;
- keep camera disabled for baseline comparison.

### Pass criteria

- semantic costmaps remain timely and stable;
- safe commands respond to real obstacles;
- health/fault states are visible;
- no real command output.

## 34. Phase 10 — Physical semantic obstacle avoidance

### Preconditions

All previous phases pass, including the clear-path CAN interface validation.

### Tests

1. Large soft static obstacle.
2. Wide detour.
3. Narrower but safe route.
4. Standing person.
5. Slowly moving obstacle.
6. Sudden obstacle requiring Collision Monitor stop.
7. Mission cancel and resume.
8. Persistent blocked obstacle leading to `BLOCKED`.
9. Repeated unrecoverable failure leading to generic `HELP_REQUIRED`.

### Pass criteria

- detection before unsafe distance;
- path avoids lethal cells or stops;
- Collision Monitor independently protects close approach;
- goal is reached or mission stops safely;
- repeated runs are consistent.

## 35. Phase 11 — Navigation Constraint Layer

### Initial state

Required but disabled until this phase.

### Tests

1. Synthetic keepout mask.
2. Virtual wall.
3. Speed zone.
4. Temporary restriction.
5. Multiple filter interaction.
6. Versioned map-editing workflow.
7. Actual site constraints later.

### Pass criteria

- global path never enters keepout cells;
- speed zone changes only speed, not localization;
- temporary restriction can be changed predictably;
- core navigation still works when filters are removed;
- constraints use separate overlays and do not contaminate `nav2_core.yaml`;
- original map remains recoverable;
- edited maps are versioned, validated and rollback-capable.

## 36. Phase 12 — Relocalization, mapping robustness, and stereo-camera evaluation

### Tasks

- extend the Phase 5 binary validity gate into full localization-health and recovery states;
- test startup away from mapped origin;
- test initial-pose recovery;
- test RTAB-Map relocalization after loss;
- compare LiDAR-only versus LiDAR-plus-stereo;
- measure loop closures, drift, relock time, map completeness and compute cost;
- evaluate camera obstacle input only after localization evaluation;
- evaluate event-triggered visual-assisted relocalization;
- compare MPPI with DWB only as a reference benchmark;
- evaluate AMCL only if RTAB-Map localization fails defined acceptance criteria;
- add controlled restart/recovery with restart limits and localization revalidation.

### Decision rule

Keep the camera in the default product profile only if it provides a documented robustness benefit that justifies compute, calibration and maintenance cost. Phase 12 adds advanced recovery; it does not postpone the minimum Phase 5 validity gate.

RTAB-Map Localization remains primary. AMCL is a contingency, not a parallel default localization system.

## 37. Phase 13 — Optional near-field sensors and future chassis adapters

### YDLIDAR

Evaluate unchanged current hardware first, then optionally redesign only as a simple stop/slow source.

### Ultrasonic

Integrate only when real hardware, mounting and protocol are known.

### Future charging-robot chassis

Implement a new chassis-specific adapter below the `/vehicle_cmd_safe` generic command boundary. Do not modify mission, localization, semantic perception, Nav2 or Collision Monitor unless the new kinematics require explicit configuration.

## 38. Phase 14 — Site-specific and fleet/product validation

### Site work

- actual parking/charging map;
- site keepout and speed masks;
- charging locations and approach corridors;
- underground and outdoor tests;
- lighting, ramps, speed bumps, moving vehicles and pedestrians;
- long-distance missions and relocalization.

### Fleet/product work

- robot-model profile version;
- site-profile version;
- release artifact and rollback;
- diagnostics and telemetry;
- log metadata;
- staged deployment/canary robot;
- regression test before wider rollout;
- company-platform adapter after formal protocol delivery;
- final mapping of generic `BLOCKED`, `FAILED`, `HELP_REQUIRED` and cancellation states to company messages.

### Pass criteria

- one versioned configuration can be reproduced;
- per-robot calibration is separated from common software;
- site restrictions are external data, not code patches;
- failures are observable and rollback is possible;
- deployment can scale without manual source edits per robot.


# Part VI - Validation, diagnostics, and rollback

## 39. Required runtime diagnostics

Recommended diagnostic outputs:

```text
/system/localization_valid
/system/localization_reason
/system/pose_age
/system/tf_age
/system/pose_jump_detected
/system/tf_valid
/system/semantic_fresh
/system/costmap_fresh
/system/nav2_active
/system/collision_monitor_active
/system/vehicle_cmd_gate_state
/system/wheelchair_adapter_state
/vehicle_state
/system/command_authority
/system/can_driver_active
/system/clock_sync_state
/system/calibration_version
/mission/status
/mission/state
/mission/block_reason
/system/heartbeat_summary
/system/frozen_nodes
/system/end_to_end_latency
```

A simple system-status node may aggregate these later, but individual nodes should first expose clear state and timestamps.

## 40. Command authority enforcement

Before real arming, automatically verify:

- exactly one publisher on `/vehicle_cmd_safe`;
- exactly one publisher on `/wheelchair_control_command` for the current chassis;
- exactly one CAN-writing process;
- no legacy Pure Pursuit publisher;
- no YDLIDAR legacy safety-filter publisher;
- Generic Command Safety Gate state is `ARMED`;
- Wheelchair Command Adapter is in the intended real-output mode;
- `/system/localization_valid` is true and stable;
- Collision Monitor safe topic is active and fresh;
- raw Twist has no physical subscriber;
- joystick override is available;
- authority uniqueness remains true continuously while armed.

If any condition fails, real arming must be refused and `/vehicle_cmd_safe` must remain zero.

While armed, command-publisher and CAN-writer authority must be monitored continuously. If another publisher or writer appears, transition immediately to `FAULT`, publish repeated zero commands, and require explicit re-arming.

## 41. Failure policy

| Failure | Required behavior |
|---|---|
| Missing MID-360 data | stop/mission pause |
| Stale semantic output | stop or conservative mode |
| Missing TF | stop and fault |
| Basic localization invalid or unstable | refuse/disarm real output; zero `/vehicle_cmd_safe`; pause mission |
| Nav2 inactive | stop and fault |
| Collision Monitor inactive | refuse real arming |
| Mission waypoint rejected | stop; do not skip automatically |
| MPPI command stale | continuous stop |
| Generic Command Safety Gate fault | continuous zero `/vehicle_cmd_safe`; refuse arming |
| Chassis adapter fault | zero chassis command; refuse real output |
| Multiple `/vehicle_cmd_safe`, chassis-command, or CAN writers | refuse arming |
| CAN driver failure | stop through available layer; require operator |
| Joystick override | manual authority wins |
| Important node frozen | pause mission; zero command; report fault; controlled restart only under later policy |
| Progress timeout / no safe route | stop; classify `TEMPORARILY_BLOCKED` or `BLOCKED` |
| Recovery limit exceeded | stop; report `BLOCKED`; optionally escalate `HELP_REQUIRED` |
| Route identity ambiguous | reject mission; request clarified route |
| Latency budget exceeded | stop or refuse arming according to severity; report measured stage |

## 42. Rollback strategy

At every phase:

1. Commit the passing state.
2. Tag important milestones.
3. Keep the legacy baseline launch unchanged.
4. Store test reports and exact parameter files.
5. Never mix baseline and new authority in one launch.
6. Roll back to the last passing commit rather than patching live hardware blindly.
7. Keep a verified full Jetson system image for OS/driver/environment recovery.
8. Test the restore procedure on suitable spare media or a non-destructive validation path.

Suggested tags:

```text
baseline_transferred
humble_build_pass
semantic_isolated_pass
semantic_costmap_pass
mission_manager_pass
mppi_collision_monitor_pass
mock_bridge_pass
live_no_motion_pass
legacy_physical_baseline_pass
nav2_clear_path_physical_pass
nav2_static_obstacle_pass
```

---

# Part VII - Resource strategy

## 43. Computational load priorities

### Essential

1. Livox driver.
2. FAST-LIO.
3. RTAB-Map localization.
4. Semantic obstacle extraction.
5. Nav2 costmaps.
6. Nav2 planner and MPPI.
7. Collision Monitor.
8. Generic Command Safety Gate.
9. Current-chassis command adapter.

### Optional or reducible

- camera streams and neural visual matcher;
- YDLIDAR branch;
- dense debug clouds;
- high-rate semantic grid publication;
- high-resolution global costmap;
- unnecessary RViz displays;
- duplicate raw and semantic obstacle sources.

## 44. Initial optimization order

If compute is too high:

1. Disable camera.
2. Reduce RViz/debug topics.
3. Process every Nth cloud in semantic node.
4. Downsample point clouds.
5. Limit semantic range.
6. Reduce semantic grid publication rate.
7. Tune costmap dimensions and update rates.
8. Tune MPPI batch/sample settings while preserving controller rate.
9. Use YDLIDAR only if it adds measured value.
10. Consider C++ or accelerated semantic processing only after profiling.

Do not guess the bottleneck before collecting measurements.

---

# Part VIII - Final acceptance criteria

## 45. Software acceptance

- clean Humble build;
- reproducible launch modes;
- correct TF chain;
- semantic outputs validated;
- semantic costmaps validated;
- plan_nav route becomes ordered Nav2 goals;
- NavFn A* and MPPI complete fake missions;
- Collision Monitor filters final Twist;
- Generic Command Safety Gate passes mock tests;
- Wheelchair Command Adapter passes mock conversion tests;
- `/vehicle_cmd_safe` is the stable chassis-independent boundary;
- minimum localization-valid gate is proven before physical arming;
- temporary and final Collision Monitor profiles are separate;
- only one command authority;
- deferred modules have explicit status, activation gate and separate configuration;
- constraint filters are absent from the core baseline until their phase;
- robot-model and site-profile selection are external configuration;
- unified RViz profiles exist and display correct frame-aligned data;
- basic Mission Manager states and blocked outcomes are validated;
- typed `RouteMission` and `MissionState` interfaces are validated;
- `RouteMission` array lengths, frame, direction values and topology version are validated;
- route identity ambiguity is rejected;
- `/vehicle_cmd_safe` REP-103/SI-unit contract is tested;
- authority is monitored continuously;
- safety gate and chassis adapter publish repeated zero heartbeats;
- full wheelchair message contract has unit tests;
- Humble Collision Monitor source schema is verified;
- module registry, calibration manifest and clock policy exist.

## 46. Live sensor acceptance

- MID-360 and IMU rates stable;
- FAST-LIO output stable;
- RTAB-Map localization stable;
- binary localization-valid gate detects stale TF, stale odometry and pose jumps;
- temporary raw-MID-360 Collision Monitor profile passes no-motion validation;
- semantic latency acceptable;
- obstacle costmaps accurate;
- resource use acceptable;
- no command to CAN;
- important topic/node freshness is supervised;
- latency is measured stage-by-stage;
- unified live RViz profile validates sensor and navigation data;
- clock skew and timestamp behavior are within the selected profile limits.

## 47. Physical acceptance

- legacy baseline reproduced and archived in Phase 5B;
- joystick override confirmed;
- new mode start/stop/turn correct;
- Generic Command Safety Gate and Wheelchair Command Adapter remain separate authorities;
- invalid localization prevents or removes real command output;
- safe clear-path goal success;
- static obstacle avoidance success;
- Collision Monitor stop success;
- mission cancel and failure safe;
- no competing command publisher;
- repeatable runs documented;
- clear-path interface validation is distinguished from final semantic autonomy;
- keepout/speed constraints pass independently;
- camera contribution is measured rather than assumed;
- persistent blockage causes a safe `BLOCKED` outcome;
- unrecoverable repeated failure can escalate generic `HELP_REQUIRED`;
- minimal Gazebo tests and bag-replay tests precede difficult physical obstacle tests.

## 47A. Product/fleet acceptance

- no per-robot source edits for normal deployment;
- robot model, calibration and site policy are versioned profiles;
- common autonomy core is chassis-independent;
- new chassis integration is isolated to a hardware adapter and model configuration;
- diagnostics identify robot, software, configuration and site versions;
- release and rollback procedures are reproducible;
- staged deployment is possible before fleet-wide rollout;
- generic `/vehicle_state` is supported by future chassis adapters;
- optional vehicle-state fields distinguish `UNKNOWN` from confirmed `FALSE`;
- lower-controller acknowledgement and fault semantics are formally defined before product release;
- robot identity, authentication, release verification, SBOM/license review and security logging are addressed before fleet release.

---

# Part IX - Final selected pipelines by section

## 48. Mapping mode

```text
MID-360 + IMU
-> Livox driver
-> FAST-LIO
-> RTAB-Map mapping mode
-> RTAB-Map database
-> exported map assets
```

No Nav2 physical output and no CAN.

## 49. Localization mode

```text
MID-360 + IMU
-> FAST-LIO odom -> body
-> RTAB-Map saved-database localization
-> map -> odom
-> complete TF tree
```

## 50. Semantic perception mode

```text
/cloud_registered in odom
-> semantic_grid_tools
-> /ground_cloud
-> /obstacle_cloud
-> /semantic_grid
```

## 51. Navigation planning mode

```text
plan_nav sparse route
-> mission manager
-> sequential NavigateToPose
-> NavFn A*
-> Nav2 global path
```

## 52. Local control mode

```text
Nav2 global path
+ local semantic costmap
-> MPPI
-> /cmd_vel_nav_raw
```

## 53. Final software safety and generic command mode

```text
/cmd_vel_nav_raw
+ active Collision Monitor observation source
-> Collision Monitor
-> /cmd_vel_nav_safe
-> Generic Command Safety Gate
-> /vehicle_cmd_safe  TwistStamped
```

Phase 6 uses the temporary verified raw MID-360 source. Phase 8 onward uses semantic `/obstacle_cloud`.

## 54. Current-wheelchair command mode

```text
/vehicle_cmd_safe
-> Wheelchair Command Adapter
-> /wheelchair_control_command
-> existing wheelchair_controller_node
-> SocketCAN
-> lower controller
```

Wheelchair-specific radius, millimetre-unit, reverse, in-place-turn and command-layout logic belongs only in the Wheelchair Command Adapter or the existing wheelchair controller.

## 55. Legacy fallback mode

```text
plan_nav
-> optional YDLIDAR custom avoidance
-> custom Pure Pursuit
-> existing wheelchair controller
-> CAN
```

This mode remains physically separate from the new Nav2 mode.

## 55A. Navigation Constraint mode

```text
base map
+ site constraint mask
-> Keepout / Speed / Virtual-Wall filters
-> Nav2 global/local costmaps
-> planner and MPPI
```

Disabled until the dedicated constraint phase.

## 55B. Stereo-assisted localization mode

```text
MID-360 + IMU
-> FAST-LIO
+ stereo camera
-> RTAB-Map mapping / loop closure / relocalization
```

Optional and enabled only after LiDAR-only baseline comparison.

## 55C. Future charging-robot chassis mode

```text
/vehicle_cmd_safe  TwistStamped
-> charging-robot-specific command adapter
-> future lower controller
```

The autonomy core remains unchanged.

---

## 55D. Unified visualization mode

```text
TF + robot model + map + pose + clouds + semantic outputs
+ costmaps + plans + zones + health
-> selected RViz profile
```

## 55E. Mission/platform state mode

```text
platform command later
-> Platform Adapter
-> Mission Manager state machine
-> Nav2
-> mission outcome
-> Platform Adapter later
```

## 55F. Health-supervision mode

```text
heartbeats + freshness + lifecycle + latency
-> Health Supervisor
-> pause / zero command / report
-> controlled restart later
```

## 55G. Gazebo validation mode

```text
minimal simulated robot and sensors
-> same Mission Manager/Nav2/MPPI/Collision Monitor interfaces
-> /vehicle_cmd_safe
-> simulated motion
```

# 56. Immediate next actions

1. Preserve V2 and V3 unchanged as historical decision records.
2. Save this V3.1 FINAL file as the Phase-0 architecture/strategy authority.
3. Complete the Phase 0 requirement matrix, module registry, ownership matrix and open-interface register.
4. Create and checksum a recoverable image of the known-working Jetson.
5. Create the first calibration manifest and clock/timestamp policy.
6. Transfer and build the native Humble workspace without hardware commands.
7. Create the initial unified RViz profile.
8. Establish core simple Nav2 fake/mock navigation with all deferred modules disabled.
9. Implement `parking_robot_interfaces`, the typed Mission Manager state machine and the plan_nav route contract.
10. Establish MPPI -> Collision Monitor -> Generic Command Safety Gate -> mock Wheelchair Command Adapter.
11. Add continuous authority monitoring, repeated-zero heartbeats, adapter deadman tests and the generic `/vehicle_state` interface.
12. Start the minimal Gazebo validation track after fake closed-loop success.
13. Run live MID-360/FAST-LIO/RTAB-Map with the localization-valid gate, freshness supervision and temporary raw-MID-360 Collision Monitor profile; no physical output.
14. Reproduce and archive the known working legacy physical baseline in Phase 5B.
15. Perform one limited clear-path Nav2-to-current-CAN interface validation through `/vehicle_cmd_safe`.
16. Port and validate `semantic_grid_tools`.
17. Complete offline and live-no-motion semantic obstacle avoidance, including blocked classification.
18. Complete physical semantic obstacle avoidance.
19. Add Navigation Constraint Layer and versioned map-editing workflow only after the obstacle-avoidance gate.
20. Evaluate event-triggered stereo-assisted relocalization, DWB reference performance and AMCL contingency only in the later robustness phase.
21. Integrate company platform messages, YDLIDAR, ultrasonic, charging functions and future chassis only through their defined interfaces.
22. Address authentication, secure release, SBOM/license and security logging before fleet deployment.

---

## Final architecture decision

The selected V2 motion architecture remains unchanged:

```text
mission sources provide intent,
the Mission Manager uses standard Nav2 actions,
FAST-LIO provides local odometry,
RTAB-Map provides map memory, loop closure and relocalization,
semantic_grid_tools provides MID-360 ground/obstacle interpretation,
NavFn A* provides global metric planning,
MPPI provides predictive local control,
Collision Monitor provides final obstacle-based stop/slow filtering,
the Generic Command Safety Gate applies reusable arming, freshness and fault rules,
/vehicle_cmd_safe is the stable chassis-independent command boundary,
the Wheelchair Command Adapter converts that command for the current chassis,
and the existing wheelchair_controller_node remains the current SocketCAN driver.
```

Revision 3 adds three cross-cutting planes, retained in Revision 3.1:

```text
Mission/state plane:
generic task, cancel, blocked, failed, succeeded and help-required states

Observability plane:
standard RViz profiles for development, validation and presentation

Supervision plane:
heartbeats, freshness, freeze detection, latency, stop and later controlled restart
```

Revision 3.1 additionally formalizes typed mission and vehicle-state interfaces, mandatory versus configurable localization checks, the `/vehicle_cmd_safe` contract, clock/calibration artifacts, and a Phase 5B legacy-baseline reproduction gate.

Normal navigation remains LiDAR-dominant. The stereo camera is deferred and later evaluated primarily for mapping, loop closure and event-triggered relocalization. It must not slow or freeze normal navigation.

The final company product will receive tasks from a platform adapter, but the company protocol is not invented before formal specifications are supplied.

Local obstacle handling must distinguish passable, temporarily blocked and unpassable situations. Unrecoverable conditions stop safely and report `BLOCKED`, with generic `HELP_REQUIRED` escalation available for later company integration.

The plan_nav collaborator retains ownership of the complete GUI/topological ambiguity solution. Our Mission Manager requires an unambiguous route identity and rejects ambiguous geometry-only selections.

Gazebo is a supporting validation track that reuses the same ROS interfaces. It does not replace bag replay, live-no-motion tests or physical validation.

The current wheelchair remains the development chassis and proven hardware baseline. The reusable software above `/vehicle_cmd_safe` is the product architecture.

Navigation constraints remain mandatory but separately configured and initially disabled. Full free-space 3D navigation remains outside the current project scope.

This V3.1 FINAL file is the architecture and implementation authority for beginning Phase 0. Future changes should be driven by implementation evidence or formal company task, charging, chassis, security or site specifications—not by another speculative redesign.
