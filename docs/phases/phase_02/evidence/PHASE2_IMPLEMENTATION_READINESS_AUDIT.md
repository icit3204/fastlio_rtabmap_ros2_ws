# Phase 2 Implementation-Readiness Audit

Date: 2026-08-01
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Executive conclusion

**`PHASE2_IMPLEMENTATION_READINESS_AUDIT_PASS`**

Phase 2 can be implemented using existing components with minimal new code.
The primary gap is a dedicated Phase 2 test runner. All Nav2 dependencies
are installed at 1.1.20. A suitable map exists in the repository. The
fake-odometry, Nav2 configuration, and mock-command designs are exact and
safe. No hardware, CAN, or physical command path can be reached from the
proposed design.

---

## 1. Exact Phase 2 authority contract

### PHASE2_REQUIRED

| # | Requirement | Authority source |
|---|---|---|
| R1 | Static map or fake localization | §26 "map / fake localization" |
| R2 | Initial pose setup | §26 |
| R3 | Simple Nav2 goal via NavigateToPose | §26 |
| R4 | NavFn A* global planner | §26 |
| R5 | MPPI local controller | §26 |
| R6 | Mock Twist output (no hardware) | §26 |
| R7 | Fake odometry | §26 |
| R8 | Offline/fake closed loop only | §26 |

### PHASE2_REQUIRED_TESTS

| # | Test | Authority source |
|---|---|---|
| T1 | Start pose | §26 Tests |
| T2 | One simple goal | §26 |
| T3 | Sequential simple goals | §26 |
| T4 | Goal cancel | §26 |
| T5 | Planner failure | §26 |
| T6 | Controller timeout | §26 |
| T7 | TF failure | §26 |
| T8 | Fake closed-loop goal completion | §26 |
| T9 | Visualization of goals, path, pose, command, health | §26 |
| T10 | Basic test-runner mission states and cancellation | §26 |

### PHASE2_PASS_CRITERIA

| # | Criterion |
|---|---|
| P1 | Nav2 reaches simple goals reproducibly |
| P2 | Global path and MPPI commands are sensible |
| P3 | Failures stop or abort cleanly |
| P4 | No real command topic exists |

### PHASE2_EXPLICITLY_EXCLUDED

All items from §26 scope and task description exclusions are confirmed
out of scope: semantic_grid_tools, Collision Monitor, Generic Command
Safety Gate, /vehicle_cmd_safe, Wheelchair Command Adapter,
wheelchair_controller_node, CAN, Pure Pursuit, laser_command_safety_filter,
plan_nav_laser_avoidance, MID-360/Livox, FAST-LIO live, RTAB-Map live
localization, stereo camera, Torch/PyMatcher, YDLIDAR, ultrasonic,
keepout/speed filters, Navigation Constraint Layer, Gazebo, physical
hardware, Phase 3 typed Mission Manager interfaces.

---

## 2. Component inventory

### REUSE_UNCHANGED (built and installed in Phase 1)

| Component | Path | Status |
|---|---|---|
| `odom_from_cmd_vel.py` | `src/robot_bringup/scripts/odom_from_cmd_vel.py` | Built, installed, tested in Phase 1 |
| `path_publisher.py` | `src/robot_bringup/scripts/path_publisher.py` | Built, installed, path-parameterized in Phase 1 |
| `path_waypoint_sender.py` | `src/robot_bringup/scripts/path_waypoint_sender.py` | Built, installed, path-parameterized in Phase 1 |
| `nav2_common.yaml` | `src/robot_bringup/config/nav2_common.yaml` | NavFn A*, MPPI configured |
| `phase1_authority_baseline.rviz` | `src/robot_bringup/config/phase1_authority_baseline.rviz` | Unified RViz profile |
| `offline_view.launch.py` | `src/robot_bringup/launch/offline_view.launch.py` | Map loading + RViz concept |

### REUSE_WITH_MODIFICATION

| Component | Modification needed |
|---|---|
| `offline_avoidance.launch.py` | Remove obstacle/clicked-point/Collision Monitor references for Phase 2 baseline; adapt map config |
| `fake_odom_publisher.py` | Repurpose for Phase 2 (currently RTAB-Map-specific) |

### NOT_REUSABLE (legacy only)

| Component | Reason |
|---|---|
| `fake_odom_sim.py` | Pre-programmed circular motion, not cmd_vel-driven |
| `replay_path.py` | hard-coded legacy path, superseded by path_publisher |
| `plan_nav_laser_avoidance.py` | PHASE2_EXCLUDED |
| `laser_command_safety_filter.py` | PHASE2_EXCLUDED |

### NEW_COMPONENTS_NEEDED

| Component | Purpose |
|---|---|
| `phase2_goal_test_runner.py` | NavigateToPose action client: send goals, cancel, log results |
| `phase2_core_nav2.launch.py` | Minimal Phase 2 bringup: map_server + fake_odom + Nav2 |
| `phase2_fake_base.py` | Wrapper combining odom_from_cmd_vel with initial-pose support |
| `phase2_navigation.rviz` | Phase 2-specific displays (or extend baseline profile) |

---

## 3. Recovered workspace assessment

| Workspace | Location | Classification |
|---|---|---|
| `rtabmap_nav2_stack_before_restore` | `/home/dog/Backup/` | EVIDENCE_ONLY — backup, not source |
| `after competition` | `/home/dog/AZ/` | EVIDENCE_ONLY — historical results |

No recovered workspace contains directly reusable Phase 2 source. The
existing `offline_avoidance.launch.py` and `odom_from_cmd_vel.py` in the
active repository already provide the needed concepts.

---

## 4. Recommended map

**Selected**: `scripts/offline_nav_maps/clean_map.yaml`

| Attribute | Value |
|---|---|
| Image | `clean_map.pgm` |
| Resolution | 0.05 m/pixel |
| Size | 1744 × 2683 pixels (87.2m × 134.15m) |
| Origin | [-39.1, -85.15, 0.0] |
| Free threshold | 0.196 |
| Occupied threshold | 0.65 |
| Format | Portable (PGM), relative path |

**Recommended poses** (provisional, to be confirmed by map inspection):
- Start: near origin or known open area
- Easy goal: 5-10m from start in free space
- Second goal: continuing beyond first
- Unreachable goal: inside occupied region or beyond map boundary

---

## 5. Fake localization and TF design

**Design**: `odom_from_cmd_vel.py` as the primary mechanism.

```
Topic flow:
  MPPI → /cmd_vel_phase2_mock [Twist]
       → odom_from_cmd_vel (subscribes)
         → /Odometry [Odometry]
         → odom → base_footprint [TF]

  static_transform_publisher (in launch)
    → map → odom [TF] (identity)
```

| Frame | Publisher | Type |
|---|---|---|
| map → odom | `static_transform_publisher` (launch) | Static identity |
| odom → base_footprint | `odom_from_cmd_vel.py` | Dynamic (cmd_vel integration) |

**Parameters**:
- `use_sim_time`: `false` (deterministic wall-clock fake integration)
- Update rate: 50 Hz (0.02s timer)
- Cmd_vel timeout: 0.5s (stale → zero velocity)
- Integration: 2D diff-drive (Euler step)

**Initial pose**: `odom_from_cmd_vel.py` accepts `/initialpose` via
`PoseWithCovarianceStamped` subscriber. Test runner publishes start pose
before sending first goal.

---

## 6. Mock command design

**Topic**: `/cmd_vel_phase2_mock`

Chosen over reusing `/cmd_vel_nav` to prevent any accidental connection
to legacy subscribers.

| Attribute | Value |
|---|---|
| Message type | `geometry_msgs/msg/Twist` |
| Publisher | Nav2 MPPI (remapped from `/cmd_vel`) |
| Subscriber | `odom_from_cmd_vel.py` (modified to subscribe this topic) |
| No subscriber from | wheelchair_controller, Pure Pursuit, CAN writer |
| Safety | Not remapped to any existing command topic |

**odom_from_cmd_vel.py modification**: Change subscription from
`/cmd_vel_nav` to `/cmd_vel_phase2_mock` (or make it a parameter).

**No velocity smoother** in minimal Phase 2 baseline. Can be added
later if MPPI output is noisy.

---

## 7. Nav2 configuration assessment

### Reused from nav2_common.yaml

| Section | Status |
|---|---|
| `bt_navigator` — BT plugin list | REUSED_UNCHANGED (standard Humble plugins) |
| `controller_server` — MPPI, DiffDrive | REUSED_UNCHANGED (vx_max=0.5, wz_max=1.5) |
| `controller_server` — goal checker (xy=1.0m, yaw=6.28) | REUSED_UNCHANGED |
| `controller_server` — progress checker (0.15m/15s) | REUSED_UNCHANGED |

### Phase 2 overlay changes

| Parameter | Phase 2 value | Reason |
|---|---|---|
| `cmd_vel_out_topic` | `/cmd_vel_phase2_mock` | Prevent topic collision |
| `global_costmap.robot_base_frame` | `base_footprint` | Match fake TF |
| `local_costmap.robot_base_frame` | `base_footprint` | Match fake TF |
| `global_costmap.global_frame` | `map` | Standard |
| `local_costmap.global_frame` | `odom` | Standard |
| `global_costmap.static_layer.map_topic` | `/map` | map_server |
| All observation sources | **REMOVED** | No sensors in Phase 2 |
| `global_costmap.obstacle_layer` | **DISABLED** | No sensors |
| `local_costmap.obstacle_layer` | **DISABLED** | No sensors |
| `global_costmap.inflation_layer` | **DISABLED** | Not needed with empty costmap |

### Costmap design

Both global and local costmaps use only `static_layer` from map_server.
Mppi will plan paths avoiding occupied cells in the static map. No
inflation needed for empty-costmap operation; can be added later if
occupancy data exists.

### Lifecycle nodes required

`map_server`, `planner_server`, `controller_server`, `bt_navigator`,
`behavior_server` (for spin/wait/backup), `lifecycle_manager`.

All installed at 1.1.20.

---

## 8. Behavior Tree assessment

The standard Humble `navigate_to_pose_w_replanning_and_recovery.xml`
(with recovery subtree) is sufficient for Phase 2.

Key BT plugins verified installed:
- `nav2_navigate_to_pose_action_bt_node`
- `nav2_compute_path_to_pose_action_bt_node`
- `nav2_follow_path_action_bt_node`
- `nav2_goal_reached_condition_bt_node`
- `nav2_planner_selector_bt_node`
- `nav2_controller_selector_bt_node`
- `nav2_goal_checker_selector_bt_node`
- `nav2_recovery_node_bt_node`
- `nav2_rate_controller_bt_node`

All are standard Humble plugins — no custom BT XML needed for Phase 2.

---

## 9. Test runner design

### phase2_goal_test_runner.py

A Python node using `NavigateToPose` action client.

**States**:
```
IDLE → WAITING_FOR_NAV2 → SETTING_START → GOAL_ACTIVE → SUCCEEDED
                                                    → CANCELLING → CANCELLED
                                                    → ABORTED
                                                    → FAILED
```

**Capabilities**:
- Wait for `navigate_to_pose` action server
- Publish `/initialpose` at configured start
- Send one `NavigateToPose` goal
- Send sequential goals (goal list)
- Cancel active goal
- Log feedback, result, and elapsed time
- Exit with deterministic status

**NOT implemented**: RouteMission, MissionState, plan_nav integration,
vehicle commands, hardware access.

---

## 10. Complete test matrix

| # | Test | Method | Pass criterion |
|---|---|---|---|
| T1 | Start pose | Publish `/initialpose`; verify odom_from_cmd_vel resets | Position within 0.1m of commanded |
| T2 | One simple goal | Send NavigateToPose to point 5-10m ahead | SUCCEEDED, position within xy_tolerance |
| T3 | Sequential goals | Send 3 goals in series | All 3 SUCCEEDED, in order, no skip |
| T4 | Goal cancel | Cancel midway through goal execution | CANCELLED result; cmd_vel goes to zero within 0.5s |
| T5 | Planner failure | Goal in occupied region or beyond map | ABORTED; no continued cmd_vel |
| T6 | Controller timeout | Obstruct goal that MPPI can't solve | ABORTED or timeout; cmd_vel zero |
| T7 | TF failure | Send goal with nav2 off; no odom | Action rejected or timeout; clean stop |
| T8 | Fake closed-loop | Run T2 5 times consecutively | All 5 SUCCEEDED; consistent timing |
| T9 | Visualization | RViz with baseline profile | Goal, path, pose, cmd_vel displayed |
| T10 | Test states | Record all state transitions | IDLE→WAITING→...→SUCCEEDED logged |

---

## 11. Package and file structure recommendation

**RECOMMENDED**: OPTION B — create `parking_robot_bringup` package.

Rationale:
- Avoids contaminating `robot_bringup` legacy launches
- Keeps hardware impossible by default
- Supports later Phase 3/4 growth (mission manager, safety gate)
- Authority-recommended package name

**Proposed files** (not created in this audit):

```
parking_robot_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── phase2_core_nav2.launch.py
│   └── phase2_test_launch.py
├── config/
│   ├── phase2_nav2_params.yaml
│   └── phase2_navigation.rviz
├── scripts/
│   ├── phase2_goal_test_runner.py
│   └── phase2_fake_base.py
└── maps/
    └── (symlink or copy clean_map.pgm + clean_map.yaml)
```

---

## 12. Dependency assessment

All required packages are already installed (Nav2 1.1.20):

```
ros-humble-nav2-bringup
ros-humble-nav2-map-server
ros-humble-nav2-navigation2
ros-humble-nav2-bt-navigator
ros-humble-nav2-planner (NavFn)
ros-humble-nav2-mppi-controller
ros-humble-nav2-controller
ros-humble-nav2-lifecycle-manager
```

**No new apt packages required.**

Colcon dependency: `parking_robot_bringup` depends on `nav2_bringup`,
`nav_msgs`, `geometry_msgs`, `tf2_ros`, `rclpy`, `nav2_msgs`,
`action_msgs`.

---

## 13. Risk audit

| Risk | Likelihood | Impact | Prevention |
|---|---|---|---|
| Fake odom angular sign mismatch | LOW | Bad paths | Verify with known 90° turn test |
| Nav2 waiting for absent sensor topics | LOW | Log spam | Remove all observation sources from params |
| Frame mismatch (base_footprint vs body) | LOW | TF errors | Use base_footprint consistently |
| MPPI computational load on Jetson | LOW | Slow planning | MPPI params already tuned (60 time_steps, 200 batch) |
| Empty footprint | LOW | MPPI crash | Set robot_radius or footprint in params |
| Cancellation leaving stale velocity | LOW | Ghost motion | 0.5s cmd_vel timeout in odom_from_cmd_vel |
| Startup race (action server not ready) | MEDIUM | Test flakiness | Test runner waits for action server |
| GUI/display unavailable (no X on Jetson) | MEDIUM | Can't verify visually | RViz recorded as NOT_RUN_NO_DISPLAY; use log-based verification |

---

## 14. Minimum implementation stages

### P2-A: Create isolated package and configuration
- Create `parking_robot_bringup` with CMakeLists.txt, package.xml
- Create `phase2_nav2_params.yaml` (nav2_common.yaml copy with Phase 2 modifications)
- Create `phase2_core_nav2.launch.py`
- Create `phase2_fake_base.py` (odom_from_cmd_vel.py wrapper)
- Build and install

### P2-B: Static launch inspection
- `ros2 launch parking_robot_bringup phase2_core_nav2.launch.py` parse check
- Verify no hardware/node auto-starts
- Verify lifecycle manager autostart

### P2-C: Bring up map + fake TF/odom + Nav2 no goal
- Launch map_server + fake_base + Nav2
- Verify `/map`, `/Odometry`, TF tree
- Verify Nav2 lifecycle nodes active
- No goal sent yet

### P2-D: One simple fake closed-loop goal
- Create `phase2_goal_test_runner.py`
- Run T1 (start pose) + T2 (simple goal)
- Verify SUCCEEDED, sensible path and velocity

### P2-E: Sequential goals and cancellation
- Run T3 (sequential 3 goals) + T4 (cancel)
- Verify all succeed, cancel returns CANCELLED

### P2-F: Failure tests
- Run T5 (planner failure), T6 (controller timeout), T7 (TF failure)
- Verify clean abort, zero velocity

### P2-G: Repetition, metrics, documentation
- Run T8 (5x closed-loop repetition)
- Record metrics
- Document and freeze

---

## 15. Integrity

| Check | Status |
|---|---|
| Git clean | ✅ |
| HEAD `2dc53a1` = origin/main | ✅ |
| All 3 tags unchanged | ✅ |
| No repository content modified | ✅ |
| No build, ROS, hardware, CAN, UDP | ✅ |
| No commit, tag, push | ✅ |

---

PHASE2_IMPLEMENTATION_READINESS_AUDIT_PASS
