# Phase 1 Static Command-Publisher and Physical-Authority Audit

Date: 2026-07-31
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Executive conclusion

**`PHASE1_STATIC_COMMAND_AUTHORITY_AUDIT_PASS`**

Every authoritative command publisher, CAN writer, and physical-authority
path has been identified and classified. The audit found ONE direct CAN
writer (`wheelchair_controller_node`), ONE final physical command chain
(legacy Pure Pursuit), and ONE Nav2 branch (Collision Monitor → `/cmd_vel`
→ pure_pursuit `/cmd_vel` subscriber). No `/vehicle_cmd_safe`,
`/cmd_vel_nav_raw`, or `/cmd_vel_nav_safe` publishers exist — the V3.1
selected architecture is NOT_IMPLEMENTED.

## Authoritative source determination

### Tracked authoritative source (under `src/`)

| Package | Path |
|---|---|
| `wheelchair_controller` | `src/wheelchair_controller/` |
| `robot_bringup` | `src/robot_bringup/` |
| `fast_lio` | `src/FAST_LIO_ROS2/` |
| `livox_ros_driver2` | `src/livox_ros_driver2/` |
| `rtabmap_ros` (all sub-packages) | `src/rtabmap_ros/` |
| `ydlidar_ros2_driver` | `src/ydlidar_ros2_driver/` |
| `ydlidar_sdk` | `src/YDLidar-SDK/` |

### Untracked authoritative directory

| Directory | Status |
|---|---|
| `plan_nav/` | Standalone GUI application (Python), NOT a ROS publisher of paths |
| `plan_nav07202900/` | Backup copy (1,192 files vs 597 active) — NOT authoritative |

### Backup copies excluded from runtime authority

- `plan_nav07202900/` — older plan_nav backup, not referenced by any launch/script/build
- `scripts/offline_nav_maps_corridor5_backup_*/` — map backups only

### Generated install copies

- `/home/dog/phase1_builds/clean_build_opencv454d_single_abi_20260731_045543/install/` — stable Phase 1 install, derived from tracked source

---

## Exact legacy command chain

### With laser avoidance (default: enable_laser_avoidance=true)

```
plan_nav (GUI export)
  → /plan_nav [nav_msgs/Path]           (published by plan_nav, external)     [PUBLISHER]
  → plan_nav_laser_avoidance.py          (subscribes /plan_nav)
    → /active_plan [nav_msgs/Path]       (published by plan_nav_laser_avoidance) [PUBLISHER]
    → pure_pursuit_controller_node       (subscribes /active_plan)
      → /wheelchair_control_command_raw [Float32MultiArray] (published)        [PUBLISHER]
      → laser_command_safety_filter.py   (subscribes /wheelchair_control_command_raw)
        → /wheelchair_control_command [Float32MultiArray] (published)          [PUBLISHER]
        → wheelchair_controller_node     (subscribes /wheelchair_control_command)
          → can0 (SocketCAN)             [CAN WRITER]
```

### Without laser avoidance (enable_laser_avoidance=false)

```
plan_nav (GUI export)
  → /plan_nav [nav_msgs/Path]           (published by plan_nav, external)
  → pure_pursuit_controller_node         (subscribes /plan_nav)
    → /wheelchair_control_command [Float32MultiArray] (published)              [PUBLISHER]
    → wheelchair_controller_node         (subscribes /wheelchair_control_command)
      → can0 (SocketCAN)                [CAN WRITER]
```

### Key observations

- Two pure_pursuit instances ARE launched simultaneously in bringup_2d, but with mutually exclusive conditions (`enable_laser_avoidance` vs `not enable_laser_avoidance`) — only ONE is active at a time.
- `wheelchair_controller_node` is the ONLY CAN writer. It subscribes to `/wheelchair_control_command`.
- `pure_pursuit_controller_node` publishes `Float32MultiArray` (radius + velocity pair), NOT `Twist`.
- `laser_command_safety_filter.py` is a relay/filter: subscribes to raw, publishes safe, with lidar stop logic.

---

## Exact Nav2 branch

### Nav2 controller configuration

From `nav2_common.yaml`:
```yaml
controller_plugins: ['FollowPath']
FollowPath:
  plugin: nav2_mppi_controller::MPPIController
```

Nav2 MPPI outputs to Nav2's default `/cmd_vel` topic. In `bringup.launch.py` and `bringup_2d.launch.py`, this is remapped to `/cmd_vel_nav`:

```python
SetRemap('/cmd_vel', '/cmd_vel_nav'),
```

### Nav2 → Collision Monitor chain

```
Nav2 MPPI
  → /cmd_vel_nav [Twist]                (Nav2 internal, remapped from /cmd_vel)
  → collision_monitor                   (subscribes /cmd_vel_nav)
    → /cmd_vel [Twist]                  (published by collision_monitor)       [PUBLISHER]
    → pure_pursuit_controller_node       (subscribes /cmd_vel, CmdVelCallback)
```

### Critical finding: Nav2 to wheelchair bridge

`pure_pursuit_controller_node` subscribes to `/cmd_vel` (line 51 of `pure_pursuit_controller.cpp`). This means:

- Nav2 `/cmd_vel` Twist IS consumed by pure_pursuit
- pure_pursuit converts Twist to its internal velocity state
- But pure_pursuit's PRIMARY output is through the path-following logic (`/plan_nav` or `/active_plan` → `Float32MultiArray`)
- Nav2 `/cmd_vel` does NOT flow directly through to `wheelchair_controller_node`

**The Nav2 branch currently feeds into pure_pursuit's velocity state but pure_pursuit still requires a path input to generate output.** Without a `/plan_nav` path, pure_pursuit won't publish wheelchair commands even with Nav2 `/cmd_vel` active.

### Nav2 branch status

| Component | Status |
|---|---|
| Nav2 MPPI controller | CONFIGURED, launched in 'navigation' mode |
| Collision Monitor | CONFIGURED, launched in 'navigation' mode |
| Collision Monitor input | `/cmd_vel_nav` [Twist] |
| Collision Monitor output | `/cmd_vel` [Twist] |
| Nav2-to-wheelchair bridge | INDIRECT — pure_pursuit subscribes `/cmd_vel` but needs path input |
| Nav2 directly reaches CAN | NO — no direct Nav2 Twist to wheelchair_controller |
| Nav2 + legacy chains coexist | YES — both launched in 'navigation' mode |

---

## Complete publisher/subscriber/remapping matrix

See: `PHASE1_COMMAND_AUTHORITY_MATRIX.tsv`

## CAN writer inventory

See: `PHASE1_CAN_AND_LOWER_CONTROLLER_WRITER_INVENTORY.md`

---

## Launch-mode authority matrix

### MODE: mapping (fast_lio2.launch.py default)

| Component | Status |
|---|---|
| Velocity controller | NONE |
| Final ROS command publisher | NONE |
| Chassis-command publisher | NONE |
| CAN writer | NOT LAUNCHED |
| Physical movement possible | NO |
| Authority conflict | NONE |

### MODE: localization (fast_lio2.launch.py)

| Component | Status |
|---|---|
| Velocity controller | NONE |
| Final ROS command publisher | NONE |
| Chassis-command publisher | NONE |
| CAN writer | NOT LAUNCHED |
| Physical movement possible | NO |
| Authority conflict | NONE |

### MODE: navigation (bringup.launch.py default)

| Component | Status |
|---|---|
| Velocity controller | Nav2 MPPI (controller_server) |
| Nav2 cmd_vel topic | `/cmd_vel_nav` (remapped from `/cmd_vel`) |
| Collision Monitor | ACTIVE (input: `/cmd_vel_nav`, output: `/cmd_vel`) |
| Pure Pursuit | NOT LAUNCHED (bringup.launch.py has no pure_pursuit) |
| Final ROS command publisher | NONE (Collision Monitor publishes `/cmd_vel` Twist but no subscriber to wheelchair) |
| CAN writer | NOT LAUNCHED |
| Physical movement possible | NO — no path to CAN |
| Authority conflict | NONE |

### MODE: navigation (bringup_2d.launch.py)

| Component | Status |
|---|---|
| Velocity controller | Nav2 MPPI + Pure Pursuit (dual authority) |
| Nav2 cmd_vel topic | `/cmd_vel_nav` |
| Collision Monitor | ACTIVE (input: `/cmd_vel_nav`, output: `/cmd_vel`) |
| Pure Pursuit | TWO INSTANCES (mutually exclusive via condition) |
| Pure Pursuit with avoidance | `/active_plan` → `/wheelchair_control_command_raw` |
| Pure Pursuit no avoidance | `/plan_nav` → `/wheelchair_control_command` |
| Laser safety filter | ACTIVE (with avoidance only) |
| CAN writer | NOT LAUNCHED (wheelchair_controller_node not in any launch) |
| Physical movement possible | NO — wheelchair_controller_node must be started manually |
| Authority conflict | DUAL (Nav2 MPPI + Pure Pursuit both compute velocity) but Pure Pursuit wins for physical output since only it can reach CAN |

---

## Conflicts and bypass findings

### A. SOURCE FACT

1. **S-F1**: `wheelchair_controller_node` is the SOLE physical CAN writer (SocketCAN to `can0`, CAN ID `0x801400`).
2. **S-F2**: `wheelchair_controller_node` is NOT launched by any workspace launch file — requires manual start.
3. **S-F3**: `pure_pursuit_controller_node` publishes `Float32MultiArray`, not `Twist` — incompatible with Nav2 Collision Monitor output type.
4. **S-F4**: Two pure_pursuit instances defined in bringup_2d.launch.py are mutually exclusive via `IfCondition`.
5. **S-F5**: Collision Monitor publishes filtered `/cmd_vel` [Twist] which is consumed by pure_pursuit's `/cmd_vel` subscriber — but this subscriber only stores velocity internally, it doesn't drive path-following output.

### B. CONFIGURATION RISK

1. **C-R1**: `bringup_2d.launch.py` launches both Nav2 MPPI and Pure Pursuit in 'navigation' mode — dual velocity authority.
2. **C-R2**: Collision Monitor configured with `/scan` observation source (PointCloud2 source not configured in current bringup profiles).
3. **C-R3**: Laser safety filter bypassed when `enable_laser_avoidance=false`.

### C. LATER IMPLEMENTATION REQUIREMENT

1. **C-L1**: `/vehicle_cmd_safe` [TwistStamped] — NOT_IMPLEMENTED, no publisher exists in source.
2. **C-L2**: `/cmd_vel_nav_raw` — NOT_IMPLEMENTED.
3. **C-L3**: `/cmd_vel_nav_safe` — NOT_IMPLEMENTED.
4. **C-L4**: Generic Command Safety Gate — NOT_IMPLEMENTED.
5. **C-L5**: Wheelchair Command Adapter (TwistStamped → Float32MultiArray) — NOT_IMPLEMENTED.

### D. NOT A PHASE 1 BLOCKER

1. **D-1**: Dual-ABI concern — resolved in Phase 1 native build (single OpenCV 4.5.4d ABI).
2. **D-2**: `odom_from_cmd_vel.py` publishes `/Odometry` and TF, not commands — diagnostic/simulation tool only.
3. **D-3**: `plan_nav` is a GUI tool publishing `/plan_nav` [Path] externally — not a command publisher.

### E. UNRESOLVED

None.

---

## Phase 1 closure classification

The static command-publisher audit PASSES because:

- All authoritative command publishers identified: 3 publishers (pure_pursuit x2, laser_safety_filter) + 1 CAN writer (wheelchair_controller_node) + 1 Nav2 chain (Collision Monitor).
- All final remappings resolved: `/cmd_vel → /cmd_vel_nav` for Nav2, `/wheelchair_control_command_raw → /wheelchair_control_command` for safety filter.
- CAN writer identified: `wheelchair_controller_node`, SocketCAN to `can0`, CAN ID `0x801400`.
- Current default physical command chain is exact and verified.
- Nav2 branch authority status is exact — Collision Monitor publishes `/cmd_vel` [Twist], pure_pursuit subscribes, but no direct Nav2 → CAN path exists.
- Publisher vs subscriber roles correctly distinguished.
- Backups separated from active authority.
- No material command source remains unresolved.

---

## Integrity confirmation

| Check | Status |
|---|---|
| Git clean | ✅ |
| HEAD `c27961d` unchanged | ✅ |
| Tags unchanged (`phase0` at `ae7517e`, `phase1` at `7556f15`) | ✅ |
| No tracked source modified | ✅ |
| No build run | ✅ |
| No ROS node launched | ✅ |
| No message published | ✅ |
| No CAN or UDP transport opened | ✅ |
| No package installed/removed | ✅ |
| No commit/tag/push | ✅ |

---

PHASE1_STATIC_COMMAND_AUTHORITY_AUDIT_PASS
