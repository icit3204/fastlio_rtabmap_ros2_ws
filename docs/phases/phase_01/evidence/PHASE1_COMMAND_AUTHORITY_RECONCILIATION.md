# Phase 1 Command Authority Reconciliation

Date: 2026-07-31
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
Branch: `main`
Source revision: `c27961dc0eb156f4811e0d55c6793df304fd7ce6`
Authority document: `/home/dog/Downloads/FINAL_REVISED_ARCHITECTURE_AND_IMPLEMENTATION_STRATEGY_AFTER_DAY0_V3_1_FINAL_PHASE0_AUTHORITY.md`
Authority SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`

## Scope

This is an independent read-only reconciliation of the current source against:

- `/home/dog/phase1_reports/PHASE1_STATIC_COMMAND_PUBLISHER_AND_AUTHORITY_AUDIT.md`
- `/home/dog/phase1_reports/PHASE1_COMMAND_AUTHORITY_MATRIX.tsv`
- `/home/dog/phase1_reports/PHASE1_CAN_AND_LOWER_CONTROLLER_WRITER_INVENTORY.md`
- verified V3.1 Phase-0 authority

No source, launch, configuration, or prior report was edited.

## 1. Resolved plan_nav Publisher Truth

`plan_nav` is a ROS publisher under current source, but `plan_nav/main.py` does not directly create the ROS node or publisher.

Exact chain:

```text
python3 plan_nav/main.py
-> QApplication
-> MainWindow()
-> user switches to operation mode
-> MainWindow._start_plan_publisher()
-> core.nav_publisher.PlanPublisher QThread
-> rclpy.create_node('plan_publisher')
-> create_publisher(nav_msgs.msg.Path, '/plan_nav', 10)
-> publish remaining route at 10 Hz when both route and current pose exist
```

Evidence:

- `plan_nav/main.py` creates `QApplication`, `MainWindow`, shows it, and enters Qt event loop (`main.py:11-30`).
- `MainWindow._on_mode_changed()` starts pose receiver and plan publisher only when `mode == 'op'` (`ui/main_window.py:1118-1133`).
- `_start_plan_publisher()` imports `PlanPublisher`, creates it, connects signals, and starts the thread (`ui/main_window.py:1164-1171`).
- Route selection updates the publisher path only in operation mode and only when `_plan_publisher` exists (`ui/main_window.py:880-883`).
- Pose updates feed current pose into the publisher (`ui/main_window.py:1186-1190`).
- `PlanPublisher.run()` imports `rclpy`, initializes it if needed, creates node `plan_publisher`, and creates publisher `/plan_nav` with `nav_msgs.msg.Path` (`core/nav_publisher.py:98-141`).
- The loop uses `interval = 0.1` and publishes when `_build_remaining_path()` returns a message (`core/nav_publisher.py:152-160`).
- `_build_remaining_path()` returns `None` unless both trajectory and pose exist, and suppresses paths with fewer than two remaining poses (`core/nav_publisher.py:182-190`, `204-207`).

Resolution:

- Incorrect: "`plan_nav` is NOT a ROS publisher."
- Correct: running `python3 main.py` can publish `/plan_nav`, after GUI operation mode starts `PlanPublisher` and after route plus pose are available.
- Correct with clarification: "`plan_nav/main.py` publishes `/plan_nav`" means the `main.py` application invokes the GUI/module chain that publishes; the publisher is created in `plan_nav/core/nav_publisher.py`, not directly in `main.py`.
- Publication is conditional, not always active from process start.

Separate lower-controller note: the same GUI path-selection flow also starts `UdpSender` (`ui/main_window.py:879`, `1267-1287`), which opens UDP and sends binary `(R, v)` frames (`core/udp_sender.py:44-93`). This is not `/plan_nav` ROS publishing and must be inventoried separately as a lower-controller UDP writer.

## 2. Resolved Collision Monitor Launch Truth

Different launch files have different states.

### `bringup.launch.py`

Collision Monitor construction exists and is active:

- Node object is created with package `nav2_collision_monitor`, executable `collision_monitor`, name `collision_monitor` (`bringup.launch.py:237-242`).
- It is conditioned on `mode == 'navigation'` (`bringup.launch.py:243`).
- Parameters set input `/cmd_vel_nav`, output `/cmd_vel`, observation source `pointcloud`, and topic `/cloud_registered_body` (`bringup.launch.py:248-275`).
- Lifecycle manager is also constructed and conditioned on `mode == 'navigation'` (`bringup.launch.py:279-289`).
- Both are added to the LaunchDescription (`bringup.launch.py:335-337`).
- Default `mode` is `navigation` (`bringup.launch.py:50`), so normal navigation invocation starts Collision Monitor.

### `bringup_2d.launch.py`

Collision Monitor construction exists but is inactive/unreachable:

- Node object is created with package `nav2_collision_monitor`, executable `collision_monitor`, name `collision_monitor` (`bringup_2d.launch.py:478-483`).
- It is conditioned on `mode == 'navigation'` and configured with input `/cmd_vel_nav`, output `/cmd_vel`, observation source `pointcloud`, topic `/cloud_registered_body` (`bringup_2d.launch.py:484-517`).
- Lifecycle manager is constructed (`bringup_2d.launch.py:520-530`).
- Both `ld.add_action(...)` lines are commented out (`bringup_2d.launch.py:572-573`).
- Default `mode` is `navigation` (`bringup_2d.launch.py:49`), but commented `ld.add_action` means the condition is never evaluated and the node is not launched.

Resolution:

- The V3.1 authority statement that Collision Monitor is commented out is correct for the current 2D live launch (`bringup_2d.launch.py`).
- The newer audit statement that Collision Monitor is active is correct for `bringup.launch.py`, but incorrect for `bringup_2d.launch.py`.
- Classification: C. different launch files have different states, plus B. the new audit misread inactive/commented code for the 2D launch.

## 3. Exact Watchdog Behavior

See `/home/dog/phase1_reports/PHASE1_WHEELCHAIR_CONTROLLER_WATCHDOG_TRUTH.md`.

Key corrections:

- `control_paused_` starts true (`wheelchair_controller_node.cpp:24`) and config `auto_start` is false (`wheelchair_controller_param.yaml:6`).
- Paused state does not continuously send frames by itself.
- Accepted commands while paused send zero immediately (`wheelchair_controller_node.cpp:171-173`).
- CAN timer is 20 ms by default (`wheelchair_controller_node.cpp:37`, `64-66`; config line 18).
- Before first accepted command, CAN timer sends nothing because `has_command_` is false (`wheelchair_controller_node.cpp:236-240`, `399`).
- While fresh and unpaused, CAN repeats the last command on the timer (`wheelchair_controller_node.cpp:242-249`).
- When stale or paused after a command, CAN sends one zero then stops (`wheelchair_controller_node.cpp:249-254`).
- Stale timeout is 500 ms by default (`wheelchair_controller_node.cpp:38`, `44`, `242-249`; config line 8).
- UDP does not use the timer watchdog because `CanTimerCallback()` exits unless `output_transport_ == "can"` (`wheelchair_controller_node.cpp:226-228`).
- CAN shutdown sends one zero before close (`wheelchair_controller_node.cpp:87-90`); UDP shutdown only closes socket (`82-86`).

## 4. Exact Nav2 / Pure Pursuit Interaction

Classification by condition:

| Condition | Classification | Evidence |
|---|---|---|
| `bringup.launch.py` navigation | `INDEPENDENT_COMPETING_CONTROLLERS` not present for physical output. Nav2 + Collision Monitor can publish `/cmd_vel`, but no Pure Pursuit or wheelchair_controller is launched there. | `bringup.launch.py:216-235`, `237-290`, `335-337`; no Pure Pursuit or wheelchair controller add_action. |
| `bringup_2d.launch.py` with default laser avoidance | `NAV2_SPEED_INPUT_PLUS_PLAN_NAV_STEERING` is not actually implemented because Collision Monitor is commented out; Pure Pursuit subscribes `/cmd_vel`, but no active Collision Monitor publisher is launched by this file. Actual physical-capable chain is plan_nav path plus YDLIDAR legacy filters, requiring manual wheelchair_controller. | `bringup_2d.launch.py:355-374`, `376-395`, `416-476`, `568-573`; `pure_pursuit_controller.cpp:50-51`, `88-91`, `304-308`. |
| If an external `/cmd_vel` publisher exists while Pure Pursuit is active | `UNUSED_CMD_VEL_SUBSCRIBER` for current source behavior. The callback only stores `cmd_vel_linear_velocity_` and `cmd_vel_angular_velocity_`; the only stop/modulation logic using these variables is commented out. | `pure_pursuit_controller.cpp:88-91`, `304-308`, `436-437`. |
| Nav2 and plan_nav simultaneously active under current `bringup_2d.launch.py` | `INDEPENDENT_COMPETING_CONTROLLERS` at computation level only: Nav2 computes `/cmd_vel_nav`; Pure Pursuit computes wheelchair commands from `/active_plan` or `/plan_nav`. Nav2 does not reach CAN unless a separate active bridge is introduced. | `bringup_2d.launch.py:355-414`, `416-476`, `568-573`; `pure_pursuit_controller.cpp:98-120`, `353-359`. |
| Collision Monitor publishes zero from an external/other launch while Pure Pursuit is active | `UNUSED_CMD_VEL_SUBSCRIBER`; zero is stored but does not stop output because the gating block is commented. | `pure_pursuit_controller.cpp:88-91`, `304-308`, `353-359`. |

Trace:

- Pure Pursuit subscribes to path topic parameter, `/baselink2map`, `/cmd_vel`, and `/laser_avoidance_state` (`pure_pursuit_controller.cpp:41-53`).
- `/cmd_vel` callback only stores two member variables (`pure_pursuit_controller.cpp:88-91`).
- The control timer returns without publishing when `path_.poses.empty()` (`pure_pursuit_controller.cpp:98-108`).
- Output is generated from path and TF geometry: lookahead point, angle, radius, and base velocity (`pure_pursuit_controller.cpp:127-359`).
- The only block that would stop on zero external `/cmd_vel` is commented out (`pure_pursuit_controller.cpp:304-308`).
- If a path exists and no `/cmd_vel` has arrived, the uninitialized stored cmd_vel members are still unused in active calculations; output can still be generated from path.
- If `/cmd_vel` exists and no path exists, the timer returns and publishes no wheelchair command (`pure_pursuit_controller.cpp:103-108`).

## 5. Corrected Publisher Counts And Categories

Detailed inventory is in `/home/dog/phase1_reports/PHASE1_COMMAND_AUTHORITY_MATRIX_RECONCILED.tsv`.

Summary:

| Category | Current count / state |
|---|---|
| A. `nav_msgs/Path` publishers | `plan_nav` `/plan_nav`; `plan_nav_laser_avoidance` `/active_plan`; Pure Pursuit `/path_future`; offline `path_publisher` `/mapping_path`; RTAB-Map also has map/global/local path publishers, but they are not chassis command authority. |
| B. `geometry_msgs/Twist` publishers | Nav2 controller publishes remapped `/cmd_vel_nav`; Collision Monitor publishes `/cmd_vel` in `bringup.launch.py` only. Collision Monitor is defined but not added in `bringup_2d.launch.py`. |
| C. `std_msgs/Float32MultiArray` wheelchair-command publishers | Pure Pursuit with avoidance publishes `/wheelchair_control_command_raw`; laser safety filter publishes `/wheelchair_control_command`; Pure Pursuit without avoidance publishes `/wheelchair_control_command`. The two Pure Pursuit launch definitions are mutually exclusive by `enable_laser_avoidance`; max one active. |
| D. direct CAN writers | `wheelchair_controller_node` is the only ROS-node CAN writer; `scripts/keyboard_can_control.py` is an additional manual direct SocketCAN writer. |
| E. UDP lower-controller writers | `wheelchair_controller_node` in `output_transport=udp`; `plan_nav/core/udp_sender.py`; `plan_nav/data/udp_send.py` helper. |
| F. future planned topics with no current publisher | `/vehicle_cmd_safe`, `/cmd_vel_nav_raw`, `/cmd_vel_nav_safe`, and Wheelchair Command Adapter are not implemented as current source publishers. |

Conditional duplicate Pure Pursuit nodes:

- Defined in launch: 2 (`bringup_2d.launch.py:376-414`).
- Maximum simultaneously active: 1.
- Conditions: with avoidance uses `IfCondition(enable_laser_avoidance)` (`379`); without avoidance uses `enable_laser_avoidance != 'true'` (`400`).

## 6. Reconciled Command Chains

### 1. `bringup.launch.py` navigation mode

Active publishers/subscribers:

- Nav2 active when `mode=navigation`, with `/cmd_vel` remapped to `/cmd_vel_nav` (`bringup.launch.py:216-235`).
- Collision Monitor active and lifecycle-managed; input `/cmd_vel_nav`, output `/cmd_vel`, observation `/cloud_registered_body` (`bringup.launch.py:237-290`, `335-337`).
- No Pure Pursuit and no wheelchair_controller are launched by this file.

Final chassis command publisher: none.
Direct CAN writer: none.
Physical motion possible: no from this launch alone.
Collision Monitor can affect physical motion: no, because no final chassis command path exists.
YDLIDAR safety filtering active: no legacy command filter; optional YDLIDAR driver/scan conversion only if separately enabled (`bringup.launch.py:56`, `84-114`).
Authority: unique but non-physical Nav2 Twist chain.

### 2. `bringup_2d.launch.py` with laser avoidance enabled

Active publishers/subscribers:

- Nav2 computes `/cmd_vel_nav` (`bringup_2d.launch.py:355-374`).
- `plan_nav_laser_avoidance`: `/plan_nav` + `/scan` -> `/active_plan` (`416-451`, `568`).
- Pure Pursuit with avoidance: `/active_plan` -> `/wheelchair_control_command_raw` (`376-395`, `569`).
- `laser_command_safety_filter`: `/wheelchair_control_command_raw` + `/scan` -> `/wheelchair_control_command` (`453-476`, `571`).
- Collision Monitor is not active because add_action is commented (`572-573`).
- wheelchair_controller is not launched by `bringup_2d.launch.py`.

Final chassis command publisher: `laser_command_safety_filter.py` on `/wheelchair_control_command`.
Direct CAN writer: none unless `wheelchair_controller_node` is manually started.
Physical motion possible: yes only with manual wheelchair_controller running and armed.
Collision Monitor can affect physical motion: no.
YDLIDAR safety filtering active: yes, through path and command filters.
Authority: hybrid computation (Nav2 + legacy Pure Pursuit) but physical-capable path is legacy/YDLIDAR/Pure Pursuit only.

### 3. `bringup_2d.launch.py` with laser avoidance disabled

Active publishers/subscribers:

- Nav2 computes `/cmd_vel_nav` (`bringup_2d.launch.py:355-374`).
- Pure Pursuit without avoidance subscribes `/plan_nav` and publishes `/wheelchair_control_command` (`397-414`, `570`).
- `plan_nav_laser_avoidance` and `laser_command_safety_filter` are disabled by `enable_laser_avoidance=false` (`416-476` conditions).
- Collision Monitor is not active because add_action is commented (`572-573`).
- wheelchair_controller is not launched by `bringup_2d.launch.py`.

Final chassis command publisher: Pure Pursuit on `/wheelchair_control_command`.
Direct CAN writer: none unless `wheelchair_controller_node` is manually started.
Physical motion possible: yes only with manual wheelchair_controller running and armed.
Collision Monitor can affect physical motion: no.
YDLIDAR safety filtering active: no.
Authority: hybrid computation, legacy physical-capable path.

### 4. Manual start of `wheelchair_controller_node`

Active subscriber: `/wheelchair_control_command` (`wheelchair_controller_node.cpp:60-62`).
Final chassis command publisher: whatever publishes `/wheelchair_control_command`.
Direct CAN writer: yes by default (`wheelchair_controller_param.yaml:3`; `wheelchair_controller_node.cpp:117-156`, `262-309`).
Physical motion possible: yes after unpaused/armed and with valid commands.
Collision Monitor can affect physical motion: only if its output is actually converted to `/wheelchair_control_command`; no current direct bridge exists.
YDLIDAR safety filtering active: only if upstream `laser_command_safety_filter.py` is active.
Authority: depends on upstream publisher uniqueness; lower-controller writer is the node itself.

### 5. Nav2 running without plan_nav path

If launched by `bringup.launch.py`: Nav2 + active Collision Monitor publish Twist chain, but no chassis command/CAN path exists. Physical motion is not possible.

If launched by `bringup_2d.launch.py`: Nav2 computes `/cmd_vel_nav`; Pure Pursuit has no path and returns without publishing (`pure_pursuit_controller.cpp:103-108`). Collision Monitor is inactive. Physical motion is not possible from Pure Pursuit without a path.

Authority: non-physical Nav2 computation only.

### 6. plan_nav / Pure Pursuit running without Nav2

`plan_nav` can publish `/plan_nav` when operation mode, route, and pose are available. With Pure Pursuit active:

- No avoidance: `/plan_nav` -> Pure Pursuit -> `/wheelchair_control_command`.
- Avoidance: `/plan_nav` -> `plan_nav_laser_avoidance` -> `/active_plan` -> Pure Pursuit -> `/wheelchair_control_command_raw` -> `laser_command_safety_filter` -> `/wheelchair_control_command`.

With manual wheelchair_controller running and armed, physical motion is possible. Collision Monitor cannot affect motion. YDLIDAR filtering applies only in the avoidance-enabled branch. Authority is legacy unique if no other `/wheelchair_control_command` publisher or direct CAN writer is running.

### 7. Nav2 and plan_nav running together

Current `bringup_2d.launch.py` default can run Nav2 computation and legacy Pure Pursuit computation simultaneously:

- Nav2: `/cmd_vel_nav`.
- Legacy path: `/plan_nav` -> optional `/active_plan` -> Pure Pursuit -> wheelchair command topic.
- Collision Monitor is defined but not launched in `bringup_2d.launch.py`.
- Pure Pursuit subscribes `/cmd_vel`, but the stored values are unused in active command generation.

Final chassis command publisher: legacy branch.
Direct CAN writer: manual wheelchair_controller only.
Physical motion possible: yes only with manual wheelchair_controller running and armed.
Collision Monitor can affect physical motion: no in current `bringup_2d.launch.py`.
YDLIDAR safety filtering active: yes when `enable_laser_avoidance=true`.
Authority: hybrid/ambiguous at navigation-computation level, legacy-only at current physical output.

## 7. Assessment Of Prior Audit

| Prior material statement | Classification | Correction |
|---|---|---|
| `PHASE1_STATIC_COMMAND_AUTHORITY_AUDIT_PASS` | `INCORRECT` | Safety-critical contradictions remained: 2D Collision Monitor state, `plan_nav` ROS publisher status, watchdog behavior, and extra manual CAN/UDP writers. |
| `plan_nav/` is standalone GUI, NOT a ROS publisher of paths | `INCORRECT` | `PlanPublisher` publishes `/plan_nav` as `nav_msgs/Path` after operation mode, route, and pose. |
| `plan_nav/main.py` publishes `/plan_nav` | `CORRECT_BUT_NEEDS_CLARIFICATION` | `main.py` invokes `MainWindow`; publisher is created in `core/nav_publisher.py`. |
| One final physical command chain: legacy Pure Pursuit | `CORRECT_BUT_NEEDS_CLARIFICATION` | True for normal physical-capable ROS chain, but manual `keyboard_can_control.py` and GUI UDP sender are separate lower-controller writers if run. |
| Nav2 branch: Collision Monitor -> `/cmd_vel` -> Pure Pursuit `/cmd_vel` subscriber | `INCORRECT` for `bringup_2d.launch.py`; `CORRECT_BUT_NEEDS_CLARIFICATION` for `bringup.launch.py` | Collision Monitor is active only in `bringup.launch.py`; commented out in `bringup_2d.launch.py`. Pure Pursuit is not launched in `bringup.launch.py`. |
| No `/vehicle_cmd_safe`, `/cmd_vel_nav_raw`, or `/cmd_vel_nav_safe` publishers exist | `CONFIRMED` | Current source does not implement selected V3.1 architecture topics. |
| `wheelchair_controller_node` is the only CAN writer | `CORRECT_BUT_NEEDS_CLARIFICATION` | It is the only ROS-node CAN writer in the normal command chain. `scripts/keyboard_can_control.py` is an additional manual direct CAN writer. |
| `wheelchair_controller_node` is not launched by robot_bringup | `CONFIRMED` | `bringup.launch.py` and `bringup_2d.launch.py` do not add it; separate package launch exists. |
| Pure Pursuit publishes `Float32MultiArray`, not `Twist` | `CONFIRMED` | `pure_pursuit_controller.cpp:56`, `353-359`. |
| Two Pure Pursuit instances are mutually exclusive | `CONFIRMED` | `bringup_2d.launch.py:376-414`. |
| Collision Monitor configured with `/scan` observation source | `INCORRECT` | Current Collision Monitor configs in both launch files use `pointcloud.topic: /cloud_registered_body`, not `/scan`. `/scan` is used by the YDLIDAR legacy filters. |
| Collision Monitor active in `bringup_2d.launch.py` | `INCORRECT` | Node construction exists but `ld.add_action` lines are commented. |
| CAN watchdog sends repeated zero when no command | `INCORRECT` | No frames before first command; one stale CAN zero then silence; UDP has no stale watchdog. |
| No UDP command writers except alternative transport in wheelchair_controller | `INCORRECT` | `plan_nav/core/udp_sender.py` and `plan_nav/data/udp_send.py` send lower-controller UDP packets. |

## 8. Remaining Unresolved Items

No source contradiction remains unresolved for the five requested areas.

Residual safety-critical facts for later phases:

- `bringup_2d.launch.py` launches Nav2 computation alongside legacy Pure Pursuit computation, while Collision Monitor is commented out.
- The selected V3.1 `/vehicle_cmd_safe` architecture is not implemented.
- Manual direct lower-controller writers exist and must be excluded by procedure or authority monitoring before physical tests.

## 9. Phase 1 Closure Decision

All requested contradictions are resolved from current source and the remaining safety-critical behavior is no longer ambiguous.

`PHASE1_COMMAND_AUTHORITY_RECONCILIATION_PASS`

## 10. Integrity Confirmation

Confirmed at report creation time:

- Source revision inspected: `c27961dc0eb156f4811e0d55c6793df304fd7ce6`.
- Verified V3.1 authority SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`.
- No source, launch, configuration, or existing documentation files were edited.
- Only new report files were created under `/home/dog/phase1_reports/`.
- No build was run.
- No ROS node or launch file was run.
- No ROS message was published.
- No CAN or UDP transport was opened.
- No package was installed or removed.
- No commit, tag, or push was performed.

Final git/tag cleanliness is verified after writing in the handoff.
