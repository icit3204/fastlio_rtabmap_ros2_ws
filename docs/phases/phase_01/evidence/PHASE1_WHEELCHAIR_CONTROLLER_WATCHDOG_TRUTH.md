# Phase 1 Wheelchair Controller Watchdog Truth

Date: 2026-07-31
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`
Source revision: `c27961dc0eb156f4811e0d55c6793df304fd7ce6`
Authority document SHA256: `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`

## Source Inspected

- `src/wheelchair_controller/src/wheelchair_controller_node.cpp`
- `src/wheelchair_controller/config/wheelchair_controller_param.yaml`
- `src/wheelchair_controller/launch/wheelchair_controller.launch.py`

No ROS node was launched. No CAN or UDP transport was opened.

## Exact Behavior

| Question | Answer |
|---|---|
| While `control_paused_` is true, are any CAN or UDP frames sent? | Yes, but only on command-related events. A received valid command while paused calls `SendControlMsg(0,0,0)` and sends one immediate zero over the selected transport (`wheelchair_controller_node.cpp:171-173`, `183-199`). Pressing space to pause also calls `SendControlMsg(0,0,0)` (`352-356`). For CAN, the timer may then send one additional zero because `SendControlMsg` resets `sent_timeout_stop_` (`183-192`, `226-259`). If no command/event has occurred, paused state alone sends nothing. |
| After arming but before the first command, are repeated zero frames sent? | No. `has_command_` starts false (`399`), and the CAN timer returns immediately when no command exists (`236-240`). UDP has no timer send path. |
| While commands are fresh, how frequently are frames sent? | CAN: each accepted command sends immediately (`176-180`, `194-198`) subject to `min_command_interval_ms` throttle (`158-165`), and the 20 ms CAN timer repeats the latest command while fresh (`64-66`, `226-259`). UDP: each accepted command sends immediately only (`194-195`, `201-223`); there is no repeated UDP heartbeat. |
| When a command becomes stale, what happens? | CAN: the first timer tick after stale/paused condition sends one zero frame, sets `sent_timeout_stop_ = true`, and later timer ticks return without sending (`242-254`, `257-259`). It is not repeated zero. UDP: stale timeout is not applied because the timer exits when `output_transport_ != "can"` (`226-228`). |
| Does the stale timer use 500 ms? | Yes by default. `command_timeout_ms` is declared as `500.0` and read into `command_timeout_ms_` (`38`, `44`); config also sets `500.0` (`wheelchair_controller_param.yaml:8`). The stale comparison is `elapsed_ms <= command_timeout_ms_` (`242-249`). |
| Is the 20 ms timer a transmit heartbeat, stale check, or both? | CAN only: both. It repeats the latest fresh command every configured `can_send_period_ms` and performs stale/paused one-shot stop handling (`64-66`, `226-259`). It is not a UDP heartbeat. |
| What happens on shutdown? | Destructor closes UDP without sending a stop (`82-86`). If CAN socket is open, destructor sends one zero CAN frame before closing (`87-90`). |
| Does UDP follow identical watchdog behavior? | No. UDP and CAN share immediate `SendControlMsg` on accepted command/pause events (`183-199`), but only CAN is covered by the 20 ms timer and stale timeout (`226-228`). UDP does not send a stale stop and does not send a shutdown stop. |

## State And Branch Trace

Constructor initializes `control_paused_(true)` and `running_(true)` (`wheelchair_controller_node.cpp:24`). Parameter `auto_start` can clear paused state before subscriptions/timers are created (`46-48`); current config keeps it false (`wheelchair_controller_param.yaml:6`).

Transport is selected by `output_transport`: `"udp"` calls `InitUdp()` and opens a nonblocking UDP socket (`50-51`, `97-115`); `"can"` calls `InitCan()` and opens/binds SocketCAN (`52-53`, `117-156`). Current config selects CAN (`wheelchair_controller_param.yaml:3`).

The subscriber listens on `/wheelchair_control_command` with `std_msgs::msg::Float32MultiArray` (`60-62`). `CommandCallback` throttles based on `last_send_time_`, then rejects arrays smaller than 3 (`158-169`). For accepted arrays while paused, it sends zero and returns (`171-173`). For accepted arrays while unpaused, it forwards radius, velocity, and distance to `SendControlMsg` (`176-180`).

`SendControlMsg` stores the last command, sets `has_command_ = true`, resets `sent_timeout_stop_ = false`, and immediately sends on the selected transport (`183-199`).

`CanTimerCallback` only runs useful logic for CAN (`226-228`). If no command has ever been accepted, it returns (`236-240`). If the last command is fresh and control is not paused, it repeats the last command (`242-249`). Otherwise, it sends exactly one zero command and latches `sent_timeout_stop_` (`249-254`).

## Correction To Prior Inventory

The prior statement "Watchdog: 20ms timer - sends repeated zero when no command" is incorrect for current source. The current code sends no frames before the first accepted command, repeats fresh CAN commands, sends one stale CAN stop frame, then stops output until a new accepted command/pause event resets the latch. UDP does not have equivalent watchdog behavior.

PHASE1_WHEELCHAIR_CONTROLLER_WATCHDOG_TRUTH_COMPLETE
