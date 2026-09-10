# PlanNav Runtime Control Authority Audit

Milestone: P5A-MK2G2B

## Frozen boundary

PlanNav owns topology editing, directed route selection, RouteSpec construction,
and explicit typed `RouteMission` publication. The standalone Operator GUI owns
mission START, PAUSE, RESUME, and CANCEL. RViz owns runtime spatial display.

## Audited runtime controls

| GUI/control | Previous source | Interface/destination | Protocol | Required by authoring | Duplicate/overlap | Motion-capable | Disposition |
|---|---|---|---|---|---|---|---|
| Debug/operation switch | `Sidebar.ModeSwitchButton` -> `MainWindow._on_mode_changed` | Started pose receiver and `/plan_nav` publisher | TF subscription + `nav_msgs/Path` | No | Runtime spatial behavior belongs to RViz/current navigation stack | Potentially, because `/plan_nav` belonged to the historical controller path | Removed from normal UI; legacy entry point hard-disabled |
| Authority selector | Sidebar `legacy` / `mission_nav2` combo | Selected UDP or Mission Manager path | Internal mode | No | Obsolete after authority freeze | Yes in legacy mode | Removed |
| Legacy route transmission | `MainWindow._start_udp_send` -> `core/udp_sender.py` | Configured IPv4 address, default `127.0.0.1:14550` | UDP, 16-byte big-endian `struct.pack('>dd', R, v)` | No | Replaced by RouteMission -> Mission Manager -> Nav2 | YES: payload is steering radius and velocity and historical receivers describe Pure Pursuit control | Normal binding and sender invocation removed; historical module retained but unbound |
| UDP endpoint configuration/preview | `LogPanel`, `Sidebar` | IP/port and frame preview | GUI configuration | No | Obsolete runtime transport | Indirectly enables motion-capable path | Removed |
| Pursuit controls | `LogPanel` -> `MainWindow._on_traj_params_change` | Legacy sender radius/speed parameters | In-process | No | Controller authority now MPPI | YES through legacy sender | Removed |
| `/plan_nav` path publication | `MainWindow._start_plan_publisher` -> `core/nav_publisher.py` | `/plan_nav` | `nav_msgs/msg/Path` | No | Runtime path authority now Nav2/RViz | Historically controller-facing | No normal binding; legacy entry point hard-disabled; module retained unbound |
| Publish RouteMission | `MainWindow._on_publish_mission_requested` -> `MissionBridgeNode.publish_route` | `/mission/route` | `parking_robot_interfaces/msg/RouteMission`, reliable transient-local | YES | No; this is the topology-to-mission intent boundary | NO by itself; it does not start a mission | Preserved and made the sole PlanNav ROS output |
| START mission | Sidebar -> MissionBridge Trigger client | `/mission/start` | `std_srvs/srv/Trigger` | No | Operator GUI duplicates/owns it | Initiates runtime navigation | Removed from UI and bridge |
| PAUSE/RESUME mission | Sidebar -> MissionBridge SetBool client | `/mission/pause` | `std_srvs/srv/SetBool` | No | Operator GUI duplicates/owns it | Runtime supervisory control | Removed from UI and bridge |
| CANCEL mission | Sidebar -> MissionBridge Trigger client | `/mission/cancel` | `std_srvs/srv/Trigger` | No | Operator GUI duplicates/owns it | Runtime supervisory control | Removed from UI and bridge |
| Mission status widget | MissionBridge `/mission/state` subscriber | `/mission/state` | `parking_robot_interfaces/msg/MissionState` | No | Operator GUI owns runtime status | No | Subscription and lifecycle status UI removed; route-publication status retained |
| Direct NavigateToPose | None found | None | None | No | N/A | N/A | None present |
| Velocity/CAN publisher | None found in normal PlanNav GUI | None | None | No | N/A | N/A | None present |

## UdpSender trace

`core/udp_sender.py` performs Pure Pursuit-like radius/speed calculation and
opens an IPv4 datagram socket. For each dense route point it sends `(R, v)` as
two big-endian doubles to the configured destination. The default is
`127.0.0.1:14550`; `tools/udp_receiver.py` and `data/udp_res.py` are historical
receivers. A historical wheelchair-controller launch file is also present in
`plan_nav/data`, but no current normal PlanNav binding proves a mandatory
topology-authoring dependency. The path is therefore classified:

`LEGACY_RUNTIME_CONTROL_NOT_REQUIRED_BY_PLAN_NAV_CORE`.

No UDP packet was transmitted during MK2G2B.

## Post-separation ownership

The normal GUI has one ROS-capable action: explicit publication of the prepared
typed route on `/mission/route`. Publication does not call START and does not
create a `NavigateToPose` client. The publication thread queues a request until
its ROS publisher is ready, then reports only publication completion.

Historical UDP, pose-receiver, and `/plan_nav` modules are retained for source
history but are unbound from the normal UI. Legacy operation entry points in
`MainWindow` return before constructing runtime components.
