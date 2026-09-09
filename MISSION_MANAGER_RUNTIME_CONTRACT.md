# Mission Manager runtime contract (P5A-MK2F4)

The production mission authority is `parking_robot_mission_manager/mission_manager_node.py` (`/mission_manager`). It accepts one typed `parking_robot_interfaces/msg/RouteMission` on `/mission/route`, exposes `/mission/start` and `/mission/cancel` (`std_srvs/srv/Trigger`) and `/mission/pause` (`std_srvs/srv/SetBool`), and is the sole mission-driven client of `/navigate_to_pose`.

`RouteMission` carries `mission_id`, `route_id`, `topology_version`, ordered `node_ids`, matching `poses`, inter-node `edge_ids`, and `edge_directions`. `plan_nav/core/mission_bridge.py` is the existing high-level adapter from a topology-backed `RouteSpec`; it publishes no wheel command and does not auto-start a mission.

The manager dispatches one map-frame pose at a time. A succeeded goal advances exactly once. Rejection, abort, unexpected cancellation, cancel protocol failure, or timeout terminates the mission as failed; it never silently skips a waypoint or retries indefinitely.

The Phase-5 canonical launch binds the manager's read-only progress observer to `/cmd_vel_nav`, Collision Monitor output `/cmd_vel`, Gate state `/phase5/gate_test/state`, `/gate_to_labmate_bridge/diagnostics`, `/Odometry`, and `odom_chassis -> base_footprint`. These observations do not create motion authority.

Each newly accepted NavigateToPose goal starts a distinct bounded command
observation epoch. Internally, `INITIAL_PRIMING` contains two phases:
`PAIR_NOT_YET_ESTABLISHED` and `PAIR_ESTABLISHED`. Before the first trustworthy
raw/Collision-Monitor pair, unequal-rate 20 Hz raw and 10 Hz safe streams may
temporarily leave the causal frontier PENDING, AMBIGUOUS, or locally STALE.
That condition remains acquisition—not runtime failure—only while both
epoch-local streams are individually fresh. Missing raw command retains its
1.0 s acquisition timeout; a raw or safe stream that has started and then
exceeds the unchanged 0.25 s command freshness fails immediately. If healthy
streams cannot establish a pair within the existing 2.0 s activation deadline,
the deterministic reason is `INITIAL_PAIR_ACQUISITION_TIMEOUT`.

The first valid causal pair permanently marks that action epoch
`PAIR_ESTABLISHED`. Normal runtime `COMMAND_PAIR_STALE` handling then applies
immediately and cannot regain priming grace. Only a genuinely new accepted
goal—initial dispatch, resume re-send, or next waypoint—starts a new epoch.
Perception/localization/controller invalidity, explicit Gate fault, adapter
invalidity, stale feedback/odometry/TF, and action failure are never masked by
first-pair acquisition.

For stationary qualification, Gate arm readiness is proven on one monotonic
clock. Both `/cmd_vel_nav` and `/cmd_vel` must remain continuously fresh for
at least 0.30 s, each stream must have no gap above 0.15 s, each latest sample
and the Gate validity diagnostic must be at most 0.10 s old, all three Gate
validity inputs must be true, and the latest MPPI/CM commands must be forward
motion commands. A gap resets only that stream's stability tail. These bounds
do not modify the production Gate's 0.25 s safe-input watchdog.

`/mission/state` and `/mission/status` use reliable transient-local history.
A newly created `ros2 topic echo --once` process can therefore print an older
cached sample, not the current authoritative state. Runtime adjudication must
use the timestamped continuous stream (or a service response plus continuous
recorder), never an isolated first historical sample.
