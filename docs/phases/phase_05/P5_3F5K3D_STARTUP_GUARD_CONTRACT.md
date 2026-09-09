# P5-3F5K3D startup stale-measurement containment

The Phase-5 FAST-LIO launch enables an opt-in startup freshness guard with:

- `startup_freshness_guard_enabled: true`
- `startup_max_lidar_age_sec: 0.50`
- `startup_fresh_consecutive_groups: 10`

The generic FAST-LIO launch defaults the guard to disabled, preserving legacy
behavior for non-Phase-5 deployments.

The guard runs after `sync_packages()` succeeds and before `flg_first_scan`,
`ImuProcess::Process()`, map initialization, or cloud/Odometry publication.
It compares the synchronized `lidar_end_time` against the current ROS clock.
An age in `[0, 0.50]` seconds is fresh. Non-finite, future, reversed, and
older-than-threshold timestamps are rejected during startup.

While not admitted, stale synchronized groups are consumed and dropped. Fresh
groups are consumed while a consecutive fresh window is accumulated. The tenth
fresh group is the admission boundary and is also discarded; normal FAST-LIO
processing starts with the next synchronized group. Thus stale startup data
cannot initialize IMU processing, mapping, cloud output, or Odometry output.

After admission the guard is bypassed and FAST-LIO runtime behavior is
unchanged. Runtime freshness monitoring remains mandatory; this is not a
permanent estimator data-drop policy and does not claim the natural
executor/DDS trigger is fixed.

## Startup ingress fast-drain layer

When the same opt-in guard is enabled and startup has not yet been admitted,
the Livox `CustomMsg` callback first applies a cheap header-time check using
the node's ROS clock. Non-finite, future, reversed, or older-than-0.50-second
headers are discarded before `p_pre->process()`, point conversion, buffer
insertion, and synchronization. The header is the scan/base timestamp, so
this is intentionally conservative; it does not replace the authoritative
synchronized `lidar_end_time` check described above.

Ingress rejection resets the one shared synchronized fresh-window counter.
Thus an ingress stale message cannot be hidden between two groups of
synchronized fresh measurements. The synchronized K3D guard remains in the
path and still controls estimator admission. Once it admits, ingress
filtering is permanently bypassed for that run and the legacy runtime path is
restored.

The ingress counter is diagnostic only (`startup_ingress_stale_dropped` and
invalid-timestamp counts); no point data are decoded by the filter and no
sensor timestamps are restamped. This layer contains the proven stale-startup
queue availability failure mode; it does not claim to resolve the intermittent
natural executor/DDS scheduling trigger.

## Synchronized stale-prefix fast drain

While the guard is enabled and not yet admitted, the timer callback may
repeatedly call `sync_packages()` only when the previous synchronized group
was classified `DROP_STALE`. Each successful synchronization consumes one
front LiDAR group and its associated IMU history through the unchanged
`sync_packages()` logic. The callback stops immediately on `sync_packages()`
false, on the first `WAIT_FRESH_WINDOW`, or on `ADMIT`.

The first fresh group therefore starts the ordinary ten-group window but is not
followed by a tight-loop consumption of the fresh tail. The tenth group remains
the discarded admission boundary, and the next timer callback enters the
existing estimator path. Disabled and already-admitted operation retain the
original one-group timer behavior.

The production executor is single-threaded, so sensor callbacks cannot append
new groups concurrently while this timer callback drains. The finite queue and
one-pop-per-successful-synchronization property provide the loop bound; there
is no sleep, busy retry, new parameter, QoS change, or executor change. A
low-rate `Startup synchronized stale-prefix drain batch` diagnostic records
batch sizes greater than one and the maximum observed batch for later live
qualification.

The proven pre-callback queue/executor-starvation mechanism can create stale
startup queues, and intermittent natural production stale starts remain an
availability issue. This repair contains stale startup contamination when the
Phase-5 opt-in is enabled; it does not resolve the underlying scheduling/DDS
trigger.
