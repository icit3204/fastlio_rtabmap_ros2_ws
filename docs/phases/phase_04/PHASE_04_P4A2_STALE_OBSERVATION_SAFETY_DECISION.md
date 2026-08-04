# Phase 4 P4-A.2 Stale Observation Safety Decision

## Decision Authority and Baseline

P4-A.2 records the additional Phase 4 command-safety interface required after
P4-B preflight observed installed Collision Monitor stale-source fail-open
behavior.

Baseline:

- Required branch: `main`
- Required committed HEAD and `origin/main` before this decision:
  `a7fe215384214aaec79bde3eec8d4df51cebcc8c`
- Baseline commit: `docs(phase4): freeze command safety interfaces`
- P4-A.1 decision document:
  `docs/phases/phase_04/PHASE_04_P4A1_INTERFACE_DECISIONS.md`
- P4-A.1 SHA256:
  `a54596859165c095cca3aebd198c660490ccc2ce3b324ace065d59fa1a8190ca`

Authority checksums:

- Primary architecture authority:
  `d9f9072bc6978fa9694927cab31101143e7397427f483f5586bc2dcbe61a199b`
- P4-A audit:
  `75a7e2eb79a1d7e71373caa4e9a9280b0dfb67eb7193938ce244abbf5a35cb26`
- Phase 3 closure:
  `169bb31a38319b7719b4c43fe0f712fafcc5a7475ea7db51b51eea578fd88103`

P4-A.1 remains authoritative except where this document explicitly adds or
clarifies Collision Monitor observation-validity handling.

## Observed Installed Behavior

Installed package:

- Debian package: `ros-humble-nav2-collision-monitor`
- Installed version: `1.1.20-1jammy.20260607.134559`
- Executable: `nav2_collision_monitor collision_monitor`

Observed classification:

`INSTALLED_COLLISION_MONITOR_STALE_SOURCE_FAIL_OPEN`

During deterministic synthetic LaserScan preflight before any Nav2 goal:

- fresh CLEAR behavior passed;
- SLOW behavior passed with ratio `0.30`;
- STOP behavior passed;
- STOP latency was approximately `0.004606755` seconds;
- recovery latency was approximately `0.024088051` seconds;
- after `/phase4/synthetic_scan` became SILENT, Collision Monitor logged
  source-timeout warnings;
- the stale source was no longer used for collision points;
- nonzero raw Twist continued through Collision Monitor output;
- `silent_safe_zero_count` was `0`;
- `silent_zero_latency_sec` was `null`;
- no automatic safe zero is guaranteed by this installed behavior.

This decision records only the behavior observed on the installed target
package.  It does not claim that another Nav2 version behaves identically.

## Added Phase 4 Validity Interface

Topic:

`/system/collision_monitor_valid`

Type:

`std_msgs/msg/Bool`

Sole approved publisher:

`collision_monitor_validity_monitor`

Approved subscriber:

`guarded_vehicle_cmd_gate`

Purpose:

`/system/collision_monitor_valid` is a generic permission indicating that the
required Collision Monitor process, lifecycle state, configured collision
observation source, and observation freshness are healthy.

This topic does not carry obstacle geometry.  It must not be published by
Collision Monitor itself unless a later verified interface provides equivalent
semantics.  It must not be inferred from the existence or freshness of
`/cmd_vel_nav_safe` alone.

## Validity Monitor Contract

Recommended node:

`collision_monitor_validity_monitor`

Initial Phase 4 responsibilities:

- use steady/monotonic receipt time;
- monitor the selected required observation topic;
- support the Phase 4 LaserScan and PointCloud2 profiles;
- check required message freshness;
- check Collision Monitor lifecycle and process availability;
- verify the expected configured source is selected;
- publish `true` only while all mandatory checks pass;
- publish `false` repeatedly whenever any mandatory check fails;
- publish at a fixed heartbeat;
- expose diagnostic reason and ages.

Initial source freshness limit:

`0.50` seconds

Validity heartbeat:

`20 Hz`

Initial recovery-stability window:

`0.50` seconds

Required behavior:

- transition to `false` no later than source timeout plus one heartbeat period;
- remain `false` while the source is stale;
- return `true` only after fresh valid observations and required process state
  have been stable for the configured recovery window.

Phase 4 validity monitoring consumes only the explicitly selected synthetic
source.  It must not monitor physical sensors in Phase 4.  Later Phase 5
profiles may select real Collision Monitor observation topics through explicit
architecture review and configuration.

## Revised Generic Safety Gate Prerequisites

The frozen Generic Safety Gate prerequisites become:

- `/cmd_vel_nav_safe` fresh;
- `/system/localization_valid` true and fresh;
- `/system/controller_valid` true and fresh;
- `/system/collision_monitor_valid` true and fresh;
- command authority unique and stable;
- valid explicit motion limits;
- no latched fault.

Collision Monitor validity permission timeout:

`0.50` seconds

False or stale Collision Monitor validity must:

- force repeated zero `/vehicle_cmd_safe`;
- latch `FAULT`;
- reject new arming;
- disallow automatic restoration of nonzero motion;
- require `arm=false`;
- require the fault cause to be removed;
- require all prerequisites to become stable;
- require a new `arm=true` request.

`/system/controller_valid` is not a substitute for
`/system/collision_monitor_valid`.

## Clarified Command Topic Semantics

The frozen topic remains:

`/cmd_vel_nav_safe`

Type:

`geometry_msgs/msg/Twist`

Clarified semantics:

- it is Collision Monitor-filtered Twist;
- with healthy and fresh observations it may implement STOP, SLOWDOWN, or
  APPROACH behavior;
- it is not independently fail-safe when required observations are stale in the
  installed package;
- it must never reach a chassis adapter directly;
- it may enter physical-authority preparation only through the Generic Safety
  Gate while `/system/collision_monitor_valid` is true and fresh.

No topic rename is approved.

## Revised Phase 4 Boundaries

P4-B resumed:

- retain the current preserved P4-B implementation;
- prove fresh-source CLEAR, SLOW, STOP, and release behavior;
- run LaserScan and PointCloud2 validation;
- run Nav2 scenarios that use fresh observations;
- preserve the stale-source fail-open result as an installed limitation;
- do not claim chain-level stale-source safety;
- do not implement `/vehicle_cmd_safe`.

P4-C:

- implement `guarded_vehicle_cmd_gate`;
- implement or provide the independent `collision_monitor_validity_monitor`;
- consume `/system/collision_monitor_valid`;
- prove stale synthetic collision observation causes repeated zero
  `/vehicle_cmd_safe` and latched `FAULT`;
- prove a stale Collision Monitor source cannot pass nonzero commands beyond
  the gate.

P4-D:

- implement the mock Wheelchair Command Adapter;
- retain its independent `/vehicle_cmd_safe` deadman.

P4-E:

- prove the complete fake closed-loop chain;
- include stale observation, stale safe Twist, false permissions, duplicate
  publishers, stop/slow, and no-bypass scenarios.

P4-F:

- regression and Phase 4 closure.

## Installed Package Patch Decision

Frozen decisions:

- do not patch files under `/opt/ros/humble`;
- do not fork Collision Monitor during current Phase 4;
- do not reinterpret source-timeout warnings as safe-stop output;
- do not inject synthetic STOP points merely to conceal source loss;
- do not compensate inside P4-B with an ad hoc Twist filter;
- apply fail-safe source-health enforcement at the generic gate permission
  boundary.

A future package-version evaluation may revisit this decision only through
explicit architecture review.

## Threat Model Update

Failure path:

1. observation source becomes stale;
2. Collision Monitor warns and forwards raw command;
3. `/cmd_vel_nav_safe` remains fresh and nonzero;
4. a gate checking only safe-Twist freshness would incorrectly remain armed.

Required prevention:

- independent `/system/collision_monitor_valid`;
- steady-clock freshness;
- false or stale validity latches gate `FAULT`;
- repeated generic zero;
- explicit re-arm.

Required tests:

- stale LaserScan;
- stale PointCloud2;
- Collision Monitor process or lifecycle loss;
- validity publisher loss;
- safe Twist remains nonzero while validity becomes false;
- no nonzero `/vehicle_cmd_safe` after the permitted response interval.

## Supersession and Change Control

P4-A.1 remains authoritative except where P4-A.2 explicitly adds or clarifies
collision-observation validity.  P4-A.2 supersedes any assumption that
Collision Monitor source timeout itself generates safe zero.  All other P4-A.1
topics, timing decisions, and ownership rules remain frozen.

Any later removal or renaming of `/system/collision_monitor_valid` requires:

- Main ChatGPT architecture review;
- evidence from the installed target package;
- safety impact analysis;
- regression plan;
- a separate decision document.
