# Phase 5A R3L-R21 demo baseline checkpoint

This document records the frozen stationary-clear demo baseline. It contains
no runtime database or generated evidence.

## Checkpoint

- Baseline branch: `main`
- Baseline commit: recorded after the approved checkpoint commit
- Tag: `phase5a_r3l_r21_demo_baseline`
- Development branch: `dev/r3h_perception_reconciliation`
- CAN: unavailable at checkpoint time; no CAN TX was performed
- Robot motion: none during checkpoint creation

## R21 capability

- One R3H sensor/localization composition with singleton protection.
- FAST-LIO/localization startup freshness protection and calibrated TF.
- Protected runtime copy for RTAB localization.
- Current Smac/MPPI production radius: `1.75 m`.
- T-mini live Nav2 obstacle layers and Collision Monitor/Gate chain.
- Native MK-mini sender/backend source with temporary, explicitly bounded
  `TEMPORARY_MKMINI_OPEN_LOOP` demo mode.
- R21 clear-area result: MID-360 CLEAR, T-mini CLEAR, localization/TF healthy,
  planner current-to-WP01 PASS.
- The prior table-side MID trigger disappeared after manual relocation.

## Deferred development work

1. Filtered MID-360 observations into Nav2 costmaps.
2. Sensor-world obstacle reconciliation.
3. Future motion-aware Collision Monitor geometry qualification.
4. Final-robot chassis feedback qualification.

The temporary MK-mini self-body crop and open-loop policy are platform-specific
development authorities and must be requalified or disabled on the final robot.
