# Temporary MK-mini open-loop policy

`TEMPORARY_MKMINI_OPEN_LOOP` is explicitly limited to the temporary MK-mini
development robot under professor/supervisor authorization. The Generic Gate's
fresh `/vehicle_cmd_safe` output is converted to a bounded MK-mini command and
sent by the sole deterministic native SocketCAN authority.

Command-side enforcement remains: forward only, speed at most 0.040 m/s,
steering at most ±30 degrees, stale command to N/zero, Gate disarm or Collision
Monitor STOP to zero, native command TTL, shutdown stops TX, and only one CAN
writer. Chassis feedback, alive/checksum, reported speed, reported steering,
fault status, and wheel odometry remain recorded diagnostics but do not admit
or revoke motion in this temporary mode.

FINAL_ROBOT_REQUIREMENT: restore `FEEDBACK_INTERLOCKED` mode and qualify full
chassis-feedback freshness, alive/checksum semantics, hard-fault supervision,
steering tracking, chassis speed accuracy, and wheel-odometry metric scale
before final production deployment.
