# Mission Manager GUI status contract

The future GUI consumes `/mission/state` (`parking_robot_interfaces/msg/MissionState`, reliable/transient-local). Required fields are already present: mission and route IDs, numeric state, current waypoint index, completed and total waypoint counts, normalized progress, active NavigateToPose goal UUID, reason code, and detail.

Operational diagnostics are `/mission/status` and `/mission/block_reason` (`diagnostic_msgs/msg/DiagnosticStatus`). Commands are `/mission/start`, `/mission/cancel`, and `/mission/pause`; `SetBool(true)` pauses and `SetBool(false)` resumes. The GUI must not send NavigateToPose directly and must not publish velocity commands. Gate state remains a separate safety authority and should be displayed separately from mission lifecycle.

Because the state and diagnostic topics are transient-local with retained
history, the GUI must consume the stream and select the newest header/receipt
state. It must not interpret the first sample returned to a new subscriber as
the latest state. A rejected service response takes precedence over an older
cached display sample until a current state update is received.
