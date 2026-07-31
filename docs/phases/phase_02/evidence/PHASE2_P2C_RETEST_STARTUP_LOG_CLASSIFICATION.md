# Phase 2 P2-C Retest Startup Log Classification

| Timestamp/phase | Node | Message summary | Repeated | Cause | Impact | Disposition | Next action |
|---|---|---|---|---|---|---|---|
| startup | phase2_map_to_odom_static_tf | Old-style static_transform_publisher arguments are deprecated | one-time | Humble CLI deprecation for positional transform arguments | Non-blocking; static transform published correctly | ACCEPTED | Optionally modernize launch syntax in later cleanup only |
| activation | controller_server | Parameter controller_server.verbose not found | one-time | MPPI/controller checks optional verbose parameter absent in Humble config | Non-blocking; controller reached ACTIVE | ACCEPTED | No action required for P2-C |
| shutdown | all nodes | Lifecycle preshutdown/deactivate/cleanup/destroy messages | one-time | Normal group SIGINT shutdown | Clean process exit | ACCEPTED | No action required |
