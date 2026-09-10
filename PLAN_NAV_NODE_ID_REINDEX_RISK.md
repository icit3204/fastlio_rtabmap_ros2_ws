# PlanNav Node-ID Reindex Risk

Status: documented only; no repair is authorized in P5A-MK2G2B.

The current `TopologyManager.remove_waypoint` behavior reindexes remaining
numeric node IDs after deletion. Edges are rewritten to the new numeric IDs.
This can alter persistent topology identity and can invalidate external or
previously prepared references that assumed stable node IDs. RouteMission
`node_ids`, `edge_ids`, and poses are generated from the current saved topology,
so a newly generated route remains internally consistent; previously retained
route intent or external node references may no longer identify the same
semantic node.

Explicit `edge_id` values reduce edge ambiguity but do not by themselves make
node identity immutable. A future architecture review should decide whether
node IDs must be stable/tombstoned, whether deletion should leave gaps, and how
migration/versioning should work. Any such change must include compatibility
analysis for edges, topology manifests, RouteSpec/RouteMission, and persisted
external references.

No node deletion or authoritative topology mutation was performed in this
milestone.
