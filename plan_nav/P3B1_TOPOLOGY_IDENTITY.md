# P3-B.1 plan_nav Topology Identity

P3-B.1 adds persistent route identity metadata to `plan_nav` without changing
legacy dense `/plan_nav` publication and without publishing `RouteMission`.

## Schema v2

`edges.txt` v2 uses:

```text
# edge_id, from_id, to_id, length_m, direction, traj_file
```

`edge_id` is a nonempty stable string persisted with each edge record. Deleted
edge IDs are not inherited by surviving records. Direction is exactly `uni` or
`bi`; trajectory filenames are optional but must be unique when present.

Legacy four- and five-column edge files can be checked and migrated explicitly,
but normal GUI loading must not silently rewrite old topology assets.

## Manifest

`topology_manifest.json` is stored next to `nodes.txt` and `edges.txt`.

- `topology_id` identifies the dataset across revisions.
- `topology_version` is `sha256:<hash>` over navigation-relevant content:
  node ID and pose, edge ID/endpoints/length/direction/trajectory filename,
  and referenced trajectory content hashes.
- Annotation text is excluded because it is not navigational.

Any navigation-relevant topology edit changes `topology_version`. Missions
created from a previous topology version must be considered invalid.

## Migration

Run:

```bash
python3 plan_nav/tools/migrate_topology_identity.py --check --work-dir plan_nav/underGround_split1
python3 plan_nav/tools/migrate_topology_identity.py --apply --work-dir plan_nav/underGround_split1
```

Existing legacy edges receive deterministic IDs by persisted record sequence:

```text
edge-000001
edge-000002
...
```

The migration writes through temporary files and atomic rename. The external
P3-B.1 preservation set is the rollback source for the migrated topology.

## Route Resolution

Sparse route construction resolves every Dijkstra node transition explicitly:

- stored A -> B edge: direction `+1`
- stored B -> A edge with `bi`: direction `-1`
- stored B -> A edge with `uni`: rejected
- zero candidates: `ROUTE_EDGE_NOT_FOUND`
- multiple candidates: `AMBIGUOUS_ROUTE_EDGE`

The builder never selects the first match silently and never infers edge
identity from dense Path geometry.

## Sparse Route Contract

The ROS-independent route spec mirrors the frozen `RouteMission` fields:

- map frame
- mission ID supplied by caller
- deterministic `route-sha256:<hash>` route ID
- topology version from the manifest
- ordered node IDs
- persisted edge IDs
- edge directions
- one pose per node

Consecutive sparse waypoints closer than 0.55 m are rejected with
`CONSECUTIVE_WAYPOINTS_TOO_CLOSE`; nodes are not silently removed.

## Scope

Dense `/plan_nav` remains the legacy/display Path output. P3-B.1 does not
publish `RouteMission`, connect Mission Manager, add GUI start/cancel controls,
launch Nav2, or touch physical command paths. P3-B.2 is the later publisher and
integration milestone.
