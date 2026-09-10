# P5A MK2H1R Git Staging Manifest

## Checkpoint scope

This checkpoint preserves the independent MK2H1 live-stack foundation and
the accepted MK2H1R canonical map-frame route reconciliation. It does **not**
claim a live MK2H1 mission pass: route publication and Operator GUI START were
correctly withheld in the blocked live run.

## Included

| Category | Files | Reason |
| --- | --- | --- |
| A — H1 live composition | `src/robot_bringup/CMakeLists.txt`, `src/robot_bringup/package.xml`, `src/robot_bringup/launch/bringup.launch.py`, `src/robot_bringup/launch/mk2h1_live_operator_mock.launch.py` | Real localization-validity opt-in, MockTransport-only composition, package dependencies, and exact topology-version binding. |
| B — H1 contract test | `src/robot_bringup/test/test_mk2h1_live_operator_contract.py` | Prevents demo-fixture, direct-motion, physical-transport, and topology-version regressions. |
| C — H1 runtime evidence contracts | `P5A_MK2H1_LIVE_OPERATOR_VIS_RUNTIME_CONTRACT.md`, `P5A_MK2H1_LIVE_DISPLAY_AUTHORITY_MATRIX.csv`, `P5A_MK2H1_LIVE_TIMING_SUMMARY.csv` | Documents the blocked pre-START live foundation without claiming a mission pass. |
| D — H1R tool and test | `plan_nav/tools/reconcile_rtab_topology.py`, `plan_nav/tests/test_rtab_topology_reconciliation.py` | Reproducible, non-overwriting rigid SE(2) topology conversion and focused regression. |
| E — H1R contracts | `P5A_MK2H1R_TOPOLOGY_INVENTORY.csv`, `P5A_MK2H1R_PLANNAV_COORDINATE_CONTRACT.md`, `P5A_MK2H1R_FRAME_RECONCILIATION_EVIDENCE.csv`, `P5A_MK2H1R_CURRENT_POSE_ADMISSION_CONTRACT.md`, `P5A_MK2H1R_H1_WIP_CLASSIFICATION.md`, `P5A_MK2H1R_RETRY_CONTRACT.md` | Map/topology provenance, transform evidence, start semantics, and bounded retry authority. |
| F — canonical transformed topology | `map/rtabmap_2d/nodes.txt`, `edges.txt`, `edge_1_traj.txt` through `edge_6_traj.txt`, `topology_manifest.json`, `reconciliation_manifest.json` | Nine-node, six-edge map-frame topology. These are deliberately force-added because `map/*` is globally ignored. |
| G — checkpoint docs | `PHASE5A_MK2H1R_ROUTE_RECONCILIATION_GIT_CHECKPOINT.md`, this manifest | States the bounded qualification boundary and commit inventory. |

## Explicitly excluded

| Category | Files or paths | Disposition |
| --- | --- | --- |
| Canonical reference DB | `map/rtabmap_2d.db` | SHA-256 verified unchanged: `73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`; never staged. |
| Historical source topology/database | `map/采集/图书馆2/`, `map/采集/图书馆2.db` | Preserved in place as ignored historical source material; not copied or rewritten. |
| Runtime RTAB copies | `/home/dog/phase5_runtime/rtabmap/` | Runtime-only, outside repository staging. |
| Supervisor evidence | `/home/dog/phase5_reports/` | Kept outside the repository. |
| Pre-existing unrelated tracked edit | `键盘控制.md` | Known trailing whitespace change; untouched and unstaged. |
| Unrelated historical/local material | `B2A*`, `RUN.TXT`, `can_faliar.md`, `keyboard_can_control.py`, `docs/phases/phase_04/*`, `docs/phases/temp text.txt`, `frames_*.gv` | Excluded; not part of MK2H1/MK2H1R. |
| Generated/cache output | `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc` | Excluded. |

## Integrity record

- Canonical topology version: `sha256:f8bd2b688ce827446ea88f42f19777f97c918c2dc1832091fa9b379a85000d70`
- Canonical route probe: `4 -> 1 -> 2`, `edge-000002`, `edge-000001`, frame `map`, Mission Manager validation `VALID`.
- Physical CAN and robot motion are outside this checkpoint and remain unqualified.
