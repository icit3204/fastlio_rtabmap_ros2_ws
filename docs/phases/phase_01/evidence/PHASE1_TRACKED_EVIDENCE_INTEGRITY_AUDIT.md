# Phase 1 Tracked Evidence Integrity Audit

Date: 2026-08-01
Machine: Jetson 2

## Summary

| Metric | Value |
|---|---|
| Tracked evidence files in index | 20 |
| Files verified (SHA256 + size) | 20/20 ✅ |
| Untracked authoritative reports | 9 |
| Duplicate rows | 0 |
| Binary/secret/build artifacts | 0 |
| Evidence preservation status | INCOMPLETE |

## Tracked evidence verification

All 20 files in `docs/phases/phase_01/PHASE_01_EVIDENCE_INDEX.md` exist
at their recorded paths with matching SHA256 checksums and byte sizes.

| Category | Count | Status |
|---|---|---|
| Native build evidence | 6 | ✅ |
| ABI verification | 2 | ✅ |
| Sysroot recovery | 1 | ✅ |
| Feature/decision audits | 2 | ✅ |
| Authority compliance | 2 | ✅ |
| Governance artifacts | 4 | ✅ |
| RViz profile | 1 | ✅ |
| Hard-coded path remediation | 4 | ✅ |

## Untracked authoritative reports

Nine reports critical to Phase 1 authority closure exist only under
`/home/dog/phase1_reports/` and are NOT preserved in Git-tracked evidence:

### Command-authority reconciliation (4 reports)

| Report | Size |
|---|---|
| `PHASE1_COMMAND_AUTHORITY_RECONCILIATION.md` | Exists — not tracked |
| `PHASE1_COMMAND_AUTHORITY_MATRIX_RECONCILED.tsv` | Exists — not tracked |
| `PHASE1_WHEELCHAIR_CONTROLLER_WATCHDOG_TRUTH.md` | Exists — not tracked |
| `PHASE1_STATIC_COMMAND_PUBLISHER_AND_AUTHORITY_AUDIT.md` | Exists — not tracked |

### CAN/lower-controller inventory (1 report)

| Report | Size |
|---|---|
| `PHASE1_CAN_AND_LOWER_CONTROLLER_WRITER_INVENTORY.md` | Exists — not tracked |

### Additional command authority (1 report)

| Report | Size |
|---|---|
| `PHASE1_COMMAND_AUTHORITY_MATRIX.tsv` | Earlier version — partially superseded by RECONCILED |

### Collision Monitor audits (3 reports)

| Report | Size |
|---|---|
| `PHASE1_COLLISION_MONITOR_HUMBLE_CAPABILITY_AND_SCHEMA_AUDIT.md` | Exists — not tracked |
| `PHASE1_COLLISION_MONITOR_CONFIG_COMPATIBILITY_MATRIX.tsv` | Exists — not tracked |
| `PHASE1_COLLISION_MONITOR_POINTCLOUD_REQUIREMENTS.md` | Exists — not tracked |

## No contradictory evidence

No earlier/obsolete report that contradicts the final reconciled authority
remains tracked in evidence. The earlier `PHASE1_COMMAND_AUTHORITY_MATRIX.tsv`
is superseded by `PHASE1_COMMAND_AUTHORITY_MATRIX_RECONCILED.tsv` but neither
is tracked — no contradiction exists in tracked evidence.

## Classification

- **TECHNICAL_CLOSURE_COMPLETE**: ✅ All authority tasks satisfied
- **TRACKED_EVIDENCE_PRESERVATION_COMPLETE**: ❌ 9 authoritative reports
  untracked

## Recommended action

1. Copy the 8 final authoritative reports (excluding the earlier
   `PHASE1_COMMAND_AUTHORITY_MATRIX.tsv`) into
   `docs/phases/phase_01/evidence/`
2. Update `PHASE_01_EVIDENCE_INDEX.md` to 28 total files
3. Commit: `docs(phase1): preserve command-authority and Collision Monitor evidence`
