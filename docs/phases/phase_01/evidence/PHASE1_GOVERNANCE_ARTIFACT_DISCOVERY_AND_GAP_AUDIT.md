# Phase 1 Governance Artifact Discovery and Gap Audit

Date: 2026-07-31
Machine: Jetson 2
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Executive conclusion

**`PHASE1_GOVERNANCE_ARTIFACT_AUDIT_PASS`**

All three V3.1 authority governance artifacts (module registry, calibration
manifest, clock/timestamp policy) are **NOT_FOUND** as dedicated tracked
artifacts. The authority's architecture comparison table (Section 21) contains
module classifications with `CORE_NOW`/`CORE_LATER_GATE` etc. statuses, and
Section 10A defines the clock policy content, but no separate tracked artifact
was created for any of the three. This audit passes because all candidates and
gaps are conclusively identified — the artifacts need to be CREATED, not
discovered.

---

## 1. Authority requirements extracted

### Module registry (V3.1 §3.5, §56.3, §56.11)

Required fields per authority:
- module
- owner
- status (`CORE_NOW`, `CORE_LATER_GATE`, `OPTIONAL_EVALUATE`, `HARDWARE_FUTURE`, `LEGACY_BASELINE`, `SITE_PROFILE`, `PRODUCTIZATION_LATER`)
- evidence
- dependency
- activation gate
- acceptance criterion

Status: `CORE_NOW` per authority comparison table (line 2145 reference).

### Calibration manifest (V3.1 §20C "Calibration manifest", §56.12)

Required fields per authority:
- `body -> base_footprint`
- `base_footprint/base_link -> livox_frame`
- camera intrinsics and extrinsics
- optional YDLIDAR and ultrasonic extrinsics
- robot footprint
- wheelbase/track and kinematic constants
- calibration method, date and operator
- source files, version and checksum
- robot serial/model applicability

Status: `CORE_NOW` per authority comparison table (line 2146).

### Clock/timestamp policy (V3.1 §10A, §56.13)

Required content per authority:
- ROS time versus system time
- `use_sim_time` policy
- MID-360 LiDAR and IMU timestamp source
- stereo-camera timestamp source
- Jetson system-clock source
- measured camera-to-LiDAR and sensor-to-host offsets
- maximum acceptable timestamp skew per synchronized pipeline
- bag-playback clock behavior
- steady/monotonic receipt time for deadman/watchdog decisions
- live mode rules (real sensor stamps, disciplined host clock)
- bag/simulation mode rules (use `/clock` consistently)
- future multi-computer NTP/PTP policy (reserved)

Status: `CORE_NOW` per authority comparison table (line 2145).

---

## 2. Search results

### Module registry: NOT_FOUND

| Search method | Result |
|---|---|
| Filename search (`*registry*`, `*module*`, `*requirement*matrix*`, `*ownership*`) | 0 matches in tracked files |
| Content search (`module registry`, `CORE_NOW`, `activation gate`, `acceptance criterion`) | Found only in Phase 1 authority-compliance audit (flags the gap) |
| Authority Section 21 comparison table | Contains 36 module rows with statuses — but embedded in authority, not a separate tracked artifact |
| Phase 0 documentation claim | Does NOT claim completion of a module registry |
| Phase 1 documentation | Flags it as `REQUIRES_SOURCE_OR_CONFIG_CHANGE` |
| External phase0_reports/ | Not searched here (Phase 0 evidence scope only) |

**Classification**: `REQUIREMENT_ONLY_NO_ARTIFACT`

### Calibration manifest: NOT_FOUND

| Search method | Result |
|---|---|
| Filename search (`*calibration*`, `*calib*`, `*robot_model*`, `*footprint*`) | Only RTAB-Map third-party calibrator files (unrelated) |
| Content search (`calibration manifest`, `body.*base_footprint.*offset`, `livox_frame.*extrinsic`, `wheelchair.*footprint`, `wheelbase`) | Found only in Phase 1 authority-compliance audit (flags the gap) |
| Scattered TF values in launch files | `body→base_footprint` z offset: `-1.5` in `record_bag_infra.launch.py`, `0` in `fast_lio2.launch.py` — CONTRADICTORY runtime params, not a formal manifest |
| Phase 0 documentation claim | Does NOT claim completion of a calibration manifest |
| Phase 1 documentation | Flags it as `REQUIRES_SOURCE_OR_CONFIG_CHANGE` |

**Classification**: `REQUIREMENT_ONLY_NO_ARTIFACT`

**Critical z-offset conflict**: Two launch files define different `body→base_footprint` transforms:
- `record_bag_infra.launch.py`: `z = -1.5` (base_footprint ABOVE body)
- `fast_lio2.launch.py`: `z = 0` (base_footprint coincident with body)

This contradiction reinforces the need for a formal calibration manifest.

### Clock/timestamp policy: NOT_FOUND

| Search method | Result |
|---|---|
| Filename search (`*clock*`, `*timestamp*`, `*time*`) | 0 matches in docs/ or root |
| Content search (`clock policy`, `timestamp policy`, `steady clock`, `monotonic`, `sensor skew`, `NTP`, `PTP`) | Found only in Phase 1 authority-compliance audit (flags the gap) |
| `use_sim_time` parameters in launch files | Present in all bringup launches — but these are runtime configs, not a formal policy document |
| Authority Section 10A | DEFINES the policy content — but no separate tracked artifact was created |
| Phase 0 documentation claim | Does NOT claim completion of a clock/timestamp policy |
| Phase 1 documentation | Flags it as `REQUIRES_SOURCE_OR_CONFIG_CHANGE` |

**Classification**: `REQUIREMENT_ONLY_NO_ARTIFACT`

---

## 3. Phase 0 documentation-claim check

The Phase 0 document (`docs/phases/phase_00/PHASE_00_PRESERVATION_AND_TRANSFER.md`) lists 8 completion criteria:
- PASS — project copied
- PASS — critical files verified (73/73)
- PASS — backup created
- PASS — secrets excluded
- PASS — local Git baseline created
- PASS — GitHub branch pushed
- PASS — baseline tag pushed
- PASS — semantic work kept separate

**None of the three governance artifacts are claimed as Phase 0 deliverables** in the Phase 0 documentation.

However, the V3.1 authority's Phase 0 pass criteria (lines 2359-2361) explicitly require:
- "complete module registry exists"
- "calibration manifest exists and unknown fields are explicit"
- "clock/timestamp policy is documented"

This creates a **gap**: the V3.1 authority was written after Phase 0 completed but Phase 0 was not re-executed to fulfill these additional requirements. The authority's Phase 0 pass criteria are aspirational for what Phase 0 SHOULD have produced, not what it actually produced.

---

## 4. Artifact assessment summary

| Artifact | Classification | Tracked | Required by | Phase 0 claim | Phase 1 status | Blocks Phase 1? |
|---|---|---|---|---|---|---|
| Module registry | NOT_FOUND | No | V3.1 §56.3, §56.11 | Not claimed | NOT_EVIDENCED | No — requires creation, not discovery |
| Calibration manifest | NOT_FOUND | No | V3.1 §56.12 | Not claimed | NOT_EVIDENCED | No — requires creation |
| Clock/timestamp policy | NOT_FOUND | No | V3.1 §56.13 | Not claimed | NOT_EVIDENCED | No — requires creation |

---

## 5. Architecture comparison table as partial module registry

The V3.1 authority Section 21 (lines 2108-2145) contains a table with 36 rows mapping "Existing architecture" → "Selected revised architecture" → "Status". This table assigns `CORE_NOW`, `CORE_LATER_GATE`, `OPTIONAL_EVALUATE`, `HARDWARE_FUTURE`, `LEGACY_BASELINE`, and `PRODUCTIZATION_LATER` statuses to modules. However:

- **Missing fields**: No owner, no evidence, no activation gate, no acceptance criterion
- **Not tracked separately**: Embedded in the authority document, not in a dedicated tracked file
- **Not independently versioned**: Changes require authority revision

**Classification**: PARTIAL_EXISTING_ARTIFACT (embedded in authority)

---

## 6. Scattered parameters as partial calibration evidence

TF transforms and wheel parameters exist in launch files but are NOT a formal calibration manifest:

| Parameter | Source | Value |
|---|---|---|
| `body → base_footprint` z | `record_bag_infra.launch.py` | `-1.5` |
| `body → base_footprint` z | `fast_lio2.launch.py` | `0` |
| `can_wheel_half_track_mm` | `wheelchair_controller_node.cpp` | `300.0` (default) |
| `can_min_turn_radius_mm` | `wheelchair_controller_node.cpp` | `1000.0` (default) |
| `can_straight_radius_threshold_mm` | `wheelchair_controller_node.cpp` | `10000.0` (default) |

All values are unverified defaults or contradictory between launch files. No measurement source, date, operator, or version is recorded.

---

## 7. Recommended dispositions

| Artifact | Recommendation | Rationale |
|---|---|---|
| Module registry | **C. CREATE_NEW_TRACKED_ARTIFACT** | No existing artifact. Authority Section 21 table provides statuses for 36 modules — use as starting point. Add owner, evidence, activation gate, acceptance criterion fields. |
| Calibration manifest | **C. CREATE_NEW_TRACKED_ARTIFACT** with **E. BLOCKED_PENDING_PHYSICAL_MEASUREMENT** for specific fields | No existing artifact. Scattered launch parameters are contradictory. Create manifest with explicit `UNKNOWN` fields where measurement is deferred. The z-offset conflict must be resolved. |
| Clock/timestamp policy | **C. CREATE_NEW_TRACKED_ARTIFACT** | No existing artifact. Authority §10A defines complete policy content. Create a tracked `clock_sync_policy.yaml` or `clock_timestamp_policy.md` directly from §10A text. |

### Detailed module registry recommendation

Create: `docs/phases/phase_01/PHASE_01_MODULE_REGISTRY.md`

Using 36 modules from authority Section 21 plus additional modules from the package inventory. Required columns: Module, Owner, Status, Evidence, Dependency, Activation Gate, Acceptance Criterion.

### Detailed calibration manifest recommendation

Create: `docs/phases/phase_01/PHASE_01_CALIBRATION_MANIFEST.yaml`

Include ALL required fields from authority §20C. Mark unknown values as `UNKNOWN` with a note explaining why (e.g., "requires physical measurement during Phase 5"). Resolve the z-offset contradiction by identifying which launch file is authoritative.

### Detailed clock/timestamp policy recommendation

Create: `docs/phases/phase_01/PHASE_01_CLOCK_TIMESTAMP_POLICY.md`

Derive content directly from authority §10A. Include live/bag/watchdog sections. Document current `use_sim_time` parameter locations for traceability.

---

## 8. Phase 1 closure impact

All three artifacts are `CORE_NOW` per the authority but were never created. They do NOT block the native-build verification milestone (which is complete), but they DO block full Phase 1 authority closure. The Phase 1 documentation correctly flags them as outstanding.

---

## 9. Integrity

| Check | Status |
|---|---|
| Git clean | ✅ |
| HEAD `c27961d` | ✅ |
| Tags unchanged | ✅ |
| No tracked/untracked content modified | ✅ |
| No ROS/build/hardware activity | ✅ |
| No commit/tag/push | ✅ |

---

PHASE1_GOVERNANCE_ARTIFACT_AUDIT_PASS
