# Phase 4 P4-B Evidence Provenance Errata

## Purpose

This errata corrects metadata provenance defects in the external P4-B resumed
fresh-source handoff evidence.  The P4-B implementation and runtime result at
commit `eb7259bf8f33448ac08db5aece7269af82062670` remain accepted.

No runtime result is changed by this document.  No P4-B scenario was rerun for
this errata.  The original external handoff files and raw evidence archives
remain preserved and unmodified.

This document is authoritative when P4-B provenance fields conflict.

## Accepted P4-B Result

- final P4-B commit:
  `eb7259bf8f33448ac08db5aece7269af82062670`
- commit message: `feat(phase4): add isolated collision monitor chain`
- accepted final status:
  `PHASE4_P4B_RESUMED_FRESH_SOURCE_COLLISION_MONITOR_CHAIN_COMPLETED_AND_PUSHED`
- repository P4-B document:
  `docs/phases/phase_04/PHASE_04_P4B_COLLISION_MONITOR_CHAIN.md`
- repository P4-B document SHA256:
  `87a21edb23ea3deeb5db2ac82c64504c8da7750a9b10d9a3b379f33caba6a6e3`

P4-B validates only fresh-source Collision Monitor filtering:

`Nav2 controller -> /cmd_vel_nav_raw -> Collision Monitor -> /cmd_vel_nav_safe -> Phase 2 fake base`

P4-B does not claim chain-level stale-source safety.

## Correct Prior Stopped-Campaign Provenance

The prior stopped P4-B campaign remains:

- status: `P4B_INSTALLED_STALE_SOURCE_FAIL_OPEN_NEEDS_REVIEW`
- classification: `INSTALLED_COLLISION_MONITOR_STALE_SOURCE_FAIL_OPEN`
- raw archive SHA256:
  `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1`
- raw manifest SHA256:
  `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea`
- `silent_safe_zero_count = 0`
- `silent_zero_latency_sec = null`

## Correct Resumed-Campaign Provenance

The current resumed P4-B campaign remains:

- final status:
  `PHASE4_P4B_RESUMED_FRESH_SOURCE_COLLISION_MONITOR_CHAIN_COMPLETED_AND_PUSHED`
- final commit:
  `eb7259bf8f33448ac08db5aece7269af82062670`
- raw archive SHA256:
  `d6b25417477b1c7c221ef7735fb6f3afb05fcba80132ab63820b05e2b0a02493`
- raw manifest SHA256:
  `d286e9b27c3236ec501b7c6cd5576c1fd083726205fe191536853e0474b1c078`
- raw archive entry count: `739`

## Metadata Defects

The external resumed handoff JSON contains an incorrect `status` field:

- recorded:
  `P4B_RESUMED_FRESH_SOURCE_PREFLIGHT_PASS`
- correct:
  `PHASE4_P4B_RESUMED_FRESH_SOURCE_COLLISION_MONITOR_CHAIN_COMPLETED_AND_PUSHED`

The external resumed handoff Markdown contains incorrect hashes in the
`Stale-Source Limitation` section.  That section says it is describing prior
stopped evidence, but it records the current resumed raw archive and manifest
hashes.  The correct prior stopped hashes are listed above.

## Provenance Audit Table

| Artifact | Claimed status | Claimed archive hash | Claimed manifest hash | Correct status/hash | Consistent | Required correction |
| --- | --- | --- | --- | --- | --- | --- |
| Repository P4-B document | Fresh-source preflight status only: `P4B_RESUMED_FRESH_SOURCE_PREFLIGHT_PASS`; final result recorded by commit | Prior stopped hash `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1` in stale-source limitation section | Prior stopped hash `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea` in stale-source limitation section | No change required; document hash remains frozen at `87a21edb23ea3deeb5db2ac82c64504c8da7750a9b10d9a3b379f33caba6a6e3` | yes | None |
| Resumed handoff Markdown | `PHASE4_P4B_RESUMED_FRESH_SOURCE_COLLISION_MONITOR_CHAIN_COMPLETED_AND_PUSHED` | Current resumed hash `d6b25417477b1c7c221ef7735fb6f3afb05fcba80132ab63820b05e2b0a02493` repeated in prior stopped section | Current resumed hash `d286e9b27c3236ec501b7c6cd5576c1fd083726205fe191536853e0474b1c078` repeated in prior stopped section | Prior stopped section must reference `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1` and `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea`; current resumed raw evidence remains `d6b25417477b1c7c221ef7735fb6f3afb05fcba80132ab63820b05e2b0a02493` and `d286e9b27c3236ec501b7c6cd5576c1fd083726205fe191536853e0474b1c078` | no | Preserve original; use this errata as correction |
| Resumed handoff JSON | `P4B_RESUMED_FRESH_SOURCE_PREFLIGHT_PASS` | Current resumed hash `d6b25417477b1c7c221ef7735fb6f3afb05fcba80132ab63820b05e2b0a02493`; nested prior stopped hash `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1` | Current resumed hash `d286e9b27c3236ec501b7c6cd5576c1fd083726205fe191536853e0474b1c078`; nested prior stopped hash `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea` | Status must be `PHASE4_P4B_RESUMED_FRESH_SOURCE_COLLISION_MONITOR_CHAIN_COMPLETED_AND_PUSHED`; hashes are consistent | no | Preserve original; use this errata as correction |
| Resumed raw manifest | Not applicable | Not applicable | SHA256 `d286e9b27c3236ec501b7c6cd5576c1fd083726205fe191536853e0474b1c078` | Resumed raw manifest hash is correct | yes | None |
| Prior stopped handoff Markdown | `P4B_INSTALLED_STALE_SOURCE_FAIL_OPEN_NEEDS_REVIEW` | Prior stopped hash `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1` | Prior stopped hash `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea` | Prior stopped status and hashes are correct | yes | None |
| Prior stopped handoff JSON | `P4B_INSTALLED_STALE_SOURCE_FAIL_OPEN_NEEDS_REVIEW` | Prior stopped hash `eb97760082758b51a027470c28014086be8325e2aeb834d368cdbc665b7d76a1` | Prior stopped hash `dabea5d652cc1184995f6a84bc77becf4ad3e6c70459b5c1e3fd54e77f3f89ea` | Prior stopped status and hashes are correct | yes | None |
| P4-A.2 stale observation decision | Records stale-source fail-open classification and observed silent metrics | Not applicable | Not applicable | P4-A.2 remains authoritative for `/system/collision_monitor_valid` and stale-source safety boundary | yes | None |
| Git commit records | P4-B commit message `feat(phase4): add isolated collision monitor chain` | Not applicable | Not applicable | Final P4-B implementation commit is `eb7259bf8f33448ac08db5aece7269af82062670` | yes | None |

## Stale-Source Safety Boundary

The stale-source limitation remains assigned to P4-C.  P4-C must enforce stale
observation safety using `/system/collision_monitor_valid` and the Generic
Command Safety Gate.  P4-B does not implement `/system/collision_monitor_valid`,
`/vehicle_cmd_safe`, the Generic Command Safety Gate, or the wheelchair
adapter.

## Change Control

The original P4-B external handoff artifacts remain preserved for audit
continuity.  They must not be silently rewritten.  When provenance fields
conflict between those preserved artifacts and this repository errata, this
errata is the authoritative correction.
