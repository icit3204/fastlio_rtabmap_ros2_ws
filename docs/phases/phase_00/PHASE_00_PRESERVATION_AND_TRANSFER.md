# Phase 0 — Preservation and Transfer

## 1. Phase status

COMPLETED

## 2. Purpose

Phase 0 preserved the original working project, verified the Jetson 2 copy, created a backup, and established a safe Git and GitHub baseline.

## 3. Machines

- Jetson 1: original labmate reference
- Jetson 2: verified development machine

## 4. Workspace

`/home/dog/fastlio_rtabmap_ros2_ws`

## 5. Transfer verification

- Critical files checked: 73
- Matching: 73
- Missing: 0
- Mismatched: 0
- Unreadable: 0

## 6. Jetson 2 preservation

The user created an untouched backup of the Jetson 2 workspace.

## 7. Git baseline

- Branch: `main`
- Baseline commit: `ae7517e`
- Baseline tag: `phase0_verified_baseline`
- Tracked files: 569
- The working tree was clean.
- No project source was changed during baseline preparation.

## 8. GitHub repository

Repository: <https://github.com/icit3204/fastlio_rtabmap_ros2_ws>

The `main` branch and the `phase0_verified_baseline` tag were successfully pushed to GitHub.

## 9. Excluded assets

The following assets remain on Jetson 2 and in backup storage. They are intentionally not stored in normal Git:

- `build/`
- `install/`
- `log/`
- Maps and RTAB-Map databases
- Point clouds
- ROS bags
- Model weights and datasets
- Backup project copies
- Large third-party source
- YDLIDAR repositories
- Local secret settings

## 10. Secret protection

- `plan_nav/config/settings.json` is ignored.
- `plan_nav/config/settings.example.json` is tracked and contains a placeholder.
- No real secret was committed.

## 11. Semantic work

`semantic_grid_tools` and the recovered semantic work remain separate. They were not merged during Phase 0.

## 12. Jetson 1 decision

Jetson 1 is no longer required for normal development because all critical files matched and Jetson 2 was backed up.

## 13. Evidence

Phase 0 reports are stored under `/home/dog/phase0_reports/`. Important external reports include:

- Final Git audit: `git_baseline/GIT_BASELINE_FINAL_AUDIT.md`
- Git audit: `git_baseline/GIT_BASELINE_AUDIT.md`
- Git result report: `git_baseline/GIT_BASELINE_RESULT.md`

The important transfer comparison summary is `PHASE0_VERIFIED_BASELINE.md` in the workspace. Its preserved evidence includes `Codex_create_JETSON2_PHASE0_BASELINE_INSPECTION.md` and `Codex_create_JETSON2_CRITICAL_FILES.sha256`.

## 14. Phase 0 completion criteria

- PASS — project copied
- PASS — critical files verified
- PASS — backup created
- PASS — secrets excluded
- PASS — local Git baseline created
- PASS — GitHub branch pushed
- PASS — baseline tag pushed
- PASS — semantic work kept separate

## 15. Information carried into Phase 1

- Jetson 2 is the development machine.
- ROS 2 Humble
- Ubuntu 22.04
- L4T R36.5
- The workspace path is unchanged.
- The baseline commit and tag are available.
- Phase 1 will inspect and perform a clean native build.
- Phase 1 has not started yet.

## 16. Future phase overview

- Phase 0 — COMPLETED
- Phase 1 — NOT STARTED
- Phase 2 — NOT STARTED
- Phase 3 — NOT STARTED
- Phase 4 — NOT STARTED
- Phase 5 — NOT STARTED
- Phase 6 — NOT STARTED
- Phase 7 — NOT STARTED
- Phase 8 — NOT STARTED
- Phase 9 — NOT STARTED
- Phase 10 — NOT STARTED
- Phase 11 — NOT STARTED
- Phase 12 — NOT STARTED
- Phase 13 — NOT STARTED
- Phase 14 — NOT STARTED
