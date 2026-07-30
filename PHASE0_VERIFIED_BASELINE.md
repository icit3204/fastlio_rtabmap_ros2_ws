# Phase 0 — Verified Baseline

**Workspace:** `/home/dog/fastlio_rtabmap_ros2_ws`  
**Date:** 2026-07-30  
**Machine:** Jetson 2 (dog-desktop) — NVIDIA Jetson Orin NX  
**Phase:** Phase 0 — Preservation and Baseline Verification

---

## Environment

| Item | Value |
|------|-------|
| **ROS 2 Distribution** | Humble |
| **Ubuntu** | 22.04 |
| **L4T** | R36.5 |
| **Kernel** | 5.15.185-tegra |
| **Architecture** | aarch64 |
| **Python** | 3.10.12 |

---

## Verification Summary

### Jetson 1 ↔ Jetson 2 Critical File Comparison

| Result | Count |
|--------|-------|
| **Matching** | 73 |
| **Missing** | 0 |
| **Mismatched** | 0 |

All critical files verified — the Jetson 2 workspace is a faithful copy.

### Backup Status

- ✅ Jetson 2 workspace backup completed
- ✅ Phase 0 baseline inspection completed
- ✅ GitHub preparation reports generated
- ✅ Local Git repository initialized with baseline commit

---

## Scope Boundaries

| Item | Status |
|------|--------|
| `semantic_grid_tools` merged | **No** — not present in workspace |
| Maps / databases in Git | **No** — excluded via `.gitignore` |
| Point clouds in Git | **No** — excluded via `.gitignore` |
| Model weights in Git | **No** — excluded via `.gitignore` |
| Build artifacts in Git | **No** — excluded via `.gitignore` |
| Real API keys in Git | **No** — `settings.json` is gitignored; `settings.example.json` is tracked |
| Third-party sources in Git | **No** — excluded; versions recorded in `THIRD_PARTY_VERSIONS.md` |
| Backups preserved | **Yes** — `plan_nav07202900/`, duplicate DBs, backup maps all retained on disk |
| Nested Git repos touched | **No** — all three nested `.git/` directories untouched |

---

## Git Repository Purpose

This Git repository tracks the **project-specific source code, launch files, configuration, scripts, and documentation**. It explicitly excludes:

- Large binary assets (databases, point clouds, map images, model weights)
- Generated build/install/log artifacts
- Third-party source trees (versioned separately)
- Local secrets and machine-specific settings
- Backup copies and duplicate data

Reproducibility is maintained through `THIRD_PARTY_VERSIONS.md` which records exact upstream commits for all excluded dependencies.

---

## Local Settings

The real `plan_nav/config/settings.json` (containing the Roboflow API key) is **excluded from Git** via `.gitignore`. A sanitized example file is tracked at `plan_nav/config/settings.example.json` with the key replaced by `<YOUR_ROBOFLOW_API_KEY>`.

To restore local settings after cloning:
```bash
cp plan_nav/config/settings.example.json plan_nav/config/settings.json
# Then edit settings.json and insert your real Roboflow API key
```

---

*Generated 2026-07-30. No project source code was changed. No maps, databases, or backups were deleted.*
