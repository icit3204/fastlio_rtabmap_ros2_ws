# Phase 5A MK2G1C Git Staging Manifest

## Accepted and staged categories

| Category | Content |
|---|---|
| A — accepted RViz source | `src/robot_visualization/robot_visualization`, README |
| B — accepted Operator GUI source | `src/operator_gui/operator_gui` |
| C — accepted visualization configuration | `src/robot_visualization/config/canonical_navigation.rviz`; `src/robot_bringup/config/nav2_common.yaml` visualization-only change |
| D — accepted demo fixtures/launches | integrated demo fixture and launch; existing offline fixtures; GUI demo fixture/launch |
| E — accepted tests | both package test directories |
| F — accepted documentation/contracts | GUI/RViz authority contracts, runtime contracts, matrices, integrated demo contract, checkpoint document |
| G — package/launch metadata | both package manifests, setup files, resources, launch files |

## Explicitly excluded

The 20 untracked root-level historical/temporary items before the accepted GUI
files are excluded: B2AP/B2AQ/B2AR/B2AS/B2AU/B2AV/B2AW/B2AX/B2AY evidence,
`RUN.TXT`, `can_faliar.md`, `keyboard_can_control.py`, old frame `.gv`
captures, Phase-4 manifest/classification drafts, and `docs/phases/temp text.txt`.
They are preserved on disk and are not deleted.

Generated `build/`, `install/`, `log/`, Python caches/bytecode, runtime RTAB
working copies, ROS logs, and temporary desktop screenshots/evidence are also
excluded. Supervisor evidence remains under `/home/dog/phase5_reports/`.

No PlanNav file is staged. The canonical RTAB database is not staged or
modified.
