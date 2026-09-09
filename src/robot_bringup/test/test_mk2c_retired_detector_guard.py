from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RETIRED = {
    "analyze_target.py", "capture_target.py", "target_expected_region.py",
    "target_from_range.py", "target_150_envelope.py", "analyze_ab.py",
    "analyze_raw_registered_ab.py", "registered_expected_check.py",
    "spatial_diagnostic.py", "spatial_profile.py", "fov_audit.py",
    "fov_window_summary.py",
}


def test_retired_detector_scripts_are_not_in_active_ros_workspace():
    workspace_src = ROOT.parents[1]
    discovered = {p.name for p in workspace_src.rglob("*") if p.is_file() and p.name in RETIRED}
    assert not discovered, f"retired target-detector scripts reintroduced: {sorted(discovered)}"


def test_retired_detector_path_is_not_a_runtime_hook():
    workspace_src = ROOT.parents[1]
    hooks = []
    for pattern in ("*.launch.py", "CMakeLists.txt", "package.xml", "setup.py"):
        hooks.extend(workspace_src.rglob(pattern))
    for path in hooks:
        text = path.read_text(errors="ignore")
        assert "P5_3F5" not in text, f"retired artifact path in runtime hook: {path}"
