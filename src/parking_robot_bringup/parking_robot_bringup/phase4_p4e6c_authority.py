"""Prospective P4-E.6C authority shape; values are sealed after build."""
from __future__ import annotations


def prospective_case_authority(case: str, identities: dict[str, str]) -> dict:
    case = str(case).upper()
    if case not in {"C-P01", "C-P02"}:
        raise RuntimeError("P4E6C_CASE_DENIED")
    required = {
        "progress_controller", "progress_runner", "progress_evidence", "progress_witness",
        "block_terminal_adjudicator", "physical_adjudicator", "global_zero_cleanup",
        "freeze", "matrix_launch", "fake_base", "pose_clamp_implementation",
        "progress_witness", "progress_live_driver",
        "mission_progress_source", "mission_manager_node",
        "mission_manager_progress_checker_source", "mission_manager_progress_checker_installed",
        "recovery_feedback_relay_source", "recovery_feedback_relay_installed",
        "c_p01_qualification_launch", "c_p02_qualification_launch_remap",
        "pose_clamp_stimulator",
    }
    missing = sorted(required - set(identities))
    if missing:
        raise RuntimeError("P4E6C_AUTHORITY_IDENTITY_MISSING:" + ",".join(missing))
    reason = ("CONTROLLER_NO_PROGRESS" if case == "C-P01"
              else "RECOVERY_EXHAUSTED_NO_PROGRESS")
    return {"case": case, "reason": reason,
            "required_pinned_identity": dict(sorted(identities.items()))}
