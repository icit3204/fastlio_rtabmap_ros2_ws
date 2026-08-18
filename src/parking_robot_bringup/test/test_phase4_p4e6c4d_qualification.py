from parking_robot_bringup.phase4_p4e6c_c4d import (
    RecoveryInjectionJournal, TerminalDrain, classify_c4c_attempt1,
    duration_aware_minimum_publications,
)


def test_stepwise_recovery_requires_ack_and_preserves_canonical_zero():
    journal = RecoveryInjectionJournal()
    journal.arm(canonical_count=0, goal_uuid="u")
    for step in range(1, 7):
        journal.append({"active_goal_uuid": "u", "canonical_recovery_count": 0,
                        "actual_shadow_output_count": step})
    journal.require_sequence()
    assert journal.last_step == 6


def test_stepwise_recovery_rejects_skips_and_canonical_mutation():
    journal = RecoveryInjectionJournal()
    journal.arm(canonical_count=0, goal_uuid="u")
    try:
        journal.append({"active_goal_uuid": "u", "canonical_recovery_count": 0,
                        "actual_shadow_output_count": 2})
    except RuntimeError as exc:
        assert str(exc) == "RECOVERY_STEP_SKIPPED"
    else:
        raise AssertionError("skipped recovery step accepted")


def test_duration_aware_clamp_lower_bound():
    assert duration_aware_minimum_publications(0.5) == 5
    assert duration_aware_minimum_publications(2.0) == 18


def test_terminal_drain_blocks_cleanup_until_tail_and_physical_zero():
    drain = TerminalDrain("RECOVERY_EXHAUSTED_NO_PROGRESS")
    for event in ("RECOVERY_EXHAUSTED_NO_PROGRESS", "BLOCK_CANCEL_ACK_ACCEPTED",
                  "CANCELING_3", "CANCELED_5", "BLOCKED"):
        drain.observe(event)
    drain.clamp_joined = True
    assert not drain.complete()
    drain.observe("APPLIED_ZERO_PERSISTENT")
    drain.physical_closed = True
    assert drain.complete()


def test_c4c_attempt1_product_branch_is_not_case_qualified():
    result = classify_c4c_attempt1(clamp_journal=False, recovery_steps={6},
                                   status3=False, status5=False, blocked=False,
                                   physical_post_terminal=False)
    assert result == {
        "product_branch": "RECOVERY_EXHAUSTED_NO_PROGRESS_OBSERVED",
        "case_qualified": False,
        "classification": "CASE_NOT_QUALIFIED",
    }
