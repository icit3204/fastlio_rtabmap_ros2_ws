from parking_robot_bringup.phase4_p4e6c_c4d import RecoveryInjectionJournal


def test_relay_output_does_not_self_ack_next_step():
    journal = RecoveryInjectionJournal()
    journal.arm(canonical_count=0, goal_uuid="u")
    journal.append({"active_goal_uuid": "u", "canonical_recovery_count": 0,
                    "actual_shadow_output_count": 1})
    # Repeated relay output is allowed, but the journal cannot claim step 2
    # until a distinct source acknowledgement is recorded.
    journal.append({"active_goal_uuid": "u", "canonical_recovery_count": 0,
                    "actual_shadow_output_count": 1})
    assert journal.last_step == 1


def test_source_acknowledged_steps_are_serial():
    journal = RecoveryInjectionJournal()
    journal.arm(canonical_count=0, goal_uuid="u")
    for step in range(1, 7):
        journal.append({"active_goal_uuid": "u", "canonical_recovery_count": 0,
                        "actual_shadow_output_count": step,
                        "mission_manager_ack": True})
    journal.require_sequence()
    assert sorted({r["actual_shadow_output_count"] for r in journal.rows}) == [1, 2, 3, 4, 5, 6]
