from parking_robot_bringup.phase4_p4e6c_pose_clamp import clamp_warmup_status


def test_c4e_one_publication_waits_for_warmup():
    assert [clamp_warmup_status(n) for n in (1, 2, 3, 4)] == ["WAIT"] * 4
    assert clamp_warmup_status(5) == "CLAMP_WARMUP_READY"


def test_c4f_warmup_authority_is_publication_count_only():
    assert clamp_warmup_status(1) == "WAIT"
    assert clamp_warmup_status(5) == "CLAMP_WARMUP_READY"


def test_no_recovery_effect_before_warmup():
    warmup = [clamp_warmup_status(n) for n in range(1, 5)]
    recovery_arm_count = sum(value == "CLAMP_WARMUP_READY" for value in warmup)
    assert recovery_arm_count == 0
