from parking_robot_bringup.phase4_p4e1b_clear_runner import adjudicate_terminal_stop


RESULT = 1_000_000_000


def adjudicate(**overrides):
    layers = overrides.pop("layers", {
        "raw": [(100, True), (900_000_000, False)],
        "safe": [(110, True), (950_000_000, False)],
        "gate": [(120, True), (1_020_000_000, False)],
        "mock": [(130, True), (1_040_000_000, False)],
        "fake": [(140, True), (1_030_000_000, False)],
    })
    return adjudicate_terminal_stop(
        layers, overrides.pop("result_ns", RESULT), overrides.pop("succeeded", True),
        overrides.pop("observation_end_ns", 3_100_000_000),
        overrides.pop("post_result_translation_m", 0.001), **overrides)


def test_raw_and_safe_zero_before_succeeded_result_pass():
    result = adjudicate()
    assert result["pass"]
    assert result["layers"]["raw"]["result_ordering"] == "ZERO_BEFORE_RESULT"
    assert result["layers"]["safe"]["result_ordering"] == "ZERO_BEFORE_RESULT"


def test_gate_mock_and_fake_zero_after_result_pass():
    result = adjudicate()
    assert result["pass"]
    assert all(result["layers"][name]["result_ordering"] == "ZERO_AFTER_RESULT"
               for name in ("gate", "mock", "fake"))


def test_all_layers_zero_before_result_pass():
    layers = {name: [(100, True), (800_000_000 + index, False)]
              for index, name in enumerate(("raw", "safe", "gate", "mock", "fake"))}
    assert adjudicate(layers=layers, observation_end_ns=2_900_000_000)["pass"]


def test_zero_then_later_nonzero_without_final_zero_is_rejected():
    layers = {name: [(100, True), (900_000_000, False)] for name in ("raw", "safe", "gate", "mock", "fake")}
    layers["raw"].append((1_100_000_000, True))
    result = adjudicate(layers=layers, observation_end_ns=3_200_000_000)
    assert not result["pass"]
    assert result["layers"]["raw"]["terminal_zero_ns"] is None


def test_temporary_stop_zero_then_resume_then_terminal_zero_passes():
    layers = {
        name: [(100, True), (500_000_000, False), (700_000_000, True),
               (900_000_000 + index, False)]
        for index, name in enumerate(("raw", "safe", "gate", "mock", "fake"))
    }
    result = adjudicate(layers=layers, observation_end_ns=3_100_000_000)
    assert result["pass"]
    assert all(result["layers"][name]["terminal_zero_ns"] >= 900_000_000
               for name in layers)


def test_downstream_zero_later_than_bound_is_rejected():
    result = adjudicate(layers={
        "raw": [(100, True), (900_000_000, False)], "safe": [(110, True), (950_000_000, False)],
        "gate": [(120, True), (1_100_000_001, False)], "mock": [(130, True), (1_120_000_000, False)],
        "fake": [(140, True), (1_110_000_000, False)]}, observation_end_ns=3_300_000_000)
    assert not result["pass"]
    assert result["layers"]["gate"]["downstream_stop_latency_ns"] == 100_000_001


def test_stationary_duration_below_two_seconds_is_rejected():
    result = adjudicate(observation_end_ns=3_039_999_999)
    assert not result["pass"]
    assert not result["stationary_window_pass"]


def test_translation_above_limit_is_rejected():
    result = adjudicate(post_result_translation_m=0.0200001)
    assert not result["pass"]
    assert not result["post_terminal_translation_pass"]


def test_non_succeeded_result_is_rejected():
    result = adjudicate(succeeded=False)
    assert not result["pass"]
    assert "not SUCCEEDED" in result["final_reason"]


def test_missing_terminal_zero_is_rejected():
    layers = {name: [(100, True), (900_000_000, False)] for name in ("raw", "safe", "gate", "mock", "fake")}
    layers["safe"] = [(100, True)]
    result = adjudicate(layers=layers, observation_end_ns=3_100_000_000)
    assert not result["pass"]
    assert result["layers"]["safe"]["terminal_zero_ns"] is None


def test_exact_accepted_p4e1b_timestamp_sequence_passes():
    result = adjudicate_terminal_stop({
        "raw": [(300_334_900_000_000, True), (300_334_951_559_983, False)],
        "safe": [(300_334_910_000_000, True), (300_334_977_036_351, False)],
        "gate": [(300_334_920_000_000, True), (300_335_020_175_351, False)],
        "fake": [(300_334_930_000_000, True), (300_335_032_590_721, False)],
        "mock": [(300_334_940_000_000, True), (300_335_045_367_963, False)],
    }, 300_334_985_061_521, True, 300_337_441_474_581, 0.0014299196067623446)
    assert result["pass"]
    assert result["stationary_duration_ns"] == 2_396_106_618
    assert result["layers"]["gate"]["downstream_stop_latency_ns"] == 35_113_830
    assert result["layers"]["fake"]["downstream_stop_latency_ns"] == 12_415_370
    assert result["layers"]["mock"]["downstream_stop_latency_ns"] == 25_192_612
