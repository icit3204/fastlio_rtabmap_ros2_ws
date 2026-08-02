from pathlib import Path
import hashlib
import math

import yaml


PKG = Path(__file__).resolve().parents[1]
MAP_YAML = PKG / "maps" / "phase2_clean_map.yaml"
MAP_PGM = PKG / "maps" / "phase2_clean_map.pgm"
SCENARIOS = PKG / "config" / "phase2_test_scenarios.yaml"


def read_pgm(path):
    with path.open("rb") as f:
        assert f.readline().strip() == b"P5"
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())
        width, height, max_value = map(int, tokens[:3])
        data = f.read()
    assert len(data) == width * height
    return width, height, max_value, data


def pixel_value(data, width, x, y):
    return data[y * width + x]


def occupancy_probability(value, negate=False):
    return (value if negate else 255 - value) / 255.0


def classify(value, meta):
    prob = occupancy_probability(value, bool(meta.get("negate", 0)))
    if prob > float(meta["occupied_thresh"]):
        return "occupied"
    if prob < float(meta["free_thresh"]):
        return "free"
    return "unknown"


def world_to_pixel(x, y, meta, height):
    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])
    px = int(math.floor((x - origin_x) / resolution))
    map_y = int(math.floor((y - origin_y) / resolution))
    return px, height - 1 - map_y


def assert_world_pose_is_free(pose, meta, width, height, data):
    px, py = world_to_pixel(float(pose["x"]), float(pose["y"]), meta, height)
    assert 0 <= px < width
    assert 0 <= py < height
    assert classify(pixel_value(data, width, px, py), meta) == "free"


def assert_straight_segment_is_free(start, goal, meta, width, height, data, samples=31):
    for i in range(samples):
        t = i / (samples - 1)
        pose = {
            "x": float(start["x"]) + (float(goal["x"]) - float(start["x"])) * t,
            "y": float(start["y"]) + (float(goal["y"]) - float(start["y"])) * t,
        }
        assert_world_pose_is_free(pose, meta, width, height, data)


def test_map_yaml_resolves_to_packaged_pgm():
    meta = yaml.safe_load(MAP_YAML.read_text())
    assert meta["image"] == "phase2_clean_map.pgm"
    assert (MAP_YAML.parent / meta["image"]).resolve() == MAP_PGM.resolve()
    assert MAP_PGM.exists()


def test_map_checksum_matches_scenario_source():
    scenarios = yaml.safe_load(SCENARIOS.read_text())
    digest = hashlib.sha256(MAP_PGM.read_bytes()).hexdigest()
    assert digest == scenarios["map_source_sha256"]


def test_scenario_poses_classify_as_intended():
    meta = yaml.safe_load(MAP_YAML.read_text())
    scenarios = yaml.safe_load(SCENARIOS.read_text())
    width, height, max_value, data = read_pgm(MAP_PGM)
    assert max_value == 255
    assert width == 1744
    assert height == 2683

    for key in ("start", "goal_a", "goal_b", "goal_c"):
        pixel = scenarios[key]["pixel"]
        value = pixel_value(data, width, int(pixel["x"]), int(pixel["y"]))
        assert classify(value, meta) == "free"
        assert scenarios[key]["connected_component"] == 1
        assert float(scenarios[key]["clearance_m"]) > 5.0

    pixel = scenarios["planner_failure_goal"]["pixel"]
    value = pixel_value(data, width, int(pixel["x"]), int(pixel["y"]))
    assert classify(value, meta) == "occupied"
    assert scenarios["planner_failure_goal"]["connected_component"] == 0


def test_p2d_forward_goal_static_map_properties():
    meta = yaml.safe_load(MAP_YAML.read_text())
    scenarios = yaml.safe_load(SCENARIOS.read_text())
    width, height, _, data = read_pgm(MAP_PGM)
    scenario = scenarios["p2d_forward_goal"]

    assert list(k for k in scenarios.keys()).count("p2d_forward_goal") == 1
    start = scenario["initial_pose"]
    goal = scenario["goal"]

    assert_world_pose_is_free(start, meta, width, height, data)
    assert_world_pose_is_free(goal, meta, width, height, data)

    distance = math.hypot(float(goal["x"]) - float(start["x"]), float(goal["y"]) - float(start["y"]))
    assert abs(distance - 3.0) <= 1.0e-6
    assert float(start["yaw"]) == 0.0
    assert float(goal["yaw"]) == 0.0

    assert_straight_segment_is_free(start, goal, meta, width, height, data)


def test_p2e_scenarios_static_map_properties():
    meta = yaml.safe_load(MAP_YAML.read_text())
    scenarios = yaml.safe_load(SCENARIOS.read_text())
    width, height, _, data = read_pgm(MAP_PGM)

    assert list(scenarios.keys()).count("p2e_sequential_forward") == 1
    assert list(scenarios.keys()).count("p2e_cancel_forward") == 1

    seq = scenarios["p2e_sequential_forward"]
    cancel = scenarios["p2e_cancel_forward"]
    seq_start = seq["initial_pose"]
    cancel_start = cancel["initial_pose"]
    assert seq_start == cancel_start == scenarios["p2d_forward_goal"]["initial_pose"]

    previous = seq_start
    for goal in seq["goals"]:
        assert float(goal["yaw"]) == 0.0
        assert_world_pose_is_free(previous, meta, width, height, data)
        assert_world_pose_is_free(goal, meta, width, height, data)
        distance = math.hypot(float(goal["x"]) - float(previous["x"]), float(goal["y"]) - float(previous["y"]))
        assert abs(distance - 1.0) <= 1.0e-6
        assert abs(float(goal["y"]) - float(seq_start["y"])) <= 1.0e-9
        assert_straight_segment_is_free(previous, goal, meta, width, height, data)
        previous = goal

    assert seq["goals"][-1] == scenarios["p2d_forward_goal"]["goal"]
    assert cancel["goal"] == scenarios["p2d_forward_goal"]["goal"]
    assert float(cancel["cancel_after_acceptance_sec"]) == 3.0
    cancel_distance = math.hypot(
        float(cancel["goal"]["x"]) - float(cancel_start["x"]),
        float(cancel["goal"]["y"]) - float(cancel_start["y"]),
    )
    assert abs(cancel_distance - 3.0) <= 1.0e-6
    assert_straight_segment_is_free(cancel_start, cancel["goal"], meta, width, height, data)


def test_p2f_scenarios_static_map_properties():
    meta = yaml.safe_load(MAP_YAML.read_text())
    scenarios = yaml.safe_load(SCENARIOS.read_text())
    width, height, _, data = read_pgm(MAP_PGM)

    for name in ("p2f_planner_occupied", "p2f_controller_no_progress", "p2f_tf_loss"):
        assert list(scenarios.keys()).count(name) == 1
        assert scenarios[name]["initial_pose"] == scenarios["p2d_forward_goal"]["initial_pose"]
        assert_world_pose_is_free(scenarios[name]["initial_pose"], meta, width, height, data)

    planner_goal = scenarios["p2f_planner_occupied"]["goal"]
    px, py = world_to_pixel(float(planner_goal["x"]), float(planner_goal["y"]), meta, height)
    assert 0 <= px < width
    assert 0 <= py < height
    assert classify(pixel_value(data, width, px, py), meta) == "occupied"
    assert planner_goal == scenarios["planner_failure_goal"]["pose"]

    for name in ("p2f_controller_no_progress", "p2f_tf_loss"):
        goal = scenarios[name]["goal"]
        assert goal == scenarios["p2d_forward_goal"]["goal"]
        assert_world_pose_is_free(goal, meta, width, height, data)
        assert_straight_segment_is_free(scenarios[name]["initial_pose"], goal, meta, width, height, data)
        assert float(goal["yaw"]) == 0.0

    assert float(scenarios["p2f_controller_no_progress"]["pose_clamp_rate_hz"]) == 10.0
    assert float(scenarios["p2f_tf_loss"]["transform_stale_wait_sec"]) == 2.0
