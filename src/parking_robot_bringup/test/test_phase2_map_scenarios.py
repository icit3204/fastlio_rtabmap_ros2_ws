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

    sample_count = 31
    for i in range(sample_count):
        t = i / (sample_count - 1)
        pose = {
            "x": float(start["x"]) + (float(goal["x"]) - float(start["x"])) * t,
            "y": float(start["y"]) + (float(goal["y"]) - float(start["y"])) * t,
        }
        assert_world_pose_is_free(pose, meta, width, height, data)
