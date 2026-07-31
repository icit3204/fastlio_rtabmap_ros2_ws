from pathlib import Path
import hashlib

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

