from types import SimpleNamespace

from robot_visualization.navigation_visualization_helper import indexed_goal, nearest_index


def test_nearest_sampled_pose():
    assert nearest_index([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], (1.2, 0.1)) == 1


def test_nearest_empty_or_nonfinite_is_safe():
    assert nearest_index([], (0.0, 0.0)) is None
    assert nearest_index([(float('nan'), 0.0)], (0.0, 0.0)) is None


def test_goal_index_is_bounded():
    poses = [SimpleNamespace(name='a'), SimpleNamespace(name='b')]
    assert indexed_goal(poses, 1).name == 'b'
    assert indexed_goal(poses, -1) is None
    assert indexed_goal(poses, 2) is None


def test_visualization_package_has_no_motion_authority():
    from pathlib import Path

    root = Path(__file__).parents[1] / 'robot_visualization'
    # Offline fixtures may publish approved observational demo data. This
    # assertion is specifically the production helper's safety boundary.
    source = (root / 'navigation_visualization_helper.py').read_text(encoding='utf-8')
    for forbidden in ('cmd_vel', 'vehicle_cmd_safe', 'wheelchair_control_command', 'NavigateToPose', 'ActionClient'):
        assert forbidden not in source
