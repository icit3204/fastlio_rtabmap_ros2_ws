"""Focused tests for the observation-only PlanNav edge rendering contract."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PyQt5.QtWidgets import QApplication, QGraphicsLineItem, QGraphicsPolygonItem, QGraphicsTextItem

from ui.map_view import MapView, edge_arrow_points, edge_label_position


def test_arrowhead_points_follow_persisted_from_to_direction():
    arrow = edge_arrow_points((0.0, 0.0), (100.0, 0.0), fraction=0.5)
    assert arrow is not None
    assert arrow[0] == (50.0, 0.0)
    assert arrow[1][0] < arrow[0][0]
    assert arrow[2][0] < arrow[0][0]

    reverse = edge_arrow_points((100.0, 0.0), (0.0, 0.0), fraction=0.5)
    assert reverse is not None
    assert reverse[0] == (50.0, 0.0)
    assert reverse[1][0] > reverse[0][0]
    assert reverse[2][0] > reverse[0][0]


def test_arrowhead_geometry_handles_diagonal_and_zero_length_edges():
    arrow = edge_arrow_points((0.0, 0.0), (10.0, 10.0))
    assert arrow is not None
    assert arrow[0][0] == pytest.approx(arrow[0][1])
    assert edge_arrow_points((1.0, 1.0), (1.0, 1.0)) is None


def test_paired_label_offsets_are_opposite_and_deterministic():
    first = edge_label_position((0.0, 0.0), (100.0, 0.0), 10.0)
    second = edge_label_position((0.0, 0.0), (100.0, 0.0), -10.0)
    assert first == (50.0, 10.0)
    assert second == (50.0, -10.0)


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


def test_draw_edges_keeps_paired_records_and_persisted_ids(qt_app):
    view = MapView()
    view.map_meta = object()
    view._w2p = lambda x, y: (float(x), float(y))
    waypoints = [
        {"id": 1, "x": 0.0, "y": 0.0},
        {"id": 2, "x": 100.0, "y": 0.0},
    ]
    edges = [
        {"edge_id": "edge-000101", "from_id": 1, "to_id": 2,
         "length": 1.0, "direction": "uni", "traj_file": ""},
        {"edge_id": "edge-000102", "from_id": 2, "to_id": 1,
         "length": 1.0, "direction": "uni", "traj_file": ""},
    ]
    before = deepcopy(edges)

    view.draw_edges(edges, waypoints)

    assert edges == before
    assert len([item for item in view._edge_items if isinstance(item, QGraphicsLineItem)]) == 2
    assert len([item for item in view._edge_items if isinstance(item, QGraphicsPolygonItem)]) == 2
    labels = [item.toPlainText() for item in view._edge_items
              if isinstance(item, QGraphicsTextItem)]
    assert "edge-000101" in labels[0]
    assert "edge-000102" in labels[1]
    lines = [item for item in view._edge_items if isinstance(item, QGraphicsLineItem)]
    assert {round(item.line().center().y(), 6) for item in lines} == {-10.0, 10.0}
    assert all(item.zValue() == 1.0 for item in lines)


def test_bidirectional_record_has_two_visual_direction_arrows(qt_app):
    view = MapView()
    view.map_meta = object()
    view._w2p = lambda x, y: (float(x), float(y))
    view.draw_edges(
        [{"edge_id": "edge-000201", "from_id": 1, "to_id": 2,
          "length": 1.0, "direction": "bi", "traj_file": ""}],
        [{"id": 1, "x": 0.0, "y": 0.0}, {"id": 2, "x": 100.0, "y": 0.0}],
    )
    assert len([item for item in view._edge_items
                if isinstance(item, QGraphicsPolygonItem)]) == 2


def test_selected_route_overlay_remains_a_separate_higher_layer(qt_app):
    view = MapView()
    view.map_meta = object()
    view._w2p = lambda x, y: (float(x), float(y))
    waypoints = [{"id": 1, "x": 0.0, "y": 0.0},
                 {"id": 2, "x": 100.0, "y": 0.0}]
    edge = {"edge_id": "edge-000301", "from_id": 1, "to_id": 2,
            "length": 1.0, "direction": "uni", "traj_file": ""}
    view.draw_edges([edge], waypoints)
    view.draw_planned_path(waypoints)
    assert len(view._planned_path_items) == 1
    assert view._planned_path_items[0].zValue() == 2.0
    assert any(item.zValue() == 3.0 for item in view._edge_items)
