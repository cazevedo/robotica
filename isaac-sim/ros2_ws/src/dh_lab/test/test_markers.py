"""Tests for visualization/markers.py. Needs the stock
visualization_msgs/geometry_msgs/std_msgs message packages (run inside
the container, same as test_dh_lab_node.py) but not rclpy itself --
these are plain message-construction functions.
"""
import numpy as np
import pytest
from visualization_msgs.msg import Marker

from dh_lab.kinematics.dh_table import DHTable
from dh_lab.kinematics.transforms import forward_kinematics
from dh_lab.visualization.markers import (
    axis_triad_marker,
    build_dh_frame_markers,
    build_tool_comparison_markers,
    clear_namespace_marker,
    label_marker,
    polyline_marker,
)


class TestAxisTriadMarker:
    def test_identity_pose_has_axis_aligned_tips(self):
        m = axis_triad_marker(np.eye(4), "base", "ns", 0, length=0.1)
        assert m.type == Marker.LINE_LIST
        assert len(m.points) == 6
        assert len(m.colors) == 6
        origin, x_tip, origin2, y_tip, origin3, z_tip = m.points
        for p in (origin, origin2, origin3):
            assert (p.x, p.y, p.z) == (0.0, 0.0, 0.0)
        assert (x_tip.x, x_tip.y, x_tip.z) == pytest.approx((0.1, 0.0, 0.0))
        assert (y_tip.x, y_tip.y, y_tip.z) == pytest.approx((0.0, 0.1, 0.0))
        assert (z_tip.x, z_tip.y, z_tip.z) == pytest.approx((0.0, 0.0, 0.1))

    def test_ns_and_id_are_set(self):
        m = axis_triad_marker(np.eye(4), "base", "my_ns", 7)
        assert m.ns == "my_ns"
        assert m.id == 7
        assert m.header.frame_id == "base"


class TestLabelMarker:
    def test_text_and_offset_position(self):
        m = label_marker([1.0, 2.0, 3.0], "hello", "base", "ns", 0, z_offset=0.02)
        assert m.type == Marker.TEXT_VIEW_FACING
        assert m.text == "hello"
        assert (m.pose.position.x, m.pose.position.y, m.pose.position.z) == pytest.approx((1.0, 2.0, 3.02))


class TestPolylineMarker:
    def test_points_match_input(self):
        pts = [[0, 0, 0], [1, 0, 0], [1, 1, 0]]
        m = polyline_marker(pts, "base", "ns", 0)
        assert m.type == Marker.LINE_STRIP
        assert len(m.points) == 3
        assert (m.points[1].x, m.points[1].y, m.points[1].z) == pytest.approx((1.0, 0.0, 0.0))


class TestBuildDhFrameMarkers:
    def _fk_result(self):
        table = DHTable(joint_count=2)
        rows, base, tool_t = table.evaluate({1: 0.3, 2: -0.4})
        return forward_kinematics(rows, base, tool_t)

    def test_marker_count_is_triad_plus_label_per_frame_plus_one_polyline(self):
        fk = self._fk_result()
        array = build_dh_frame_markers(fk, "base")
        num_frames = len(fk.transforms)  # "0", "1", "2", "tool"
        assert len(array.markers) == 2 * num_frames + 1

    def test_frames_are_ordered_and_polyline_traces_them(self):
        fk = self._fk_result()
        array = build_dh_frame_markers(fk, "base")
        labels = [m.text for m in array.markers if m.type == Marker.TEXT_VIEW_FACING]
        assert labels == ["0", "1", "2", "tool"]
        polylines = [m for m in array.markers if m.type == Marker.LINE_STRIP]
        assert len(polylines) == 1
        assert len(polylines[0].points) == len(fk.transforms)

    def test_marker_ids_are_unique_within_namespace(self):
        fk = self._fk_result()
        array = build_dh_frame_markers(fk, "base", ns="dh_frames")
        ids = [(m.ns, m.id) for m in array.markers]
        assert len(ids) == len(set(ids))


class TestBuildToolComparisonMarkers:
    def test_error_label_matches_actual_distance(self):
        predicted = np.eye(4)
        truth = np.eye(4).copy()
        truth[0, 3] = 0.01  # 10 mm apart
        array = build_tool_comparison_markers(predicted, truth, "base")
        text_markers = [m for m in array.markers if m.type == Marker.TEXT_VIEW_FACING]
        error_label = next(m for m in text_markers if "mm" in m.text)
        assert error_label.text == "10.00 mm"

    def test_six_markers_two_triads_two_labels_segment_and_error_label(self):
        array = build_tool_comparison_markers(np.eye(4), np.eye(4), "base")
        assert len(array.markers) == 6


def test_clear_namespace_marker_is_delete_all():
    array = clear_namespace_marker("base", "dh_frames")
    assert len(array.markers) == 1
    assert array.markers[0].action == Marker.DELETEALL
    assert array.markers[0].ns == "dh_frames"
