"""Visualization content for the DH Lab tool (spec section 8): labelled
axis triads at each DH frame, a stick-figure polyline through them, and
a ghost/tool/ground-truth comparison -- all as ``visualization_msgs/Marker``
content, published as one ``MarkerArray`` for RViz (or any other Marker
subscriber). No Isaac-Sim-specific rendering: this only ever draws
*frames*, never a "ghost robot" reconstructed from a wrong table, since
an incorrect DH table has no single consistent geometry to render --
only the frames it implies (spec section 8).

Needs the stock ``visualization_msgs``/``geometry_msgs``/``std_msgs``
message packages, but not rclpy itself: these are plain message-
construction functions (a ``Node`` only becomes necessary to publish the
result), so they're testable without an rclpy context.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

AXIS_LENGTH_M = 0.05
AXIS_LINE_WIDTH_M = 0.004
TEXT_HEIGHT_M = 0.02
POLYLINE_WIDTH_M = 0.003

_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
_BLUE = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
_WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
_GREY = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8)
_YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)


def _point(xyz) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def _new_marker(frame_id: str, ns: str, marker_id: int, marker_type: int, stamp=None, action=Marker.ADD) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = action
    m.pose.orientation.w = 1.0  # geometry given as absolute world/base points; identity pose
    return m


def axis_triad_marker(
    matrix: np.ndarray, frame_id: str, ns: str, marker_id: int,
    length: float = AXIS_LENGTH_M, stamp=None,
) -> Marker:
    """One LINE_LIST marker drawing all three axes (RGB = XYZ) of a
    single frame's world/base-frame pose ``matrix`` (4x4)."""
    origin = matrix[:3, 3]
    m = _new_marker(frame_id, ns, marker_id, Marker.LINE_LIST, stamp)
    m.scale.x = AXIS_LINE_WIDTH_M
    for axis_index, color in enumerate((_RED, _GREEN, _BLUE)):
        tip = origin + length * matrix[:3, axis_index]
        m.points.append(_point(origin))
        m.points.append(_point(tip))
        m.colors.append(color)
        m.colors.append(color)
    return m


def label_marker(
    origin_xyz, text: str, frame_id: str, ns: str, marker_id: int,
    height: float = TEXT_HEIGHT_M, z_offset: float = TEXT_HEIGHT_M,
    color: Optional[ColorRGBA] = None, stamp=None,
) -> Marker:
    """A TEXT_VIEW_FACING marker at ``origin_xyz`` (a bare point, not a
    full pose: view-facing text always faces the camera regardless of
    any orientation you set, so there's no rotation to give it)."""
    m = _new_marker(frame_id, ns, marker_id, Marker.TEXT_VIEW_FACING, stamp)
    m.pose.position = _point(np.asarray(origin_xyz) + np.array([0.0, 0.0, z_offset]))
    m.scale.z = height
    m.color = color if color is not None else _WHITE
    m.text = text
    return m


def polyline_marker(
    points_xyz: list, frame_id: str, ns: str, marker_id: int,
    width: float = POLYLINE_WIDTH_M, color: Optional[ColorRGBA] = None, stamp=None,
) -> Marker:
    """A LINE_STRIP through consecutive frame origins -- the "stick
    figure" that makes frame-assignment errors legible."""
    m = _new_marker(frame_id, ns, marker_id, Marker.LINE_STRIP, stamp)
    m.scale.x = width
    m.points = [_point(p) for p in points_xyz]
    m.color = color if color is not None else _GREY
    return m


def build_dh_frame_markers(fk_result, frame_id: str, ns: str = "dh_frames", stamp=None) -> MarkerArray:
    """Triads + labels at every frame of the *student's* DH-table FK
    result (:func:`dh_lab.kinematics.transforms.forward_kinematics`,
    keyed ``"0".."N"`` plus ``"tool"``), plus one polyline through their
    origins in chain order -- the stick figure.

    Deliberately not meant for the ground-truth chain's own FKResult
    (:mod:`dh_lab.ground_truth.analytic_fk`, keyed by *link name*, not
    numeric frame index): comparing intermediate frames across the two
    conventions isn't meaningful in general (spec section 5), so this
    only ever draws the student's own frames.
    """
    array = MarkerArray()
    marker_id = 0
    numbered = sorted((name for name in fk_result.transforms if name != "tool"), key=int)
    ordered_names = numbered + (["tool"] if "tool" in fk_result.transforms else [])

    origins = []
    for name in ordered_names:
        matrix = fk_result.transforms[name].matrix
        origins.append(matrix[:3, 3])
        array.markers.append(axis_triad_marker(matrix, frame_id, ns, marker_id, stamp=stamp))
        marker_id += 1
        array.markers.append(label_marker(matrix[:3, 3], name, frame_id, ns, marker_id, stamp=stamp))
        marker_id += 1

    if len(origins) >= 2:
        array.markers.append(polyline_marker(origins, frame_id, ns, marker_id, stamp=stamp))
    return array


def build_tool_comparison_markers(
    predicted_matrix: np.ndarray, ground_truth_matrix: np.ndarray, frame_id: str,
    ns: str = "tool_comparison", stamp=None,
) -> MarkerArray:
    """Ghost triad at the predicted tool pose, a triad at the
    ground-truth tool pose, and a labelled segment between them showing
    the error magnitude (spec section 8, second bullet)."""
    array = MarkerArray()
    predicted_origin = predicted_matrix[:3, 3]
    truth_origin = ground_truth_matrix[:3, 3]
    array.markers.append(axis_triad_marker(predicted_matrix, frame_id, ns, 0, stamp=stamp))
    array.markers.append(label_marker(predicted_origin, "predicted", frame_id, ns, 1, stamp=stamp))
    array.markers.append(axis_triad_marker(ground_truth_matrix, frame_id, ns, 2, stamp=stamp))
    array.markers.append(label_marker(truth_origin, "ground truth", frame_id, ns, 3, stamp=stamp))

    error_mm = float(np.linalg.norm(predicted_origin - truth_origin) * 1000.0)
    array.markers.append(polyline_marker(
        [predicted_origin, truth_origin], frame_id, ns, 4, color=_YELLOW, stamp=stamp,
    ))
    midpoint = (predicted_origin + truth_origin) / 2.0
    array.markers.append(label_marker(
        midpoint, f"{error_mm:.2f} mm", frame_id, ns, 5, color=_YELLOW, stamp=stamp,
    ))
    return array


def clear_namespace_marker(frame_id: str, ns: str, stamp=None) -> MarkerArray:
    """A single DELETEALL marker for ``ns``, wrapped in a MarkerArray --
    publish this to hide a whole layer, the standard RViz per-layer
    toggle idiom (spec section 8: "per-layer toggles; markers must not
    obscure the robot")."""
    m = _new_marker(frame_id, ns, 0, Marker.ARROW, stamp, action=Marker.DELETEALL)
    return MarkerArray(markers=[m])
