"""One-time extraction of the Lite6's kinematic chain from its live USD
articulation, using ``pxr`` directly. Needs ``pxr`` importable (see
``extract_chain.py`` at the repo root, which adds Isaac Sim's bundled pxr
libraries to ``sys.path`` before calling this) and a stage with the robot
already loaded -- either the live Kit stage (run via autoload_stage.py's
``--exec`` hook) or a standalone-opened one. Never imported by
:mod:`.chain` / :mod:`.analytic_fk`, which must stay pxr-free so they
remain testable without Isaac Sim.

Deliberately specific to this asset's known structure (confirmed in
CLAUDE.md and by the live ``/joint_states`` topic: joints named
``root_joint``, ``joint1``..``joint6``, forming one serial chain) rather
than a generic USD-articulation walker -- a generic walker would need to
solve graph-ordering and body0-vs-body1-as-parent ambiguities that this
asset's own naming already resolves for free.
"""
from __future__ import annotations

import math

from pxr import Usd, UsdPhysics

from .chain import ChainDescription, FixedOffset, JointSpec

ROBOT_PATH = "/World/lite6"
# root_joint welds the base to world and establishes base_link, but is
# not itself one of the driven joints returned in ChainDescription.joints.
JOINT_NAMES_IN_ORDER = ["root_joint"] + [f"joint{i}" for i in range(1, 7)]


def _find_prim_by_name(stage, root_path: str, name: str):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise ValueError(f"no prim at {root_path} -- is the robot loaded in this stage?")
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    raise ValueError(f"no prim named '{name}' found anywhere under {root_path}")


def _axis_token_to_vector(token) -> tuple:
    vector = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}.get(str(token))
    if vector is None:
        raise ValueError(f"unexpected revolute joint axis token {token!r} (expected X, Y, or Z)")
    return vector


def _quat_to_wxyz(q) -> tuple:
    im = q.GetImaginary()
    return (float(q.GetReal()), float(im[0]), float(im[1]), float(im[2]))


def _local_frame(joint_api, side: str) -> FixedOffset:
    """``side`` is ``'0'`` or ``'1'`` -- the joint frame as authored
    relative to body0 (parent) or body1 (child), respectively."""
    pos_attr = getattr(joint_api, f"GetLocalPos{side}Attr")()
    rot_attr = getattr(joint_api, f"GetLocalRot{side}Attr")()
    pos = pos_attr.Get()
    rot = rot_attr.Get()
    translation = tuple(float(c) for c in pos) if pos is not None else (0.0, 0.0, 0.0)
    quat = _quat_to_wxyz(rot) if rot is not None else (1.0, 0.0, 0.0, 0.0)
    return FixedOffset(translation=translation, quaternion_wxyz=quat)


def _resolve_body_path(rel):
    targets = rel.GetTargets()
    return str(targets[0]) if targets else None


def walk_lite6_chain(stage, robot_path: str = ROBOT_PATH) -> ChainDescription:
    """Extract joint1..joint6 as a :class:`~dh_lab.ground_truth.chain.ChainDescription`:
    ``base_link`` is root_joint's child body, ``tool_link`` is joint6's
    child body. Raises :class:`ValueError` with a specific message (naming
    the prim and what was expected) rather than guessing, for every place
    this asset's assumed structure could fail to hold.
    """
    joints = []
    prev_child_link = None
    base_link = None

    for name in JOINT_NAMES_IN_ORDER:
        prim = _find_prim_by_name(stage, robot_path, name)
        joint_api = UsdPhysics.Joint(prim)
        if not joint_api:
            raise ValueError(f"'{name}' at {prim.GetPath()} is not a UsdPhysics Joint (type={prim.GetTypeName()})")

        parent_link = _resolve_body_path(joint_api.GetBody0Rel()) or "world"
        child_link = _resolve_body_path(joint_api.GetBody1Rel())
        if child_link is None:
            raise ValueError(f"joint '{name}' at {prim.GetPath()} has no body1 target (nothing to attach)")

        parent_frame = _local_frame(joint_api, "0")
        child_frame = _local_frame(joint_api, "1")

        if name == "root_joint":
            # The world-welding fixed joint just establishes base_link;
            # it is not part of the joint1..joint6 chain returned.
            base_link = child_link
            prev_child_link = child_link
            continue

        if prev_child_link is not None and parent_link != prev_child_link:
            raise ValueError(
                f"joint '{name}' expected parent link '{prev_child_link}' (the previous joint's child) "
                f"but body0 resolves to '{parent_link}' -- this asset's joints may not be a simple serial "
                f"chain in the assumed order; re-check by hand before trusting the extracted table."
            )

        revolute = UsdPhysics.RevoluteJoint(prim)
        if not revolute:
            raise ValueError(f"expected '{name}' at {prim.GetPath()} to be a revolute joint, got {prim.GetTypeName()}")
        axis = _axis_token_to_vector(revolute.GetAxisAttr().Get())
        lower_deg = revolute.GetLowerLimitAttr().Get()
        upper_deg = revolute.GetUpperLimitAttr().Get()
        # UsdPhysics revolute joint limits are authored in DEGREES -- this
        # is the one line in the whole tool where getting that backwards
        # would silently produce a plausible-looking but wrong number.
        lower_rad = math.radians(lower_deg) if lower_deg is not None else None
        upper_rad = math.radians(upper_deg) if upper_deg is not None else None

        joints.append(JointSpec(
            name=name,
            parent_link=parent_link,
            child_link=child_link,
            joint_type="revolute",
            axis=axis,
            lower_limit=lower_rad,
            upper_limit=upper_rad,
            parent_frame=parent_frame,
            child_frame=child_frame,
        ))
        prev_child_link = child_link

    return ChainDescription(
        robot_prim_path=robot_path,
        base_link=base_link,
        tool_link=prev_child_link,
        joints=joints,
        source_note="extracted via dh_lab.ground_truth.usd_walk from the live Isaac Sim stage",
    )
