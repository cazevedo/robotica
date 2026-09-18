"""Static description of the robot's kinematic chain, as extracted once
from its USD/articulation description (see :mod:`.usd_walk`, which needs
``pxr`` and is not imported here) and cached to JSON. Loading and
evaluating this description needs nothing but NumPy -- no pxr, no Isaac
Sim, no network -- which is what lets the ground-truth model
(:mod:`.analytic_fk`) be evaluated instantly at any configuration.

Assumes a single **serial** chain from ``base_link`` to ``tool_link``
(true for the Lite6: joint1..joint6 in series, no branching) -- extending
to a branching tree is not attempted since nothing here needs it.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class FixedOffset:
    """A constant rigid-body offset: translation (metres) + quaternion
    ``(w, x, y, z)`` rotation."""
    translation: tuple
    quaternion_wxyz: tuple


@dataclass(frozen=True)
class JointSpec:
    """One joint of the extracted chain, already in SI (radians, metres)
    regardless of the units the source schema used -- UsdPhysics revolute
    joint limits are authored in *degrees*, converted to radians once at
    extraction time (see :mod:`.usd_walk`), not left for a caller to get
    wrong later.

    A USD Physics joint is authored as two local frames, one relative to
    each body (``parent_frame`` relative to ``parent_link``, ``child_frame``
    relative to ``child_link``) -- they coincide in world space exactly
    when ``child_link`` is rotated about ``axis`` by the current joint
    angle relative to ``parent_link``. Both are kept (rather than
    assuming ``child_frame`` is identity) because the schema does not
    guarantee that; see :func:`dh_lab.ground_truth.analytic_fk._joint_transform`
    for the composition this enables: ``parent_frame @ Rot(axis, q) @
    inverse(child_frame)``.
    """
    name: str
    parent_link: str
    child_link: str
    joint_type: str  # "revolute" or "fixed"
    axis: tuple  # unit vector, in the joint frame (ignored if fixed)
    lower_limit: Optional[float]  # radians; None for a fixed joint
    upper_limit: Optional[float]
    parent_frame: FixedOffset
    child_frame: FixedOffset


@dataclass(frozen=True)
class ChainDescription:
    """The full extracted chain from the articulation's base link to a
    chosen tool link, in joint order (``joints[0].parent_link ==
    base_link``, and each subsequent joint's ``parent_link`` equals the
    previous joint's ``child_link``)."""
    robot_prim_path: str
    base_link: str
    tool_link: str
    joints: list = field(default_factory=list)
    source_note: str = ""

    def to_json_dict(self) -> dict:
        def offset_dict(o: FixedOffset) -> dict:
            return {"translation": list(o.translation), "quaternion_wxyz": list(o.quaternion_wxyz)}

        return {
            "robot_prim_path": self.robot_prim_path,
            "base_link": self.base_link,
            "tool_link": self.tool_link,
            "source_note": self.source_note,
            "joints": [
                {
                    "name": j.name,
                    "parent_link": j.parent_link,
                    "child_link": j.child_link,
                    "joint_type": j.joint_type,
                    "axis": list(j.axis),
                    "lower_limit": j.lower_limit,
                    "upper_limit": j.upper_limit,
                    "parent_frame": offset_dict(j.parent_frame),
                    "child_frame": offset_dict(j.child_frame),
                }
                for j in self.joints
            ],
        }

    @classmethod
    def from_json_dict(cls, data: dict) -> "ChainDescription":
        def offset_from(d: dict) -> FixedOffset:
            return FixedOffset(translation=tuple(d["translation"]), quaternion_wxyz=tuple(d["quaternion_wxyz"]))

        joints = [
            JointSpec(
                name=j["name"],
                parent_link=j["parent_link"],
                child_link=j["child_link"],
                joint_type=j["joint_type"],
                axis=tuple(j["axis"]),
                lower_limit=j["lower_limit"],
                upper_limit=j["upper_limit"],
                parent_frame=offset_from(j["parent_frame"]),
                child_frame=offset_from(j["child_frame"]),
            )
            for j in data["joints"]
        ]
        return cls(
            robot_prim_path=data["robot_prim_path"],
            base_link=data["base_link"],
            tool_link=data["tool_link"],
            joints=joints,
            source_note=data.get("source_note", ""),
        )

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_json_dict(), indent=2))

    @classmethod
    def load(cls, path) -> "ChainDescription":
        return cls.from_json_dict(json.loads(Path(path).read_text()))

    @property
    def revolute_joint_names(self) -> list:
        return [j.name for j in self.joints if j.joint_type == "revolute"]
