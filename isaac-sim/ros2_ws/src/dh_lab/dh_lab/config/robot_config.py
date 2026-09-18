"""Static, instructor-editable configuration: which real robot joints
``q1..qN`` map to (by name, not message order -- see
:meth:`RobotConfig.reorder_joint_state`), display/pass tolerances, and
named preset configurations. Loaded from JSON so a different joint
arrangement only needs a new config file, not a code change.

Joint *limits* are deliberately not duplicated here -- they come from
the extracted :class:`~dh_lab.ground_truth.chain.ChainDescription` (the
one place that already has them, in SI, from the robot's own
description), loaded separately by whatever combines the two (the ROS 2
node, the GUI, the headless grader).

``base_link``/``tool_link`` default to ``None``, meaning "use whatever
:class:`ChainDescription` reports" -- set them here only to deliberately
override the chain's own choice (see spec: base/tool frame choice must
be explicit and configurable, and reported to the student).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class Tolerances:
    """Pass/warn/fail thresholds for the colour-coded error panel."""
    position_green_mm: float = 1.0
    position_amber_mm: float = 10.0
    orientation_green_deg: float = 0.5
    orientation_amber_deg: float = 5.0


@dataclass
class RobotConfig:
    joint_names: list  # q1..qN, in order -- q{i} maps to joint_names[i-1]
    base_link: Optional[str] = None
    tool_link: Optional[str] = None
    tolerances: Tolerances = field(default_factory=Tolerances)
    presets: dict = field(default_factory=dict)  # name -> {joint_name: radians}
    default_speed_scale: float = 0.2  # fraction of max_joint_speed_rad_s; spec: "default slow"
    max_joint_speed_rad_s: float = 1.0  # conservative placeholder -- see README assumptions

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    def joint_name_for(self, q_index: int) -> str:
        """``q_index`` is 1-based: ``joint_name_for(1)`` is ``joint_names[0]``."""
        return self.joint_names[q_index - 1]

    def q_index_for(self, joint_name: str) -> int:
        return self.joint_names.index(joint_name) + 1

    def reorder_joint_state(self, names: list, positions: list) -> dict:
        """Map an incoming ``sensor_msgs/JointState``'s ``(name,
        position)`` -- in whatever order the message happens to list
        them, which is *not* guaranteed to match ``joint_names`` -- to
        ``{q_index: radians}`` for this config's ``q1..qN``.

        Silently ignores any name in the message this config doesn't
        know about (extra sensors, a gripper, ...); raises if one of
        *this config's* joint names is missing from the message, since
        silently evaluating a table with a stale or missing joint value
        would be worse than failing loudly.
        """
        by_name = dict(zip(names, positions))
        result = {}
        for i, joint_name in enumerate(self.joint_names, start=1):
            if joint_name not in by_name:
                raise KeyError(f"joint '{joint_name}' (q{i}) not present in this JointState message")
            result[i] = by_name[joint_name]
        return result

    def color_for_error(self, position_mm: float, orientation_deg: float) -> str:
        """``'green'``/``'amber'``/``'red'``, per the worse of the two errors."""
        t = self.tolerances
        if position_mm < t.position_green_mm and orientation_deg < t.orientation_green_deg:
            return "green"
        if position_mm < t.position_amber_mm and orientation_deg < t.orientation_amber_deg:
            return "amber"
        return "red"

    def to_json_dict(self) -> dict:
        return {
            "joint_names": list(self.joint_names),
            "base_link": self.base_link,
            "tool_link": self.tool_link,
            "tolerances": {
                "position_green_mm": self.tolerances.position_green_mm,
                "position_amber_mm": self.tolerances.position_amber_mm,
                "orientation_green_deg": self.tolerances.orientation_green_deg,
                "orientation_amber_deg": self.tolerances.orientation_amber_deg,
            },
            "presets": self.presets,
            "default_speed_scale": self.default_speed_scale,
            "max_joint_speed_rad_s": self.max_joint_speed_rad_s,
        }

    @classmethod
    def from_json_dict(cls, data: dict) -> "RobotConfig":
        tol_data = data.get("tolerances") or {}
        return cls(
            joint_names=list(data["joint_names"]),
            base_link=data.get("base_link"),
            tool_link=data.get("tool_link"),
            tolerances=Tolerances(**tol_data),
            presets=data.get("presets", {}),
            default_speed_scale=data.get("default_speed_scale", 0.2),
            max_joint_speed_rad_s=data.get("max_joint_speed_rad_s", 1.0),
        )

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_json_dict(), indent=2))

    @classmethod
    def load(cls, path) -> "RobotConfig":
        return cls.from_json_dict(json.loads(Path(path).read_text()))
