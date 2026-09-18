"""The parametric Denavit-Hartenberg table: the tool's central data
structure. Each row holds four Craig (modified-DH) parameters
``(alpha_{i-1}, a_{i-1}, d_i, theta_i)``; any cell may be a numeric
constant or an expression in ``q1..qN`` (see :mod:`.expressions`).

Bare numeric literals in a cell are interpreted in *this table's own*
current angle/length unit (``angle_unit``/``length_unit`` below), which
is saved and loaded as part of the table's JSON rather than left to
whatever a GUI toggle happens to show at load time -- so a saved file
always means the same physical table regardless of the unit mode active
when it is later reopened. ``pi`` and ``q1..qN`` are never unit-scaled:
internal storage and every evaluated number is SI (metres, radians).

See :mod:`.expressions` for the literal-scaling caveat this table
inherits (built for additive offsets and sign flips, not geared joints).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .expressions import Expr, ExpressionError, evaluate, parse_expression
from .units import AngleUnit, LengthUnit, angle_literal_to_si, length_literal_to_si

_DEFAULT_ROW_COUNT = 6
_ROW_CELLS = ("alpha", "a", "d", "theta")
_FIXED_CELLS = ("x", "y", "z", "roll", "pitch", "yaw")


@dataclass
class Cell:
    """One DH-table cell: the text a student typed, and either its parsed
    expression or the validation error it produced. Never both set."""
    text: str = "0"
    expr: Optional[Expr] = None
    error: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        return self.error is None


@dataclass
class Row:
    alpha: Cell = field(default_factory=lambda: Cell("0"))
    a: Cell = field(default_factory=lambda: Cell("0"))
    d: Cell = field(default_factory=lambda: Cell("0"))
    theta: Cell = field(default_factory=lambda: Cell("0"))

    def cell(self, name: str) -> Cell:
        return getattr(self, name)


@dataclass
class FixedTransformCells:
    """Six constant cells (xyz translation, rpy rotation) for the fixed
    base (``world -> frame0``) or tool (``frameN -> tool``) transform.
    Constants only -- these are not functions of q, so they are parsed
    with ``joint_count=0`` and any ``qN`` reference is rejected."""
    x: Cell = field(default_factory=lambda: Cell("0"))
    y: Cell = field(default_factory=lambda: Cell("0"))
    z: Cell = field(default_factory=lambda: Cell("0"))
    roll: Cell = field(default_factory=lambda: Cell("0"))
    pitch: Cell = field(default_factory=lambda: Cell("0"))
    yaw: Cell = field(default_factory=lambda: Cell("0"))


@dataclass
class EvaluatedRow:
    """Numeric (SI) alpha/a/d/theta for one row at a specific ``q``."""
    alpha: float
    a: float
    d: float
    theta: float


@dataclass
class EvaluatedFixedTransform:
    """Numeric (SI) xyz + rpy for a fixed base/tool transform."""
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


class DHTable:
    """The student's parametric DH table.

    Row count defaults to 6 (the Lite6 is a 6R arm) but is adjustable.
    ``angle_unit``/``length_unit`` govern how bare numeric literals in
    *this table's* cells are interpreted, and are saved/loaded with it
    (see module docstring) -- they are this table's own state, not a
    transient GUI setting.
    """

    def __init__(self, joint_count: int = _DEFAULT_ROW_COUNT):
        if joint_count < 1:
            raise ValueError("a DH table needs at least one row/joint")
        self.joint_count = joint_count
        self.angle_unit = AngleUnit.DEGREES
        self.length_unit = LengthUnit.MILLIMETRES
        self.base_transform = FixedTransformCells()
        self.tool_transform = FixedTransformCells()
        self.rows: list[Row] = [self._default_row(i) for i in range(joint_count)]
        self._reparse_fixed(self.base_transform)
        self._reparse_fixed(self.tool_transform)

    def _default_row(self, index: int) -> Row:
        # theta defaults to this row's own joint variable (the common
        # case for a revolute chain); everything else defaults to 0.
        row = Row(theta=Cell(f"q{index + 1}"))
        self._reparse_row(row)
        return row

    # --- editing -----------------------------------------------------

    def set_row_count(self, joint_count: int) -> None:
        if joint_count < 1:
            raise ValueError("a DH table needs at least one row/joint")
        if joint_count == self.joint_count:
            return
        if joint_count > self.joint_count:
            self.rows.extend(self._default_row(i) for i in range(self.joint_count, joint_count))
        else:
            self.rows = self.rows[:joint_count]
        self.joint_count = joint_count
        # Row count changed which q symbols are in range -- every cell's
        # parse (not just the new/removed rows') must be redone.
        for row in self.rows:
            self._reparse_row(row)
        self._reparse_fixed(self.base_transform)
        self._reparse_fixed(self.tool_transform)

    def set_cell(self, row_index: int, column: str, text: str) -> Cell:
        """Set one DH-row cell's text, parse it immediately, and return
        the resulting :class:`Cell` (check ``.error`` for an inline
        validation message; never raises for a bad expression)."""
        if column not in _ROW_CELLS:
            raise ValueError(f"unknown DH column '{column}' (expected one of {_ROW_CELLS})")
        cell = Cell(text)
        self._parse_into(cell)
        setattr(self.rows[row_index], column, cell)
        return cell

    def set_fixed_cell(self, which: str, field_name: str, text: str) -> Cell:
        """Set one cell of the fixed base or tool transform
        (``which`` is ``'base'`` or ``'tool'``)."""
        if which not in ("base", "tool"):
            raise ValueError("which must be 'base' or 'tool'")
        target = self.base_transform if which == "base" else self.tool_transform
        cell = Cell(text)
        self._parse_into(cell, joint_count=0)
        setattr(target, field_name, cell)
        return cell

    def set_units(self, angle_unit: AngleUnit, length_unit: LengthUnit) -> None:
        """Change this table's own unit mode. Does not touch cell text --
        only how literals in it are interpreted from now on."""
        self.angle_unit = angle_unit
        self.length_unit = length_unit

    # --- parsing -------------------------------------------------------

    def _reparse_row(self, row: Row) -> None:
        for name in _ROW_CELLS:
            self._parse_into(row.cell(name))

    def _reparse_fixed(self, fixed: FixedTransformCells) -> None:
        for name in _FIXED_CELLS:
            self._parse_into(getattr(fixed, name), joint_count=0)

    def _parse_into(self, cell: Cell, joint_count: Optional[int] = None) -> None:
        n = self.joint_count if joint_count is None else joint_count
        try:
            cell.expr = parse_expression(cell.text, n)
            cell.error = None
        except ExpressionError as exc:
            cell.expr = None
            cell.error = str(exc)

    @property
    def is_valid(self) -> bool:
        """True only if every cell in the table parsed without error."""
        cells = [row.cell(name) for row in self.rows for name in _ROW_CELLS]
        cells += [getattr(self.base_transform, f) for f in _FIXED_CELLS]
        cells += [getattr(self.tool_transform, f) for f in _FIXED_CELLS]
        return all(c.is_valid for c in cells)

    # --- evaluation ------------------------------------------------------

    def evaluate(
        self, q_rad: dict[int, float]
    ) -> tuple[list[EvaluatedRow], EvaluatedFixedTransform, EvaluatedFixedTransform]:
        """Evaluate every cell at ``q_rad`` (joint index -> radians).

        Raises if any cell failed to parse (check :attr:`is_valid` first)
        or if ``q_rad`` is missing an index a cell references. Returns
        ``(rows, base_transform, tool_transform)``, all numeric SI values
        -- this is the "evaluated table" the GUI shows next to the
        symbolic one, updating live as ``q`` changes.
        """
        if not self.is_valid:
            raise ValueError("cannot evaluate a DH table with invalid cells")
        rows = [
            EvaluatedRow(
                alpha=evaluate(row.alpha.expr, q_rad, self._angle_scale()),
                a=evaluate(row.a.expr, q_rad, self._length_scale()),
                d=evaluate(row.d.expr, q_rad, self._length_scale()),
                theta=evaluate(row.theta.expr, q_rad, self._angle_scale()),
            )
            for row in self.rows
        ]
        base = self._evaluate_fixed(self.base_transform, q_rad)
        tool = self._evaluate_fixed(self.tool_transform, q_rad)
        return rows, base, tool

    def _evaluate_fixed(self, fixed: FixedTransformCells, q_rad: dict[int, float]) -> EvaluatedFixedTransform:
        return EvaluatedFixedTransform(
            x=evaluate(fixed.x.expr, q_rad, self._length_scale()),
            y=evaluate(fixed.y.expr, q_rad, self._length_scale()),
            z=evaluate(fixed.z.expr, q_rad, self._length_scale()),
            roll=evaluate(fixed.roll.expr, q_rad, self._angle_scale()),
            pitch=evaluate(fixed.pitch.expr, q_rad, self._angle_scale()),
            yaw=evaluate(fixed.yaw.expr, q_rad, self._angle_scale()),
        )

    def _angle_scale(self) -> float:
        return angle_literal_to_si(1.0, self.angle_unit)

    def _length_scale(self) -> float:
        return length_literal_to_si(1.0, self.length_unit)

    # --- persistence -----------------------------------------------------

    def to_json_dict(self) -> dict:
        def fixed_dict(fixed: FixedTransformCells) -> dict:
            return {f: getattr(fixed, f).text for f in _FIXED_CELLS}

        return {
            "joint_count": self.joint_count,
            "angle_unit": self.angle_unit.value,
            "length_unit": self.length_unit.value,
            "rows": [{name: row.cell(name).text for name in _ROW_CELLS} for row in self.rows],
            "base_transform": fixed_dict(self.base_transform),
            "tool_transform": fixed_dict(self.tool_transform),
        }

    @classmethod
    def from_json_dict(cls, data: dict) -> "DHTable":
        table = cls(joint_count=data["joint_count"])
        table.angle_unit = AngleUnit(data["angle_unit"])
        table.length_unit = LengthUnit(data["length_unit"])
        for row, row_data in zip(table.rows, data["rows"]):
            for name in _ROW_CELLS:
                cell = Cell(row_data[name])
                table._parse_into(cell)
                setattr(row, name, cell)
        for which in ("base", "tool"):
            fixed = table.base_transform if which == "base" else table.tool_transform
            key = f"{which}_transform"
            for f in _FIXED_CELLS:
                cell = Cell(data[key][f])
                table._parse_into(cell, joint_count=0)
                setattr(fixed, f, cell)
        return table

    def save(self, path: Path | str) -> None:
        Path(path).write_text(json.dumps(self.to_json_dict(), indent=2))

    @classmethod
    def load(cls, path: Path | str) -> "DHTable":
        return cls.from_json_dict(json.loads(Path(path).read_text()))
