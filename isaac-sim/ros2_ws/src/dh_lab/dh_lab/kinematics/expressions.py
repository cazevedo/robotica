"""Safe, restricted-grammar parser for DH table cell expressions.

Each DH cell (alpha, a, d, theta) may hold a numeric constant or an
expression in the joint variables ``q1..qN``, e.g. ``q2 - 90`` or
``-q3 / 2``. Expressions are parsed with Python's ``ast`` module and then
walked against an explicit whitelist -- numeric literals, ``q1..qN``,
``pi``, unary ``+``/``-``, and binary ``+ - * /`` -- and rejected
otherwise. ``eval``/``exec`` are never called on the input text, and no
name in it is ever resolved by importing anything: an unrecognised name
simply fails to validate.

Unit caveat (see :mod:`.units` / :mod:`.dh_table` for the full rule): a
bare numeric :class:`Literal` is unit-scaled by whichever toggle applies
to its column, *unconditionally* -- including when it is multiplied or
divided by another literal or by a joint variable. This module is built
for the additive-offset and sign-flip idioms a serial revolute chain
like the Lite6 actually needs (``qN + offset``, ``-qN``), not geared or
scaled-coupling joints; don't write ``2*q1`` expecting "twice q1 in
radians" -- the ``2`` would itself be angle/length-scaled.
"""
from __future__ import annotations

import ast
import math
import re
from dataclasses import dataclass
from typing import Mapping, Union

_Q_NAME_RE = re.compile(r"^q(\d+)$")


class ExpressionError(ValueError):
    """A DH cell expression failed validation.

    Carries the 0-indexed column offset into the original text so a GUI
    can underline the offending token, in addition to a human-readable
    message naming the rejected construct.
    """

    def __init__(self, message: str, text: str, col_offset: int = 0):
        super().__init__(f"{message} (at column {col_offset}): {text!r}")
        self.text = text
        self.col_offset = col_offset
        self.message = message


# A parsed, validated expression is a small tree of the node types below
# -- deliberately disjoint from Python's own `ast` node types, so nothing
# downstream can accidentally hand a raw `ast.AST` to `eval`/`compile`.

@dataclass(frozen=True)
class Literal:
    """A bare numeric constant, e.g. ``90`` or ``0.5``. Unit-scaled by
    the caller (see module docstring); this class does not know its own
    units."""
    value: float


@dataclass(frozen=True)
class Pi:
    """The dimensionless mathematical constant pi. Never unit-scaled."""


@dataclass(frozen=True)
class JointVar:
    """A reference to joint variable ``q{index}`` (1-based). Always
    substituted with a value already in SI radians -- never unit-scaled."""
    index: int


@dataclass(frozen=True)
class UnaryOp:
    op: str  # "+" or "-"
    operand: "Expr"


@dataclass(frozen=True)
class BinOp:
    op: str  # "+", "-", "*", "/"
    left: "Expr"
    right: "Expr"


Expr = Union[Literal, Pi, JointVar, UnaryOp, BinOp]

_BINOP_TYPES = {ast.Add: "+", ast.Sub: "-", ast.Mult: "*", ast.Div: "/"}
_UNARYOP_TYPES = {ast.UAdd: "+", ast.USub: "-"}


def parse_expression(text: str, joint_count: int) -> Expr:
    """Parse and validate a DH cell expression string.

    Allowed grammar: numeric literals, the symbols ``q1..q{joint_count}``
    and ``pi``, unary ``+``/``-``, and binary ``+ - * /``. Anything else
    (function calls, attribute access, subscripting, comparisons, boolean
    operators, imports, string/collection literals, ...) raises
    :class:`ExpressionError` naming the rejected construct.

    Never calls ``eval``/``exec`` and never imports anything on behalf of
    the input text.
    """
    text_stripped = text.strip()
    if not text_stripped:
        raise ExpressionError("empty expression", text, 0)
    try:
        tree = ast.parse(text_stripped, mode="eval")
    except SyntaxError as exc:
        raise ExpressionError(f"syntax error: {exc.msg}", text, exc.offset or 0) from exc
    return _walk(tree.body, text, joint_count)


def _walk(node: ast.AST, text: str, joint_count: int) -> Expr:
    col = getattr(node, "col_offset", 0)

    if isinstance(node, ast.Constant):
        if isinstance(node.value, bool) or not isinstance(node.value, (int, float)):
            raise ExpressionError(
                f"only numeric literals are allowed, got {type(node.value).__name__}", text, col,
            )
        return Literal(float(node.value))

    if isinstance(node, ast.Name):
        if node.id == "pi":
            return Pi()
        match = _Q_NAME_RE.match(node.id)
        if match:
            index = int(match.group(1))
            if 1 <= index <= joint_count:
                return JointVar(index)
            raise ExpressionError(
                f"q{index} is out of range (table has {joint_count} joint(s))", text, col,
            )
        raise ExpressionError(
            f"unknown symbol '{node.id}' (only q1..q{joint_count} and pi are allowed)", text, col,
        )

    if isinstance(node, ast.UnaryOp):
        op = _UNARYOP_TYPES.get(type(node.op))
        if op is None:
            raise ExpressionError(f"operator '{type(node.op).__name__}' is not allowed", text, col)
        return UnaryOp(op, _walk(node.operand, text, joint_count))

    if isinstance(node, ast.BinOp):
        op = _BINOP_TYPES.get(type(node.op))
        if op is None:
            raise ExpressionError(f"operator '{type(node.op).__name__}' is not allowed", text, col)
        return BinOp(op, _walk(node.left, text, joint_count), _walk(node.right, text, joint_count))

    raise ExpressionError(f"'{type(node).__name__}' is not allowed in a DH cell expression", text, col)


def evaluate(expr: Expr, q_rad: Mapping[int, float], literal_scale: float = 1.0) -> float:
    """Evaluate a parsed expression.

    ``q_rad`` maps joint index (1-based) to its current value **in
    radians**, regardless of the GUI's angle-unit toggle. ``literal_scale``
    is applied only to bare :class:`Literal` leaves (the caller picks it
    based on whether this cell is angle- or length-typed and which unit
    is currently toggled); ``pi`` and joint variables are never scaled.
    """
    if isinstance(expr, Literal):
        return expr.value * literal_scale
    if isinstance(expr, Pi):
        return math.pi
    if isinstance(expr, JointVar):
        try:
            return q_rad[expr.index]
        except KeyError as exc:
            raise ValueError(f"no value supplied for q{expr.index}") from exc
    if isinstance(expr, UnaryOp):
        value = evaluate(expr.operand, q_rad, literal_scale)
        return value if expr.op == "+" else -value
    if isinstance(expr, BinOp):
        left = evaluate(expr.left, q_rad, literal_scale)
        right = evaluate(expr.right, q_rad, literal_scale)
        if expr.op == "+":
            return left + right
        if expr.op == "-":
            return left - right
        if expr.op == "*":
            return left * right
        return left / right
    raise TypeError(f"unreachable: unknown expression node {expr!r}")


def free_joint_indices(expr: Expr) -> set[int]:
    """Return the set of joint indices (``{2}`` for ``q2 - 90``) an
    expression actually depends on. Used by the sweep diagnostic (only a
    row whose cell mentions ``q_j`` moves when ``q_j`` is swept) and by
    the GUI to grey out irrelevant rows."""
    if isinstance(expr, JointVar):
        return {expr.index}
    if isinstance(expr, (Literal, Pi)):
        return set()
    if isinstance(expr, UnaryOp):
        return free_joint_indices(expr.operand)
    if isinstance(expr, BinOp):
        return free_joint_indices(expr.left) | free_joint_indices(expr.right)
    raise TypeError(f"unreachable: unknown expression node {expr!r}")
