import math

import pytest

from dh_lab.kinematics.expressions import (
    ExpressionError,
    evaluate,
    free_joint_indices,
    parse_expression,
)


def _eval(text: str, joint_count: int, q_rad: dict, scale: float = 1.0) -> float:
    return evaluate(parse_expression(text, joint_count), q_rad, scale)


class TestValidExpressions:
    def test_bare_literal(self):
        assert _eval("0", 6, {}) == 0.0
        assert _eval("44.5", 6, {}) == 44.5

    def test_joint_reference(self):
        assert _eval("q1", 6, {1: 1.23}) == pytest.approx(1.23)

    def test_pi_is_never_scaled(self):
        # A scale of 1000 would blow up a literal, but pi must stay pi.
        assert _eval("pi", 6, {}, scale=1000.0) == pytest.approx(math.pi)

    def test_unary_minus(self):
        assert _eval("-q2", 6, {2: 0.5}) == pytest.approx(-0.5)

    def test_unary_plus(self):
        assert _eval("+q2", 6, {2: 0.5}) == pytest.approx(0.5)

    def test_offset_expression(self):
        # The canonical "revolute joint with a constant offset" pattern.
        assert _eval("q2 - 90", 6, {2: 0.0}, scale=math.pi / 180) == pytest.approx(-math.pi / 2)

    def test_arithmetic_combo(self):
        assert _eval("(q1 + 1) / 2", 6, {1: 3.0}) == pytest.approx(2.0)

    def test_literal_scaling_applies_only_to_literals(self):
        expr = parse_expression("q1 - 90", 3)
        # q1 untouched by scale, bare 90 fully scaled.
        assert evaluate(expr, {1: 0.0}, literal_scale=2.0) == pytest.approx(0.0 - 180.0)

    def test_joint_index_at_upper_bound(self):
        assert _eval("q6", 6, {6: 1.0}) == 1.0

    def test_whitespace_tolerant(self):
        assert _eval("  q1  -  90  ", 6, {1: 0.0}) == pytest.approx(-90.0)


class TestFreeJointIndices:
    def test_simple(self):
        assert free_joint_indices(parse_expression("q2 - 90", 6)) == {2}

    def test_constant_has_none(self):
        assert free_joint_indices(parse_expression("90", 6)) == set()

    def test_multiple_joints(self):
        assert free_joint_indices(parse_expression("q1 + q3", 6)) == {1, 3}

    def test_unary(self):
        assert free_joint_indices(parse_expression("-q4", 6)) == {4}


class TestRejectedExpressions:
    @pytest.mark.parametrize(
        "text",
        [
            "__import__('os')",
            "import os",
            "os.system('ls')",
            "q1.bit_length()",
            "abs(q1)",
            "(lambda: 1)()",
            "[q1 for _ in range(1)]",
            "q1 if q1 else 0",
            "q1 == 0",
            "q1 and q2",
            "'hello'",
            "True",
            "q1 ** 2",
            "q1 % 2",
            "q1; q2",
        ],
    )
    def test_rejected(self, text):
        with pytest.raises(ExpressionError):
            parse_expression(text, 6)

    def test_unknown_symbol(self):
        with pytest.raises(ExpressionError):
            parse_expression("qq1", 6)

    def test_out_of_range_joint(self):
        with pytest.raises(ExpressionError):
            parse_expression("q7", 6)

    def test_empty_expression(self):
        with pytest.raises(ExpressionError):
            parse_expression("   ", 6)

    def test_syntax_error(self):
        with pytest.raises(ExpressionError):
            parse_expression("q1 +", 6)

    def test_never_calls_eval_or_exec(self):
        # A payload that would prove eval/exec ran, if it ever did.
        marker = {"ran": False}
        text = "__import__('builtins').exec('marker[\"ran\"] = True')"
        with pytest.raises(ExpressionError):
            parse_expression(text, 6)
        assert marker["ran"] is False
