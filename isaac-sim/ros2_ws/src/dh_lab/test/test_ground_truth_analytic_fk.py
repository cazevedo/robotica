import math

import numpy as np
import pytest

from dh_lab.ground_truth.analytic_fk import forward_kinematics
from dh_lab.ground_truth.chain import ChainDescription, FixedOffset, JointSpec

_IDENTITY_Q = (1.0, 0.0, 0.0, 0.0)


_IDENTITY_FRAME = FixedOffset(translation=(0.0, 0.0, 0.0), quaternion_wxyz=_IDENTITY_Q)


def _revolute(
    name, parent, child, axis=(0, 0, 1), translation=(0, 0, 0), quat=_IDENTITY_Q,
    lower_limit=-math.pi, upper_limit=math.pi, child_frame=_IDENTITY_FRAME,
):
    return JointSpec(
        name=name, parent_link=parent, child_link=child, joint_type="revolute",
        axis=axis, lower_limit=lower_limit, upper_limit=upper_limit,
        parent_frame=FixedOffset(translation=translation, quaternion_wxyz=quat),
        child_frame=child_frame,
    )


def _fixed(name, parent, child, translation=(0, 0, 0), quat=_IDENTITY_Q, child_frame=_IDENTITY_FRAME):
    return JointSpec(
        name=name, parent_link=parent, child_link=child, joint_type="fixed",
        axis=(0, 0, 1), lower_limit=None, upper_limit=None,
        parent_frame=FixedOffset(translation=translation, quaternion_wxyz=quat),
        child_frame=child_frame,
    )


class TestAnalyticFK:
    def test_no_joints_is_identity(self):
        chain = ChainDescription(robot_prim_path="/World/x", base_link="base", tool_link="base", joints=[])
        result = forward_kinematics(chain, {})
        np.testing.assert_allclose(result.tool.matrix, np.eye(4), atol=1e-12)

    def test_single_revolute_pure_rotation(self):
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="link1",
            joints=[_revolute("j1", "base", "link1")],
        )
        result = forward_kinematics(chain, {"j1": math.pi / 2})
        from dh_lab.kinematics.transforms import rot_z
        np.testing.assert_allclose(result.tool.matrix, rot_z(math.pi / 2), atol=1e-12)

    def test_missing_joint_value_defaults_to_zero(self):
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="link1",
            joints=[_revolute("j1", "base", "link1")],
        )
        result = forward_kinematics(chain, {})  # j1 not supplied
        np.testing.assert_allclose(result.tool.matrix, np.eye(4), atol=1e-12)

    def test_two_link_planar_arm_matches_dh_closed_form(self):
        # Same geometry and closed form as test_fk.py's DH-table version
        # -- two independent FK implementations agreeing is exactly
        # what the real ground-truth-vs-simulation validation checks.
        L1, L2 = 1.0, 1.0
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="tool",
            joints=[
                _revolute("j1", "base", "link1"),
                _revolute("j2", "link1", "link2", translation=(L1, 0, 0)),
                _fixed("tool_fixed", "link2", "tool", translation=(L2, 0, 0)),
            ],
        )
        q1, q2 = math.radians(30), math.radians(45)
        result = forward_kinematics(chain, {"j1": q1, "j2": q2})
        expected_x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        expected_y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        assert result.tool.matrix[0, 3] == pytest.approx(expected_x, abs=1e-9)
        assert result.tool.matrix[1, 3] == pytest.approx(expected_y, abs=1e-9)

    def test_every_link_transform_is_returned(self):
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="link2",
            joints=[_revolute("j1", "base", "link1"), _revolute("j2", "link1", "link2")],
        )
        result = forward_kinematics(chain, {"j1": 0.1, "j2": 0.2})
        assert set(result.transforms) == {"base", "link1", "link2"}

    def test_malformed_chain_raises_clear_error(self):
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="link2",
            joints=[_revolute("j1", "base", "link1"), _revolute("j2", "WRONG_PARENT", "link2")],
        )
        with pytest.raises(ValueError, match="don't meet"):
            forward_kinematics(chain, {"j1": 0.0, "j2": 0.0})


class TestChainJsonRoundtrip:
    def test_save_and_load(self, tmp_path):
        chain = ChainDescription(
            robot_prim_path="/World/lite6",
            base_link="base_link",
            tool_link="link6",
            source_note="unit test fixture",
            joints=[
                _revolute("joint1", "base_link", "link1", lower_limit=-2.0, upper_limit=2.0),
                _fixed("weld", "link1", "link6", translation=(0.1, 0.2, 0.3)),
            ],
        )
        path = tmp_path / "chain.json"
        chain.save(path)
        loaded = ChainDescription.load(path)
        assert loaded.robot_prim_path == chain.robot_prim_path
        assert loaded.base_link == chain.base_link
        assert loaded.tool_link == chain.tool_link
        assert loaded.revolute_joint_names == ["joint1"]
        assert loaded.joints[0].lower_limit == pytest.approx(-2.0)
        assert loaded.joints[1].parent_frame.translation == pytest.approx((0.1, 0.2, 0.3))
        assert loaded.joints[1].child_frame.quaternion_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))


class TestNonIdentityChildFrame:
    def test_child_frame_offset_is_inverted_not_ignored(self):
        # If child_frame were (wrongly) ignored, this would evaluate as a
        # pure rotation with no translation; the correct composition
        # (parent_frame @ Rot(axis,q) @ inverse(child_frame)) must not.
        chain = ChainDescription(
            robot_prim_path="/World/x", base_link="base", tool_link="link1",
            joints=[_revolute(
                "j1", "base", "link1",
                child_frame=FixedOffset(translation=(0.5, 0.0, 0.0), quaternion_wxyz=_IDENTITY_Q),
            )],
        )
        result = forward_kinematics(chain, {"j1": 0.0})
        # At q=0 the two frames must coincide in world space, so the
        # tool ends up translated by -0.5 in the parent/base frame (the
        # link origin sits "before" its own joint-frame offset).
        np.testing.assert_allclose(result.tool.matrix[:3, 3], [-0.5, 0.0, 0.0], atol=1e-12)
