"""The tool's ground truth: an independent forward-kinematics model built
from the robot's own description, not from any student DH table.

:mod:`.chain` and :mod:`.analytic_fk` are pure Python + NumPy (no pxr, no
Isaac Sim, no ROS, no network) -- they load an already-extracted
:class:`~dh_lab.ground_truth.chain.ChainDescription` from JSON and
evaluate it at any configuration instantly. Only :mod:`.usd_walk`, which
performs the one-time extraction itself, needs ``pxr`` and a live or
locally-resolvable USD stage; it is never imported by the other two.
"""
