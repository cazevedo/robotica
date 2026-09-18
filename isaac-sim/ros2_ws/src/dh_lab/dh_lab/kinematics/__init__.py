"""Pure Python + NumPy kinematics core for the DH Lab teaching tool.

No Isaac Sim or ROS 2 imports anywhere in this package -- every module
here is importable and unit-testable with nothing running (see
``../../test/``). Isaac/ROS-specific code (the ground-truth-from-USD
model, the ROS 2 node, the omni.ui GUI) lives in sibling packages and
imports *this* package, never the other way around.
"""
