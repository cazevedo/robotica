"""ROS 2-facing code: the DH Lab node and its joint-space trajectory
helper. ``trajectory.py`` is pure Python (no rclpy) and importable/
testable on its own; ``dh_lab_node.py`` needs ``rclpy`` and stock ROS 2
message packages (available from the apt-installed ROS 2 *or* from
Isaac Sim's own bundled rclpy when instantiated from inside Kit -- see
its module docstring)."""
