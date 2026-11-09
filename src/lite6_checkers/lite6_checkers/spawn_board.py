"""
Put the checkerboard into the running Gazebo world.

    ros2 run lite6_checkers spawn_board [--x X] [--y Y] [--z Z] [--yaw RAD]

It waits for the world's `create` service to appear (Gazebo takes a while to start,
longer the first time, when it downloads the table from Fuel), then asks for the model.
`lab sim` and `lab sim-lite` start it for you; run it by hand to put a board back after
`gz service ... remove`, or to try another pose.

The default pose is where the board sits in the beside-table world of xarm_ros2: on the
tabletop, in front of the Lite 6. See the README for how it was chosen.
"""
import argparse
import math
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory

# World coordinates (m, rad). xarm_gazebo spawns the robot base at (-0.2, -0.5, 1.021) with a
# yaw of 1.571, on the edge of a table (x -0.75..0.75, y -1.24..-0.44) whose top is at
# z = 1.015. The robot faces +y, off the table, so the board goes on the table behind it:
# on the robot's axis, its near edge 0.12 m from the base axis (clear of the base), the
# 240 mm playing area ending 0.36 m from it. White's side faces the robot.
DEFAULT_X = -0.20
DEFAULT_Y = -0.74
DEFAULT_Z = 1.025       # tabletop (1.015) + slab thickness (0.010): the playing surface
DEFAULT_YAW = -math.pi / 2
WORLD = 'default'       # the <world name=...> of xarm_gazebo/worlds/table_gz.world


def create_request(sdf_filename, name, x, y, z, yaw):
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (f'sdf_filename: "{sdf_filename}" name: "{name}" '
            f'pose: {{position: {{x: {x} y: {y} z: {z}}} '
            f'orientation: {{x: 0 y: 0 z: {qz} w: {qw}}}}}')


def try_create(world, request):
    """One attempt. Returns (created, message)."""
    cmd = ['gz', 'service', '-s', f'/world/{world}/create',
           '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', request]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except FileNotFoundError:
        sys.exit('spawn_board: the `gz` command is not on the PATH (is this the lab container?)')
    except subprocess.TimeoutExpired:
        return False, 'service call timed out'
    text = (out.stdout + out.stderr).strip()
    return (out.returncode == 0 and 'true' in out.stdout), text


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--x', type=float, default=DEFAULT_X, help='world x of the board centre, m')
    ap.add_argument('--y', type=float, default=DEFAULT_Y, help='world y of the board centre, m')
    ap.add_argument('--z', type=float, default=DEFAULT_Z, help='world z of the playing surface, m')
    ap.add_argument('--yaw', type=float, default=DEFAULT_YAW,
                    help='rotation of the board about z, rad; the board x axis (White -> Black) points '
                         'along the direction yaw')
    ap.add_argument('--name', default='checkerboard')
    ap.add_argument('--world', default=WORLD)
    ap.add_argument('--timeout', type=float, default=240.0, help='give up after this many seconds')
    args = ap.parse_args()

    sdf = get_package_share_directory('lite6_checkers') + '/models/checkerboard/model.sdf'
    request = create_request(sdf, args.name, args.x, args.y, args.z, args.yaw)

    print(f'spawn_board: waiting for Gazebo world "{args.world}" ...', flush=True)
    deadline = time.monotonic() + args.timeout
    last = ''
    while time.monotonic() < deadline:
        created, last = try_create(args.world, request)
        if created:
            print(f'spawn_board: "{args.name}" is in the world at '
                  f'x={args.x:.3f} y={args.y:.3f} z={args.z:.3f} yaw={args.yaw:.3f}', flush=True)
            return
        time.sleep(2.0)
    sys.exit(f'spawn_board: gave up after {args.timeout:.0f} s. Last answer: {last or "(none)"}')


if __name__ == '__main__':
    main()
