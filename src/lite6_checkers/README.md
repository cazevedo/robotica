# lite6_checkers

The checkerboard of the challenge, as a Gazebo model, and the node that puts it in the simulation.

**8 × 8 squares of 30 mm: a 240 mm × 240 mm playing area** (Challenge, §2). It has a 12 mm
frame around the playing area, which is cosmetic and not part of the specification, and a slab 10 mm
thick. The dark squares are the ones the game is played on: a1 is dark, at White's left-hand
corner, so the long dark diagonal a1–h8 runs to each player's left (§1.1).

`lab sim` and `lab sim-lite` start the board for you: it appears a few seconds after Gazebo opens.

## Where it is

The beside-table world of `xarm_ros2` is hard-coded in that package, so the board is spawned into the
running world instead of being written into it (`spawn_board` waits for Gazebo, spawns, exits).

| | World frame |
|---|---|
| Table | top at z = 1.015, x ∈ [−0.75, 0.75], y ∈ [−1.24, −0.44] |
| Robot base | (−0.2, −0.5, 1.021), yaw 90°: it faces +y, **off the table** |
| Board | centre (−0.20, −0.74), playing surface at z = 1.025, yaw −90° |

Upstream turns the robot to face +y, off the table. The lab's copy of `xarm_gazebo`
(`src/xarm_gazebo`, see its `LAB.md`) turns it to face −y instead, onto the table, so the board is
**in front of the robot, on its axis**, 0.24 m from the base axis to its centre: the playing area
spans 0.12 m to 0.36 m, White's side towards the robot, the slab clear of the base.

Checked against the Lite 6 kinematics with the gripper fitted (tool 83.6 mm long) and the tool
vertical: every square, every corner and every edge midpoint of the 240 mm area has an inverse
kinematic solution inside the joint limits at 5 mm (pushing), 17 mm (grasping) and 50–120 mm
(flying), for tool yaws of 0°, 30°, 45°, 60° and ±90°, with the nearest joint limit 17° away. In
Gazebo, the arm driven to a1, h1, a8 and h8 reached each within 0.3 mm of the expected position.
Closer than about 0.18 m the near squares run into the elbow and wrist limits; beyond 0.30 m the far
corners approach the 0.43 m reach. A board that fits in the reach always crosses the circle (radius
about 0.22 m at the push and grasp heights) where the forearm is vertical and joints 4 and 6 line up:
the arm can be there, but it is a wrist singularity, which matters for FR-09 and PR-09.

That is a default, not the answer to LW1's *board placement*: the reachability survey of T-02 is what
decides where the real board goes. To try another pose of the board (the robot's own pose is the one
in `src/xarm_gazebo`):

```bash
ros2 run lite6_checkers spawn_board --x -0.2 --y -0.74 --z 1.025 --yaw -1.5708
```

(Remove the first board before spawning another:
`gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "checkerboard" type: MODEL'`.)

## The board frame

The model origin is the **centre of the playing area, on the playing surface**; the slab is below it.
`x` runs from White's side to Black's side, `y` to White's left (towards the a-file), `z` up, so the
board frame can be laid with the same yaw as a robot facing the board. Square (file `i`, rank `j`),
counted from 0, has its centre at `x = (j − 3.5)·0.03`, `y = (3.5 − i)·0.03`.

The collision surface is one flat box with its top at z = 0: a piece pushed across the board meets no
step. The dark squares are drawn 0.5 mm proud of the light plate, for the renderer only.

## Changing the model

`models/checkerboard/model.sdf` is generated. Change `lite6_checkers/make_board_sdf.py` and regenerate:

```bash
cd ~/dev_ws/src/lite6_checkers
python3 -m lite6_checkers.make_board_sdf > models/checkerboard/model.sdf
```

then `lab build`. The test `test/test_board_sdf.py` fails if the two disagree.

## Using it without `lab`

```bash
ros2 launch xarm_gazebo lite6_beside_table_gazebo.launch.py load_controller:=true show_rviz:=false
ros2 run lite6_checkers spawn_board     # in a second terminal
```
