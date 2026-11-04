# Examples

Short programs with comments, one per part of the robot. Read them from 01 upwards; each
is meant to be copied and changed.

Run one with the bridge running, in a terminal set up with `baxter.sh`
(see [docs/quickstart.md](../docs/quickstart.md)):

```bash
python3 examples/01_read_robot.py
```

Without ROS 2 installed, in the `client` container:
`docker compose run --rm client python3 examples/01_read_robot.py`.

| Example | Shows | Moves |
| --- | --- | --- |
| [01_read_robot.py](01_read_robot.py) | `setup()`, robot state, joint angles, where the hands are | nothing |
| [02_move_arm.py](02_move_arm.py) | enabling, neutral pose, moves by joint angles, speed, a move that fails | **an arm** |
| [03_move_hand.py](03_move_hand.py) | the hand to a place: `move_by`, `move_to_pose`, `solve_pose` | **an arm** |
| [04_head.py](04_head.py) | turning the head, nodding | the head |
| [05_screen.py](05_screen.py) | pictures on the screen, drawn with OpenCV or from a file | nothing |
| [06_buttons_and_lights.py](06_buttons_and_lights.py) | events from buttons and the wheel, lights | nothing |
| [07_camera.py](07_camera.py) | a photo, the live picture on the screen, the head camera | nothing |
| [08_gripper.py](08_gripper.py) | calibrate, open, close, "did it grip something?" | a gripper |
| [09_sensors.py](09_sensors.py) | sonar, hand distance sensor, accelerometer | nothing |
| [10_wave.py](10_wave.py) | a control loop of your own, 100 targets per second | **an arm, widely** |
| [11_teach_and_repeat.py](11_teach_and_repeat.py) | show postures by hand; the arm repeats them, first one by one, then in one motion (`move_through`) | **an arm** |
| [12_trajectory_client.py](12_trajectory_client.py) | a whole path through the joint trajectory action, in plain ROS 2 | **an arm** |
| [13_at_once.py](13_at_once.py) | both arms and the head at the same time: `at_once()` | **both arms**, the head |
| [14_head_lights.py](14_head_lights.py) | the halo's colour, the yellow sonar lights | nothing |
| [17_moveit.py](17_moveit.py) | a motion planned by MoveIt around a wall in the way; needs MoveIt running ([docs/moveit.md](../docs/moveit.md)) | **an arm** |
| [15_two_robots.py](15_two_robots.py) | two robots in one program: `Robot('one')`, `at_once()` across robots ([docs/two-robots.md](../docs/two-robots.md)) | **the left arm of both robots** |
| [16_mobility_base.py](16_mobility_base.py) | the Mobility Base: mode, battery, bumpers; with `drive`, 10 cm forward and back ([docs/mobility-base.md](../docs/mobility-base.md)) | nothing; with `drive`, **the base** |

[project-template/](project-template/) is not an example to run but a starting point
for a project of your own; see [docs/own-project.md](../docs/own-project.md).

Examples that take an arm use the left one; add `right` for the other
(`python3 examples/02_move_arm.py right`).

Whenever something moves: someone at the e-stop, nobody within reach of the arms. Run an
example for the first time slowly, with a hand on the e-stop.
