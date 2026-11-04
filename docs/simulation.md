# A simulated Baxter

**Work in progress, on the branch `simulator` only.** This guide was written before the
rest of the documentation was reworked for release 1.0.0, and some of it may be out of
date.

Rethink's own simulator runs a Baxter in Gazebo, a physics simulation with a 3D window,
and answers on the same ROS 1 topics and services as the real robot. The bridge connects
to it exactly as it connects to the robot, so the library, the tools, the examples and
your own programs run against it unchanged. Use it to try a program at home before it
moves a real arm.

```
your program (ROS 2)  <->  bridge  <->  simulator (ROS 1, Gazebo)     instead of the robot
```

## Start

Once: copy the settings and build the image (a few minutes; it downloads Gazebo).

```bash
cp .env.sim.example .env.sim          # on WSL2, also remove the "# " before DDS_PROFILE=wsl
docker compose build sim
```

Every time: start the simulator, wait until its log says the controllers are loaded
(about half a minute), then start the bridge.

```bash
docker compose --env-file .env.sim up -d sim
docker compose --env-file .env.sim logs -f sim       # wait for "Loaded 'right_joint_effort_controller'", then Ctrl+C
docker compose --env-file .env.sim up -d bridge
```

Programs are started as usual, with the same `--env-file .env.sim`:

```bash
docker compose --env-file .env.sim run --rm client ros2 run baxter_interface enable_robot -e
docker compose --env-file .env.sim run --rm client python3 examples/03_move_arm.py
```

Stop everything:

```bash
docker compose --env-file .env.sim --profile tools down
```

`--env-file .env.sim` is what makes a command talk to the simulator. Without it, the
same command uses `.env`, the real robot. The two use different ROS 2 domains (43 and
42), so a program started with the wrong file finds nothing instead of the wrong robot.

(`--profile tools` in the last command: without it, `down` stops the bridge and leaves
the simulator running.)

The bridge has to be started after the simulator is ready: in its default mode it only
carries the services that exist at that moment. If inverse kinematics is "not
available", run `docker compose --env-file .env.sim restart bridge`.

## The window

Two windows open on the screen of the machine Docker runs on (on WSL2: on the Windows
desktop), as RViz's does: Gazebo with the robot, and a small one, "Baxter IO", with
buttons to click for the navigators, the cuff buttons and the cuff sensors. Clicking
them is how a simulated button press reaches your program.

If the windows do not open: `xhost +local:docker`, see [rviz.md](rviz.md).

`SIM_WINDOW=false` in `.env.sim` runs the simulator without its windows. Navigators and
cuffs then exist but are never pressed. That is enough for trying arm programs and saves
a third or more of the processor time.

## A VS Code project against the simulator

In the project's `.devcontainer/devcontainer.json`, set `ROS_DOMAIN_ID` to `43` and
rebuild the container. Set it back to `42` for the real robot. With two simulators, also add
`"BAXTER_ROBOT": "sim1"` there for the simulator the project's programs are for.

## Checking it

The robot check runs against the simulator too:

```bash
ENV_FILE=.env.sim bash scripts/robot_check.sh --from bridge
```

The stages that need what the simulator does not have (next section) fail; that is
expected.

## What the simulated robot can and cannot do

| Part | In the simulator |
| --- | --- |
| Enabling, robot state | as on the robot |
| Arms: joint moves, untuck and tuck, neutral pose, `move_by`, `move_through`, `at_once` | work |
| Inverse kinematics | works from the arm's current posture (hand moves); a place far from where the arm is may get "cannot reach" although the real robot would find a way |
| Joint trajectory action server | works in all three modes |
| Head: pan and nod | work |
| Electric grippers | work (calibrate, close, open); always mounted on both arms |
| Navigators, cuff buttons, cuff sensors | exist; pressed by clicking in the "Baxter IO" window |
| Sonar ring, hand range sensors | topics exist; in the empty world nothing is in range |
| Accelerometers | a stand-in that always reports a hand at rest |
| Cameras | the `Camera` class works: list, open, close, two at a time, as on the robot. Pictures are always 640 x 400, whatever size is asked for. About 3 per second without a graphics card, about 25 with one (see "Graphics card") |
| Screen | the topic exists; whether a picture shows on the simulated head has not been tried |
| Head lights, analog I/O overview, calibration and tare | not there |

The simulated hand positions are for arms with grippers mounted, about 14 cm further
along the hand than on a robot without grippers.

## Two simulated robots

Two simulators run next to each other, each a robot of its own in its own world, each
with its own bridge. This is the set-up of [two-robots.md](two-robots.md) with simulators
in place of the robots, and the way to try a program for two robots without them: the
simulated robots get names, and both appear in the same ROS 2 system.

Each simulator needs a settings file of its own:

```bash
cp .env.sim.example .env.sim1
cp .env.sim.example .env.sim2
nano .env.sim1          # then .env.sim2
```

| Setting | `.env.sim1` | `.env.sim2` |
| --- | --- | --- |
| `ROBOT_NAME` (remove the `# `) | `sim1` | `sim2` |
| `COMPOSE_PROJECT_NAME` | `baxter-sim1` | `baxter-sim2` |
| `ROS_MASTER_URI` | `http://127.0.0.1:11311` (as it is) | `http://127.0.0.1:11312` |
| `SIM_GAZEBO_PORT` | `11345` (as it is) | `11346` |
| `ROBOT_POSITION` (remove the `# `) | `0 0 0 0` | for example `0 1.5 0 0`: 1.5 m to the left |
| `ROS_DOMAIN_ID` | `43` (as it is) | `43` (as it is) |

On WSL2 also remove the `# ` before `DDS_PROFILE=wsl` and the `COMPOSE_FILE` line in
both, as for one simulator.

Start both simulators, then, half a minute later, both bridges:

```bash
docker compose --env-file .env.sim1 up -d sim
docker compose --env-file .env.sim2 up -d sim
docker compose --env-file .env.sim1 up -d bridge
docker compose --env-file .env.sim2 up -d bridge
```

In a program the robots are called by their names:

```python
import baxter_interface

first = baxter_interface.Robot('sim1')
second = baxter_interface.Robot('sim2')
```

```bash
docker compose --env-file .env.sim1 run --rm client python3 examples/15_two_robots.py sim1 sim2
```

A program for one robot runs on the simulator whose file it is started with, and both
robots in one RViz picture, each at its `ROBOT_POSITION`:

```bash
docker compose --env-file .env.sim2 run --rm client python3 examples/02_move_arm.py
docker compose --env-file .env.sim1 run --rm rviz ros2 launch baxter_description view_robots.launch.py
```

Stop each with its own `--env-file ... --profile tools down`.

Two simulators are twice the work. On the laptop measured below, with the graphics card
and without windows, each ran at about 70 % of real time while the bridges carried all
four hand cameras. Without the graphics card, expect much less.

The two robots are in separate worlds: they cannot see or touch each other, although
RViz draws them next to each other.

## Simulation time

On a busy computer the simulation runs slower than real time: at 0.5, one second of
simulated motion takes two seconds on the clock. A program that times an arm motion by
the computer's clock then finds the simulated arm far behind, and stops the motion. Seen
with the joint trajectory action server (and so with MoveIt), whose paths ended with
"the arm is still moving at the end".

The answer is the usual one for Gazebo: programs that time motions follow the
simulation's own clock instead. `.env.sim.example` switches it on:

```
SIM_TIME=1
```

With it:

- the bridge also carries the simulation's clock, as `/clock` (with a robot name in
  front if the simulated robot has one); the simulator sends it 100 times per second;
- the joint trajectory action server times paths by it, and MoveIt runs on it. A slow
  simulation then takes a motion more slowly, and nothing is stopped.

Everything else in the library keeps the computer's clock, which only means that its
time limits (waiting for an arm to arrive, for example) are generous in a slow
simulation.

An older `.env.sim` has no `SIM_TIME` line: add it. Never use it with a real robot: the
robot has no simulation clock, and the action server would wait for one and refuse to
start ("no simulation clock arrives").

## Graphics card

By default everything is drawn on the processor, which works everywhere and is slow.
On WSL2 the simulator and RViz can use the computer's graphics card instead, also a
laptop's built-in one. Add this line to `.env.sim` (and to `.env`, for RViz with the
real robot):

```
COMPOSE_FILE=compose.yaml:compose.wsl-gpu.yaml
```

and start the simulator again. Nothing has to be installed. `compose.wsl-gpu.yaml`
explains how it works. It is for WSL2 only: on another machine the containers would not
start with it.

Measured on a laptop with an Intel i7-12700H (14 cores) and its built-in Iris Xe
graphics, both hand cameras streaming:

| | drawn on the processor | with the graphics card |
| --- | --- | --- |
| Pictures per second from each camera, no windows | 2.6 to 2.9 | 25 |
| ... with the windows open | not measured | 15 |
| Processor cores busy, cameras streaming, no windows | about 6 | about 2.5 |
| Speed of the simulation, cameras not watched, no windows | 100 % of real time | 100 % |
| ... cameras streaming, no windows | 97 % | 89 % |
| ... cameras streaming, windows open | 91 % | 71 % |

The graphics card itself costs no speed. The simulation is slower with it while cameras
stream because the cameras then do ten times the work: 50 pictures per second instead
of 5 are copied back from the graphics card, published and carried through the bridge,
and the simulation waits for its cameras at every picture. At 71 %, a move that takes
5 s on the robot takes 7 s of your time. Programs are not affected, they follow the
simulated robot; but close the windows (`SIM_WINDOW=false`), and the cameras a program
does not need, when speed matters.

## How it is built

`docker/sim-noetic/` builds Rethink's `baxter_simulator` (branch `kinetic-devel`, the
newest) and `baxter_common` v1.2.0 from their GitHub repositories, at fixed commits, on
ROS 1 Noetic with Gazebo 11. The simulator was last released for ROS Kinetic (2016);
`docker/sim-noetic/patches/` holds the changes that were needed, each with its reason at
the top of the file. Besides two that only make it compile, they fix faults that show
with current compilers and libraries: the program emulating the robot's interface
crashed at start, inverse kinematics answered every request with "no solution", and
gravity was not switched off for the arms as the simulator intends, so that a shoulder
could not lift its arm.

`baxter_sim.launch` and `sim_extras.py` in the same folder are this project's additions:
the launch file keeps the "Baxter IO" program running also without windows, and the
helper adds what the robot has and the simulator lacks. That is the stand-in
accelerometers and the camera services: a patch makes the simulated cameras publish
under `/sim/cameras/...`, and the helper passes on only the cameras that are "open". A
closed camera is not drawn at all.

## Status

Simulation time (14 Oct 2026): with the computer kept busy on purpose, the simulation ran
at 0.43 to 0.64 of real time, and 3 of 5 first MoveIt motions were stopped by the action
server. With simulation time, at 0.72 to 0.77 of real time, 4 of 4 passed, each motion
taking longer than planned (about 6 s for 3.85 s) as intended, and the robot check's arm
stage passed completely at 0.83. The two did not run at exactly the same load; a run
side by side is still to be done.


New on 8 Oct 2026. Run in WSL2 without windows: the stages bridge, read, enable, arms
and cameras of `robot_check.sh` pass except two checks (the analog I/O overview, and
the inverse kinematics test for a far-away place); the head and gripper stages passed
before the last changes. With windows, both opened on the Windows desktop and the
navigators appeared; clicking the buttons was not tried. With the graphics card: the
numbers above; RViz started with it next to the simulator, its window was not looked
at.

Two simulators side by side (10 Oct 2026, with the graphics card, no windows), named
`sim1` and `sim2` as described above: one program enabled each separately, moved four
arms and two heads at the same time, moved hands by inverse kinematics on each, closed
one gripper without the other and took a camera picture; example 15 ran with the two
names; RViz's start file for all robots published the frames of both, the second 1.5 m
to the left as set; the robot check's stages read, enable, arms, head, grippers and
cameras ran on `sim2` while `sim1` ran next to it, 32 checks passed and 1 failed (the
analog I/O overview). Each simulation ran at about 70 % of real time. After these
changes the single simulator without a name was checked again: 37 passed, the same 2
as before failed. In all of this the bridge and RViz images were not rebuilt; the
changed files were put into the existing images for the test.

Not tried: a VS Code project against the simulator, two simulators with their windows
in this version, the Mobility Base.
