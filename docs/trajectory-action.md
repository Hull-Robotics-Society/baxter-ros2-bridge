# Whole paths: the joint trajectory action

Motion planners such as MoveIt, and many other ROS 2 programs, do not command an arm
joint by joint. They send a whole path, "be at these joint angles after 3 s, at those
after 5 s", to an *action* called `FollowJointTrajectory`, and wait for the result. The
joint trajectory action server offers that action for Baxter's arms and takes the arm
through the waypoints in one smooth motion, without stopping at each.

For your own Python programs, `Limb.move_through()` ([arms.md](arms.md)) does the same
without a server. Use the action when other ROS 2 software should move the arm.

The server has the modes, defaults and behaviour of the one in the original Baxter SDK
(`joint_trajectory_action_server.py`, SDK 1.2.0). The differences are listed at the end.

## Starting the server

The robot must be enabled. Clear the area and have someone at the e-stop: the arms move
as soon as a program sends a path.

```bash
ros2 run baxter_interface joint_trajectory_action_server                # both arms
ros2 run baxter_interface joint_trajectory_action_server -l left        # one arm
ros2 run baxter_interface joint_trajectory_action_server -m velocity    # another mode
ros2 run baxter_interface joint_trajectory_action_server --curve sdk    # the SDK's curve
```

It runs until Ctrl+C and offers:

```
/robot/limb/left/follow_joint_trajectory      control_msgs/action/FollowJointTrajectory
/robot/limb/right/follow_joint_trajectory
```

Options: `-l` arm (`both`, `left`, `right`), `-m` mode, `--curve` (`smooth`, `sdk`),
`-r` commands per second (100).

## The three modes

| Mode | How the arm is commanded | Notes |
| --- | --- | --- |
| `position` | position commands | The robot's safety layer stays active: speed limit and collision avoidance between the arms. **The default here.** |
| `position_w_id` | "raw" position commands, plus the path's speeds and accelerations for the robot's inverse dynamics (feed-forward) | Follows the path most exactly. **Raw commands bypass the robot's collision avoidance.** The SDK's default. Every path must name all seven joints of the arm. |
| `velocity` | velocity commands from a PID controller per joint | The gains decide how well the arm follows; with the default (`kp` 2, no `ki`, `kd`) it lags behind and ends short of the last waypoint. Tune them with the parameters below. |

## The two curves

Both curves pass through every waypoint at its time. They differ in between:

| | `--curve smooth` (default) | `--curve sdk` |
| --- | --- | --- |
| Where it comes from | this library | the SDK's server (`bezier.py`), unchanged |
| How it is built | piece by piece; a stretch depends only on its two ends | one spline for the whole path; every waypoint influences all of it |
| Start and end | at rest: the arm eases in and out | at speed, with zero acceleration ("natural" ends) |
| At a waypoint, evenly timed | speed continuous; acceleration jumps unless the goal carries accelerations | speed and acceleration continuous |
| At a waypoint, unevenly timed | speed continuous | speed steps (the arithmetic treats all stretches as equally long) |
| Speeds and accelerations in the goal (MoveIt sends them) | shape the path itself | not used for the path; each gets its own spline, passed on only as feed-forward in `position_w_id` mode |

Use `sdk` when the arm should move exactly as it did under the original SDK, for example
to compare with old recordings. Otherwise `smooth` follows a planner's speeds and is
gentler on hand-written paths.

## Sending a path

[examples/12_trajectory_client.py](../examples/12_trajectory_client.py) is a complete
client in plain ROS 2. From the command line, one waypoint (the neutral pose, reached
after 4 seconds):

```bash
ros2 action send_goal /robot/limb/left/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2], points: [{positions: [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0], time_from_start: {sec: 4}}]}}"
```

## What the server does with a path

- It starts from where the arm is and fills in a smooth curve between the waypoints.
  Waypoints with velocities (and accelerations) are followed with those; without them
  the arm starts and ends at rest and passes the waypoints in between without stopping.
- It commands the arm 100 times per second along the curve and reports progress 20 times
  per second (where the arm should be, where it is).
- After the path's end it keeps commanding the last waypoint for `goal_time` seconds,
  then checks the result.
- **Afterwards it keeps holding the arm at the path's end**, until the next path arrives,
  someone takes the arm by its cuff, or the robot is disabled. While the server holds an
  arm, do not move that arm from another program (`Limb.move_...`): the two would send
  commands against each other. Stop the server first.
- A new path for an arm replaces the one that is running.

## Tolerances and gains

Each goal can carry its own tolerances (`path_tolerance`, `goal_tolerance`,
`goal_time_tolerance`); MoveIt sends them. Where a goal carries none, the server's
parameters apply. The SDK used dynamic reconfigure for these; here they are ROS 2
parameters of the server's node, and can be changed while it runs:

```bash
ros2 param list /rsdk_position_joint_trajectory_action_server
ros2 param set  /rsdk_position_joint_trajectory_action_server left_s0_trajectory 0.3
```

(The node is called `rsdk_<mode>_joint_trajectory_action_server`, with `_left` or
`_right` at the end when started for one arm.)

| Parameter | Meaning | `position` | `position_w_id` | `velocity` |
| --- | --- | --- | --- | --- |
| `goal_time` | seconds the arm may be late at the end | 0.1 | 0.1 | 0.0 |
| `stopped_velocity_tolerance` | speed (rad/s) below which the arm counts as stopped at the end | 0.25 | 0.20 | not checked |
| `<joint>_trajectory` | how far (rad) the joint may be from the path on the way | 0.2 | 0.35 | not checked |
| `<joint>_goal` | how far (rad) it may be from the last waypoint at the end | not checked | not checked | not checked |
| `<joint>_kp`, `_ki`, `_kd` | PID gains | | | 2.0, 0, 0 |

A negative value means "not checked". These are the SDK's defaults.

## How a path ends

| What happened | Result |
| --- | --- |
| the arm arrived | succeeded |
| a joint strayed further from the path than its `_trajectory` tolerance, for example because the arm was held back | aborted, `PATH_TOLERANCE_VIOLATED` |
| at the end a joint is further from the last waypoint than its `_goal` tolerance, or the arm is still moving | aborted, `GOAL_TOLERANCE_VIOLATED` |
| the path was cancelled | cancelled |
| a newer path arrived, the robot was disabled (e-stop), or the robot stopped reporting | aborted, with the reason as text |
| the path names a joint the arm does not have, or not all seven in `position_w_id` mode | the path is refused |

## Differences from the SDK's server

- The default mode is `position`; the SDK started in `position_w_id`.
- The default curve between the waypoints is this library's; the SDK's is available
  with `--curve sdk` (see "The two curves").
- A path whose first waypoint lies in the future starts from where the arm is (the SDK
  did this only for paths of one waypoint).
- A path for unknown joints is refused at once (the SDK accepted and then aborted it).
- A lost connection to the robot ends the path with an error.
- Progress is reported 20 times per second (the SDK: 100).
- There is no gripper action and no head action yet.

With the bridge in `fixed_list` mode the server needs the topics listed under "Joint
trajectory action server" in `bridge_config/topics.yaml` (they are there). It does not
check a path against the joint limits or for collisions.

## How closely the arm follows

Measured on robot 1 with the left arm, a path of 5 waypoints over 11 s, the smooth curve
and the bridge in `fixed_list` mode: how far the furthest joint was from the path.

| Mode | on average | at most |
| --- | --- | --- |
| `position` | 0.019 rad | 0.044 rad |
| `position_w_id` | 0.018 rad | 0.042 rad |
| `velocity` | 0.083 rad | 0.249 rad |

## Limitations

- `velocity` mode follows loosely with the SDK's default gains, which have not been
  tuned.
- Not tested yet on the robot (only against a simulated arm): paths with velocities and
  accelerations, the `sdk` curve, a path for some joints only, cancelling, replacing, an
  unknown joint, an arm that is held back, a lost connection, a disabled robot, and
  letting go when the cuff is held.
