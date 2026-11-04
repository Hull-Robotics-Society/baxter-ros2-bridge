"""Tolerances used when waiting for the robot to reach a commanded position.

The values are the original SDK's: 0.5 degrees for joints, 8 degrees for the head pan
(both in radians here).
"""

JOINT_ANGLE_TOLERANCE: float = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE: float = 0.1396263401

# How far (rad) a joint may be from where its path wants it during Limb.move_through()
# before the move is stopped: the arm is being held back, or cannot keep up.
PATH_ERROR_LIMIT: float = 0.35

# Seconds to wait for the connection through the bridge: for the first state of a robot
# part when one of the classes is created, and for the bridge to listen before a command
# is sent. The bridge connects a topic a second or two after a program starts using it.
CONNECT_TIMEOUT: float = 10.0

# Seconds between the moment the bridge starts listening to a publisher of this program
# and the first command that is sent only once. The bridge listening is half the way: the
# robot then still has to connect to the bridge, and a command sent before that is lost
# without a trace. Waited once per publisher.
BRIDGE_SETTLE_TIME: float = 0.5

# Seconds without a new state from the robot after which a command that is waiting for
# the robot gives up (the robot sends its state 100 times per second). A longer silence
# means the connection to the robot is lost: bridge stopped, cable out, robot off.
STATE_TIMEOUT: float = 1.0

# MoveIt (planning.py): the share of the top speed and of the top acceleration a planned
# motion uses when a move does not say otherwise, from 0 to 1. 1.0 is everything
# moveit_config/config/joint_limits.yaml allows (3.0 rad/s and 3.0 rad/s^2, Rethink's
# values). 0.1: on the real robot with grippers (21 Oct 2026) only 0.1 ran cleanly; at 0.3
# and 0.5 the elbow fell 0.2 rad behind and the motion was stopped (docs/moveit.md).
PLANNING_SPEED: float = 0.1
PLANNING_ACCELERATION: float = 0.1

# The Mobility Base (mobility_base.py). The library never asks the base for more than
# this, whatever a program passes: m/s forward and sideways, rad/s turning. The base
# itself can do 1.5 m/s and 2.1 rad/s; these are walking pace and below.
BASE_MAX_SPEED: float = 0.3
BASE_MAX_TURN: float = 0.5
# Speed commands per second while drive() runs. The base stops when it has heard nothing
# for a fifth of a second, so this must stay well above 5.
BASE_COMMAND_RATE: float = 20.0
# Seconds after the first command by which the base must report that it follows the
# program; otherwise drive() gives up and says in which mode the base is.
BASE_FOLLOW_TIME: float = 1.5
