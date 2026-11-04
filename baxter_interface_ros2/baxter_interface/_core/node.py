"""The ROS 2 node that all baxter_interface classes share, and how threads are used.

Why this exists: in ROS 1, `rospy.init_node()` created one node per program and processed
incoming messages in background threads automatically. The original SDK relied on that:
`Limb('left').joint_angles()` just returns the latest values, without the caller doing
anything to receive them.

ROS 2 has no such automatic background processing: a node only receives messages while an
executor "spins" it. To keep the original, simple programming style, this module creates
one node the first time any class needs it and spins it in a background thread for the
rest of the program. Users never have to call rclpy themselves.

Threads in a program that uses baxter_interface:

    your program          the main thread: your own code
    receiving thread      one, shared: takes every message from ROS 2 and stores the
                          newest values in the classes (Limb, Gripper, ...). It only ever
                          copies values, so it is always fast.
    callback threads      one for every function you connect to a signal (see
                          callbacks.py). Your functions run there, never in the receiving
                          thread, so they may take long, move the robot or wait for it.

What makes this safe:
    - Values shared between threads are guarded by locks inside the classes.
    - Blocking commands to the same robot part (one arm, the head, one gripper, one
      camera) run one after another, even when called from different threads.
    - An error while receiving a message is printed, and receiving continues. Without
      that, one bad message would silently stop all updates.

Programs that already have their own node (for example inside a larger ROS 2 application)
can pass it to the classes instead: Limb('left', node=my_node). They are then responsible
for spinning it.

Which robot: the classes use names without a leading slash ('robot/joint_states'), so a
name is completed with the namespace of the node, as everywhere in ROS 2. A robot whose
bridge gives it a name appears under that name (docs/two-robots.md):

    node in namespace /       'robot/joint_states' is /robot/joint_states       (no name)
    node in namespace /one    'robot/joint_states' is /one/robot/joint_states   (robot "one")

The shared node's namespace is set when the program is started: with the ROS 2 argument
`--ros-args -r __ns:=/one`, or with the variable BAXTER_ROBOT=one (the argument wins).
A program for several robots gets one more node per robot from robot_node(); they all
share the one receiving thread.
"""

from __future__ import annotations

import atexit
import os
import re
import sys
import threading
import time
import traceback

import rclpy
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import ok as ros_is_running
from rclpy.utilities import try_shutdown

from . import callbacks

# The variable that names the robot a program is for (see the module text).
ROBOT_VARIABLE = 'BAXTER_ROBOT'

_lock = threading.Lock()
_node: Node | None = None
_executor: Executor | None = None
_thread: threading.Thread | None = None
_stop = threading.Event()
_registered = False
# The nodes for further robots, by namespace (see robot_node()).
_robot_nodes: dict[str, Node] = {}


def robot_namespace(name: str) -> str:
    """Return the ROS 2 namespace of the robot with this name.

    Parameters:
        name: the robot's name, e.g. 'one'; '' for the robot without a name

    Returns:
        '/one' for 'one', '/' for ''

    Raises:
        ValueError for a name that cannot be part of a ROS 2 name.
    """
    if name == '':
        return '/'
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9]*(_[A-Za-z0-9]+)*', name):
        raise ValueError(
            f'{name!r} cannot be a robot name: it becomes part of ROS 2 topic names, so only '
            'letters, digits and single underscores are possible, starting with a letter '
            "(for example 'one' or 'baxter_2').")
    return '/' + name


def _receive(executor: Executor, stop: threading.Event) -> None:
    """The receiving thread: process incoming messages until told to stop."""
    while not stop.is_set() and ros_is_running():
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception:
            if stop.is_set() or not ros_is_running():
                break  # errors while shutting down are expected
            print('baxter_interface: error while receiving a message (receiving continues):',
                  file=sys.stderr)
            traceback.print_exc()
            time.sleep(0.01)  # don't spin at full speed if the error repeats


def _warn_about_middleware() -> None:
    """Warn when this program's ROS 2 middleware is not the bridge's (Fast DDS).

    Topics work between Fast DDS and other middlewares, services and actions do not: every
    call then times out with "No answer from ...". The lab machines' shells use Cyclone
    DDS (docs/lab-machines.md).
    """
    identifier = rclpy.utilities.get_rmw_implementation_identifier()
    if not identifier.startswith('rmw_fastrtps'):
        print(
            f'baxter_interface: this program uses the ROS 2 middleware {identifier}, the '
            'bridge uses Fast DDS. Reading works, but every service call (the hand to a '
            'place, cameras, ...) will time out. In this terminal: '
            'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp (baxter.sh does it).',
            file=sys.stderr
        )


def get_node(node: Node | None = None) -> Node:
    """Return `node` if given, otherwise the shared background node (created on first use)."""
    global _node, _executor, _thread, _registered
    if node is not None:
        return node
    with _lock:
        if _node is None:
            # Before anything is started: a wrong name must not leave half of it running.
            namespace = robot_namespace(os.environ.get(ROBOT_VARIABLE, '').strip())
            if not ros_is_running():
                rclpy.init()
            _warn_about_middleware()
            _stop.clear()
            callbacks.resume()
            # The process id keeps names unique when several programs run at once.
            # The namespace given here is the one from BAXTER_ROBOT; a namespace argument
            # at the program's start (--ros-args -r __ns:=/one) replaces it, as it does
            # for every ROS 2 node.
            _node = rclpy.create_node(
                f'baxter_interface_{os.getpid()}',
                namespace=namespace
            )
            # Single-threaded on purpose. The receiving thread only copies values; user
            # callbacks get their own threads (callbacks.py). A MultiThreadedExecutor would
            # add nothing, and its thread pool is closed by Python at exit before atexit
            # handlers run, which caused a "cannot schedule new futures" error on every exit.
            _executor = SingleThreadedExecutor()
            _executor.add_node(_node)
            _thread = threading.Thread(
                target=_receive,
                args=(_executor, _stop),
                name='baxter-receiving',
                daemon=True
            )
            _thread.start()
            if not _registered:
                atexit.register(shutdown)
                _registered = True
    return _node


def robot_node(name: str) -> Node:
    """Return the background node for the robot with this name (created on first use).

    For the robot the program was started for, that is the shared node of get_node(), so
    that Robot('one').Limb('left') and a plain Limb('left') are the same arm with the same
    lock. Every other robot gets a node of its own in its namespace; all of them are
    served by the one receiving thread.

    Parameters:
        name: the robot's name, e.g. 'one'; '' for the robot without a name

    Returns:
        the node; the same one for every call with the same name
    """
    namespace = robot_namespace(name)
    shared = get_node()
    if shared.get_namespace() == namespace:
        return shared
    with _lock:
        if namespace not in _robot_nodes:
            # use_global_arguments=False: a namespace argument at the program's start is
            # meant for the shared node. This node would otherwise be moved there too.
            node = rclpy.create_node(
                f'baxter_interface_{os.getpid()}',
                namespace=namespace,
                use_global_arguments=False
            )
            assert _executor is not None
            _executor.add_node(node)
            _robot_nodes[namespace] = node
        return _robot_nodes[namespace]


def shutdown() -> None:
    """Stop the background nodes. Called automatically when the program exits.

    Order matters for a quiet exit: first the callback threads are told to stop (from then
    on a callback that is still running fails silently), then the receiving thread ends
    its loop by itself, and only then are the executor and the nodes taken away.
    """
    global _node, _executor, _thread
    with _lock:
        callbacks.stop_all()
        _stop.set()
        if _thread is not None and _thread is not threading.current_thread():
            _thread.join(timeout=2.0)  # let the receiving loop finish before the nodes go away
        if _executor is not None:
            _executor.shutdown()
        for node in list(_robot_nodes.values()) + [_node]:
            if node is not None:
                node.destroy_node()
        _robot_nodes.clear()
        _node = _executor = _thread = None
    if ros_is_running():
        try_shutdown()
