"""Tests that a program using the library closes without error messages.

Needs ROS 2 (rclpy) but no robot and no bridge. Each test starts a small program of its
own, lets it end, and checks what it printed: Python reports problems at exit (such as the
earlier "cannot schedule new futures after interpreter shutdown") on the error output,
while the exit code stays 0, so they are easy to miss.
"""

import os
import signal
import subprocess
import sys

import pytest

pytest.importorskip('rclpy')

# What must not appear on the error output of a program that ended normally.
ERROR_SIGNS = (
    'Traceback',
    'Exception ignored',
    'Error in callback',
    'error while receiving',
    'cannot schedule new futures',
    'Fatal Python error',
)

START = '''
import atexit
import threading
import time

from baxter_interface import Signal, shutdown
from baxter_interface._core.dataflow import wait_for
from baxter_interface._core.node import get_node

# Registered before the library's own exit handler, so it runs after it: the program
# lingers for a moment once the library has shut down. A connected function that is still
# running then gets the time to stumble over the closed ROS 2, and to complain if it would.
atexit.register(time.sleep, 0.5)

node = get_node()
signal = Signal()
started = threading.Event()
'''

PROGRAMS = {
    'nothing but the shared node': '''
print('end of program')
''',

    'a connected function that is idle': '''
signal.connect(lambda: started.set())
signal()
assert started.wait(5)
time.sleep(0.2)
print('end of program')
''',

    # Like a button callback that is still inside limb.move_to_joint_positions().
    'a connected function that is still waiting for the robot': '''
def wait_for_the_robot():
    started.set()
    wait_for(lambda: False, timeout=-1)

signal.connect(wait_for_the_robot)
signal()
assert started.wait(5)
print('end of program')
''',

    'a connected function that is still sending commands': '''
from std_msgs.msg import Bool

publisher = node.create_publisher(Bool, '/baxter_interface_test/clean_exit', 10)

def send_commands():
    started.set()
    wait_for(lambda: False, timeout=-1, rate=1000, body=lambda: publisher.publish(Bool(data=True)))

signal.connect(send_commands)
signal()
assert started.wait(5)
time.sleep(0.2)
print('end of program')
''',

    'messages still arriving': '''
from std_msgs.msg import Bool

received = []
publisher = node.create_publisher(Bool, '/baxter_interface_test/clean_exit', 10)
subscription = node.create_subscription(Bool, '/baxter_interface_test/clean_exit', received.append, 10)

def send():
    started.set()
    wait_for(lambda: False, timeout=-1, rate=1000, body=lambda: publisher.publish(Bool(data=True)))

signal.connect(send)
signal()
assert started.wait(5)
wait_for(lambda: len(received) > 10, timeout=10)
print('end of program')
''',

    'shutdown() called by the program itself': '''
signal.connect(lambda: started.set())
signal()
assert started.wait(5)
shutdown()
shutdown()
print('end of program')
''',
}


def _errors_in(result):
    return [sign for sign in ERROR_SIGNS if sign in result.stderr]


@pytest.mark.parametrize('name', list(PROGRAMS))
def test_program_closes_without_errors(name):
    result = subprocess.run(
        [sys.executable, '-c', START + PROGRAMS[name]],
        capture_output=True,
        text=True,
        timeout=60
    )
    report = f'\n--- output ---\n{result.stdout}\n--- error output ---\n{result.stderr}'

    assert 'end of program' in result.stdout, report
    assert result.returncode == 0, report
    assert _errors_in(result) == [], report


@pytest.mark.skipif(os.name == 'nt', reason='Ctrl+C cannot be sent to a program this way on Windows')
def test_program_closes_without_errors_after_ctrl_c():
    program = '''
import signal
signal.signal(signal.SIGINT, signal.default_int_handler)  # in case it is switched off here
''' + START + '''
try:
    # 'ready' is printed inside the try: the test sends Ctrl+C as soon as it reads it,
    # and the interrupt must not arrive before the program is prepared to catch it.
    print('ready', flush=True)
    time.sleep(30)
    print('no Ctrl+C arrived')
except KeyboardInterrupt:
    print('interrupted')
'''
    process = subprocess.Popen(
        [sys.executable, '-c', program],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    try:
        assert process.stdout.readline().strip() == 'ready'
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=20)
    finally:
        process.kill()
    report = f'\n--- output ---\n{stdout}\n--- error output ---\n{stderr}'

    assert 'interrupted' in stdout, report
    assert [sign for sign in ERROR_SIGNS if sign in stderr] == [], report
