"""The individual smoke tests (one class each), run by smoke_test.py.

ROS 2 version of baxter_tools' smoketests.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE). Same tests and same movements.

Each test needs its topics and services in bridge_config/topics.yaml:
    Messages   /robot/analog_io_states
    Services   /ExternalTools/left/PositionKinematicsNode/IKService
    Head       the head topics, /robot/xdisplay
    Grippers   the gripper topics
    BlinkLEDs  the navigator light topics
    Cameras    the camera services, /robot/xdisplay and /cameras/<name>/image for all
               three cameras
"""

import os
import queue
import threading
import time
import traceback

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from baxter_core_msgs.msg import AnalogIOStates, EndEffectorState
from baxter_core_msgs.srv import SolvePositionIK

import baxter_interface
from baxter_interface._core import dataflow
from baxter_interface._core.node import get_node
from baxter_interface._core.services import call_service
from baxter_interface.camera import cameras_in_the_way

_IMAGE_DIR = os.path.join(os.path.dirname(baxter_interface.__file__), 'images')
_WORKING_IMAGE = os.path.join(_IMAGE_DIR, 'baxterworking.png')
_SDK_IMAGE = os.path.join(_IMAGE_DIR, 'researchsdk.png')


class SmokeTest:
    """Smoke tests base class."""

    def __init__(self, name):
        self._name = name
        self._node = get_node()
        self._rs = baxter_interface.RobotEnable()
        self.result = [False, '']

    def start_test(self):
        """Run the test. The result is stored as [success, 'stack trace if failed']."""
        raise NotImplementedError()

    def finish_test(self, filename):
        """Append the result to the output file."""
        outcome = 'Success' if self.result[0] else 'Failure'
        with open(filename, 'a') as testlog:
            testlog.write(f'Test: {self._name} ------ Result: {outcome}\n')
            if not self.result[0]:
                testlog.write(f"{'*' * 40}\n{self.result[1]}{'*' * 40}\n")
        print(f'Results of {self._name} saved to file: {filename}')
        print(f'------- Result: {outcome} -------\n\n')

    def return_failure(self, trace):
        """Record a failure and try to disable the robot."""
        self.result[0] = False
        self.result[1] = trace
        try:
            self._rs.disable()
        except Exception:
            pass


class Enable(SmokeTest):
    """Verify ability to enable, check state and disable Baxter."""

    def __init__(self, name='Enable'):
        super().__init__(name)

    def start_test(self):
        try:
            print('Test: Enable - Zero G for 10 Seconds')
            self._rs.enable()
            # Allow the user to test zero-G mode (hold a cuff and move the arm)
            time.sleep(10.0)
            print('Test: State')
            print(self._rs.state())
            print('Test: Disable')
            self._rs.disable()
            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class Messages(SmokeTest):
    """Verify messages being published and ability to subscribe."""

    def __init__(self, name='Messages'):
        super().__init__(name)

    def start_test(self):
        try:
            topic = 'robot/joint_states'
            print(f'Test: Subscribe to topic: {topic}')
            dataflow.wait_for_message(self._node, JointState, topic, 5.0)

            topic = 'robot/analog_io_states'
            print(f'Test: Subscribe to topic: {topic}')
            dataflow.wait_for_message(self._node, AnalogIOStates, topic, 5.0)

            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class Services(SmokeTest):
    """Verify services available and ability to make calls as client."""

    def __init__(self, name='Services'):
        super().__init__(name)

    def start_test(self):
        try:
            srv = 'ExternalTools/left/PositionKinematicsNode/IKService'
            print(f'Test: Service availability: {srv}')
            client = self._node.create_client(SolvePositionIK, srv)
            if not client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError(f'Service {srv} not available')

            print('Test: IK service call Solve IK')
            pose = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id='base'
                ),
                pose=Pose(
                    position=Point(
                        x=0.657579481614,
                        y=0.851981417433,
                        z=0.0388352386502
                    ),
                    orientation=Quaternion(
                        x=-0.366894936773,
                        y=0.885980397775,
                        z=0.108155782462,
                        w=0.262162481772
                    )
                )
            )
            request = SolvePositionIK.Request(pose_stamp=[pose])
            call_service(client, request, timeout=5.0)

            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class Head(SmokeTest):
    """Move the head pan and nod, display an image on the screen."""

    def __init__(self, name='Head'):
        super().__init__(name)

    def start_test(self):
        try:
            print('Enabling robot...')
            self._rs.enable()

            print('Test: Moving Head to Neutral Location')
            head = baxter_interface.Head()
            head.set_pan(0.0, 5)
            print('Test: Pan Head')
            head.set_pan(1.0, 5)
            head.set_pan(-1.0, 5)
            head.set_pan(0.0, 5)
            print('Test: Nod Head')
            for _ in range(3):
                head.command_nod()

            print('Test: Display Image on Screen - 5 seconds')
            display = baxter_interface.Display()
            display.show(_WORKING_IMAGE)
            time.sleep(5.0)
            display.show(_SDK_IMAGE)
            time.sleep(0.5)

            print('Disabling robot...')
            self._rs.disable()
            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class MoveArms(SmokeTest):
    """Move both arms to numerous joint positions."""

    def __init__(self, name='MoveArms'):
        super().__init__(name)

    def start_test(self):
        def move_thread(limb, angles, result_queue, timeout=15.0):
            """Move one arm in its own thread, so both arms move at the same time."""
            try:
                limb.move_to_joint_positions(angles, timeout)
                result_queue.put(None)
            except Exception as exception:
                result_queue.put(exception)

        try:
            print('Enabling robot...')
            self._rs.enable()
            print('Test: Create Limb Instances')
            right = baxter_interface.Limb('right')
            left = baxter_interface.Limb('left')
            left_queue = queue.Queue()
            right_queue = queue.Queue()

            # Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
            #     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
            # Min Joint Range (s0, s1, e0, e1, w0, w1, w2)
            #     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
            joint_moves = (
                [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
                [0.5, -0.8, 2.8, 0.15, 0.0, 1.9, 2.8],
                [-0.1, -0.8, -1.0, 2.5, 0.0, -1.4, -2.8],
                [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
            )
            for move in joint_moves:
                print('Test: Moving to Joint Positions: ' + ', '.join(f'{x:.2f}' for x in move))
                left_thread = threading.Thread(
                    target=move_thread,
                    args=(left, dict(zip(left.joint_names(), move)), left_queue),
                    daemon=True
                )
                right_thread = threading.Thread(
                    target=move_thread,
                    args=(right, dict(zip(right.joint_names(), move)), right_queue),
                    daemon=True
                )
                left_thread.start()
                right_thread.start()
                dataflow.wait_for(
                    lambda: not (left_thread.is_alive() or right_thread.is_alive()),
                    timeout=20.0,
                    timeout_msg='Timeout while waiting for arm move threads to finish',
                    rate=10,
                )
                left_thread.join()
                right_thread.join()
                for result_queue in (left_queue, right_queue):
                    error = result_queue.get()
                    if error is not None:
                        raise error
                time.sleep(1.0)

            print('Disabling robot...')
            self._rs.disable()
            self.result[0] = True
            time.sleep(5.0)
        except Exception:
            self.return_failure(traceback.format_exc())


class Grippers(SmokeTest):
    """Calibrate and move the grippers using position and velocity control."""

    def __init__(self, name='Grippers'):
        super().__init__(name)

    def start_test(self):
        try:
            print('Enabling robot, Moving to Neutral Location...')
            self._rs.enable()
            for name in ['left', 'right']:
                side = name.capitalize()
                limb = baxter_interface.Limb(name)
                gripper = baxter_interface.Gripper(name)
                limb.move_to_neutral()
                time.sleep(2.0)

                print(f'Test: Verify {side} Gripper Type')
                if gripper.type() != 'electric':
                    raise RuntimeError('Test Requires Two Electric Grippers')
                dataflow.wait_for_message(
                    self._node,
                    EndEffectorState,
                    f'robot/end_effector/{name}_gripper/state',
                    5.0
                )

                print(f'Test: Reboot {side} Gripper')
                gripper.reboot()
                print(f'Test: Calibrating {side} Gripper')
                gripper.calibrate()
                print(f'Test: Close {side} Gripper')
                gripper.close(True)
                print(f'Test: Open {side} Gripper')
                gripper.open(True)
                print(f'Test: Close {side} Gripper')
                gripper.close(True)
                print(f'Test: Open {side} Gripper')
                gripper.open(True)

                print(f'Test: {side} Gripper Position Moves')
                gripper.command_position(50.0, True)
                gripper.command_position(75.0, True)
                gripper.command_position(0.0, True)
                gripper.command_position(100.0, True)

                print(f'Test: {side} Gripper Velocity Moves')
                gripper.set_moving_force(100.0)
                gripper.set_velocity(50.0)
                gripper.close(True)
                gripper.set_velocity(25.0)
                gripper.open(True)
                gripper.set_velocity(100.0)
                gripper.close(True)
                gripper.open(True)
                gripper.set_velocity(50.0)
                gripper.set_moving_force(30.0)
                gripper.close(True)
                gripper.open(True)

            print('Disabling robot...')
            self._rs.disable()
            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class BlinkLEDs(SmokeTest):
    """Blink the navigator LEDs."""

    def __init__(self, name='BlinkLEDs'):
        super().__init__(name)

    def start_test(self):
        def blink(io):
            """Toggle a light a few times."""
            for i in range(9):
                io.set_output(i % 2 == 1)
                time.sleep(0.1)

        try:
            itb_names = [
                f'{location}_{which}_light'
                for location in ('left', 'torso_left', 'right', 'torso_right')
                for which in ('outer', 'inner')
            ]
            for itb in itb_names:
                print(f'Test: Blink {itb}')
                blink(baxter_interface.DigitalIO(itb))
            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())


class Cameras(SmokeTest):
    """Verify camera publishing and visualization."""

    def __init__(self, name='Cameras'):
        super().__init__(name)

    def start_test(self):
        camera_names = (
            'left_hand_camera',
            'right_hand_camera',
            'head_camera',
        )

        def power_from(camera):
            """Name a camera to close so that `camera` can be opened, or None if none has to."""
            in_the_way = cameras_in_the_way(camera.name)
            return in_the_way[0] if in_the_way else None

        def show_on_screen(camera):
            """Open the camera and show its pictures on the screen for 10 seconds."""
            print(f'Test: Opening {camera.name}')
            camera.open(
                resolution=(960, 600),
                replace=power_from(camera)
            )
            print(f'Test: Pictures arrive from {camera.name}')
            camera.frame(timeout=10.0)  # raises TimeoutError if none arrives
            print(f'Test: Display {camera.name} to Screen - 10 Seconds')
            camera.frame_received.connect(display.show)
            time.sleep(10.0)
            camera.frame_received.disconnect(display.show)
            camera.close()
            time.sleep(0.5)  # let a picture that is still on its way pass first
            display.show(_SDK_IMAGE)

        def reset_defaults(cameras):
            """Turn the left and right hand cameras back on with default settings."""
            print('Restarting the Default Cameras...')
            for camera in cameras[:2]:
                camera.open(
                    resolution=(640, 400),
                    fps=25,
                    replace=power_from(camera)
                )

        try:
            print('Enabling robot...')
            self._rs.enable()
            print('Test: Left_Hand, Right_Hand, and Head Cameras, one after the other')
            display = baxter_interface.Display()
            cameras = [baxter_interface.Camera(name) for name in camera_names]
            # Only two cameras have power at a time. show_on_screen() takes the power for
            # each camera from another one where needed.
            for camera in cameras:
                show_on_screen(camera)
            reset_defaults(cameras)

            print('Disabling robot...')
            self._rs.disable()
            self.result[0] = True
        except Exception:
            self.return_failure(traceback.format_exc())
