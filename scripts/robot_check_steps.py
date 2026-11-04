#!/usr/bin/env python3
"""The individual steps of the robot check (scripts/robot_check.sh runs them one by one).

Each step is a small program of its own:

    python3 robot_check_steps.py <step>

It prints what it finds and ends with exit code 0 (worked) or 1 (did not). The steps are
also short examples of how the library is used. Steps that only read are named read_...;
all others command the robot.
"""

import sys
import threading
import time
import traceback

OUT = '/out'  # robot_check.sh mounts its log folder here (pictures are saved into it)

NAVIGATORS = ('left', 'right', 'torso_left', 'torso_right')


# --- Reading only ---------------------------------------------------------------------------

def read_version():
    """Which software the robot reports, and whether that is what the bridge is built for."""
    import baxter_interface
    from baxter_interface.tools.robot_info import BUILT_FOR

    version = baxter_interface.software_version()
    print(f'robot software version: {version}')
    print(f'bridge and library are built for: {BUILT_FOR}')
    if version is None:
        print('NOTE: the robot does not report a version (no /rethink/software_version); '
              'the checksum comparison of the inventory decides.')
    elif not version.startswith(BUILT_FOR):
        raise RuntimeError(
            f'Version mismatch: the robot runs {version}, the bridge and library are built '
            f'for {BUILT_FOR}. See msgs/README.md.')


def read_connect_time():
    """How long the parts of a typical program take to connect, created in one setup() block.

    The number shows whether settings.CONNECT_TIMEOUT (how long a part may wait at start)
    is chosen well for this robot and this bridge.
    """
    import baxter_interface
    from baxter_interface import Head, Limb, Navigator, RobotEnable

    start = time.monotonic()
    with baxter_interface.setup():
        RobotEnable()
        Limb('left')
        Limb('right')
        Head()
        Navigator('left')
    took = time.monotonic() - start
    print(f'5 parts (robot state, both arms, head, one navigator) connected in {took:.1f} s; '
          f'the limit per part is {baxter_interface.settings.CONNECT_TIMEOUT:g} s')


def read_arms():
    from baxter_interface import Limb

    for side in ('left', 'right'):
        limb = Limb(side)
        angles = limb.joint_angles()
        print(f'{side} arm joint angles:')
        for name in limb.joint_names():
            print(f'  {name}: {angles[name]:+.3f} rad')
        if len(angles) != 7:
            raise RuntimeError(f'{side} arm: expected 7 joints, got {len(angles)}: {sorted(angles)}')
        pose = limb.endpoint_pose()
        if pose:
            p = pose['position']
            print(f'  hand position: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} m')
        else:
            raise RuntimeError(f'{side} arm: no endpoint state received')


def read_grippers():
    from baxter_interface import Gripper

    for side in ('left', 'right'):
        gripper = Gripper(side)
        print(f'{side} gripper: type={gripper.type()} calibrated={gripper.calibrated()} '
              f'ready={gripper.ready()} error={gripper.error()}')


def read_head():
    from baxter_interface import Head

    head = Head()
    print(f'head pan: {head.pan():+.3f} rad, panning: {head.panning()}, nodding: {head.nodding()}')


def read_navigators():
    from baxter_interface import Navigator

    for location in NAVIGATORS:
        nav = Navigator(location)
        print(f'{location} navigator: wheel={nav.wheel} buttons='
              f'{[nav.button0, nav.button1, nav.button2]} '
              f'lights inner={nav.inner_led} outer={nav.outer_led}')


def read_cameras():
    from baxter_interface.camera import camera_status, list_cameras

    cameras = list_cameras()
    print(f'cameras: {cameras}')
    if not cameras:
        raise RuntimeError('the robot lists no cameras')
    for name, is_open in camera_status(cameras).items():
        print(f'  {name}: {"open" if is_open else "closed"}')


def read_ik():
    """Ask the robot's inverse kinematics for joint angles of a reachable hand pose."""
    from baxter_core_msgs.srv import SolvePositionIK
    from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
    from std_msgs.msg import Header

    from baxter_interface._core.node import get_node
    from baxter_interface._core.services import call_service

    node = get_node()
    client = node.create_client(SolvePositionIK, 'ExternalTools/left/PositionKinematicsNode/IKService')
    pose = PoseStamped(
        header=Header(
            stamp=node.get_clock().now().to_msg(),
            frame_id='base'
        ),
        pose=Pose(
            position=Point(x=0.657579481614, y=0.851981417433, z=0.0388352386502),
            orientation=Quaternion(x=-0.366894936773, y=0.885980397775, z=0.108155782462, w=0.262162481772)
        )
    )
    request = SolvePositionIK.Request(pose_stamp=[pose])
    response = call_service(client, request)
    valid = list(response.is_valid)
    print(f'IK answered, solution valid: {valid}')
    if valid and valid[0]:
        print('  ' + ', '.join(f'{n}={p:+.3f}' for n, p in zip(response.joints[0].name, response.joints[0].position)))
    if not valid or not valid[0]:
        raise RuntimeError('the IK service answered, but found no solution for a pose that has one')


# --- Commanding the robot -------------------------------------------------------------------

def read_sensors():
    """Sonar ring, the distance sensor in each hand, the accelerometer in each hand."""
    import baxter_interface
    from baxter_interface import Accelerometer, RangeSensor, Sonar

    with baxter_interface.setup():
        sonar = Sonar()
        ranges = {side: RangeSensor(side) for side in ('left', 'right')}
        accelerometers = {side: Accelerometer(side) for side in ('left', 'right')}
    time.sleep(1.0)

    distances = sonar.distances()
    print(f'sonar: {len(distances)} of 12 sensors hear an echo: '
          + ', '.join(f'{sensor}: {metres:.2f} m' for sensor, metres in sorted(distances.items())))
    print(f'sonar: nearest thing: {sonar.closest()}')
    for side in ('left', 'right'):
        sensor = ranges[side]
        distance = sensor.distance()
        seen = 'nothing in range' if distance is None else f'{distance:.3f} m'
        print(f'{side} hand range sensor: {seen} (measures {sensor.min_range:g} to {sensor.max_range:g} m)')
        a = accelerometers[side].acceleration()
        strength = accelerometers[side].strength()
        print(f'{side} hand accelerometer: x={a.x:+.2f} y={a.y:+.2f} z={a.z:+.2f} m/s^2, strength {strength:.2f}')
        if not 7.0 < strength < 12.5:
            raise RuntimeError(
                f'{side} accelerometer: a hand at rest should measure gravity (about 9.8 m/s^2), not {strength:.2f}')


def hand_move():
    """The left hand 5 cm up and back down: inverse kinematics on the robot, then a joint move."""
    from baxter_interface import Limb

    limb = Limb('left')
    limb.set_joint_position_speed(0.3)
    start = limb.hand_position()
    print(f'left hand starts at x={start.x:.3f} y={start.y:.3f} z={start.z:.3f} m')
    for label, distance in (('up', 0.05), ('down', -0.05)):
        before = limb.hand_position()
        began = time.monotonic()
        limb.move_by(z=distance)
        took = time.monotonic() - began
        time.sleep(0.5)  # let the hand's reported position settle
        after = limb.hand_position()
        moved = after.z - before.z
        print(f'{label}: z {before.z:.3f} -> {after.z:.3f} m (asked {distance:+.3f}, got {moved:+.3f}); '
              f'sideways x {after.x - before.x:+.3f}, y {after.y - before.y:+.3f} m; took {took:.1f} s')
        if abs(moved - distance) > 0.015:
            raise RuntimeError(f'the hand moved {moved:+.3f} m {label} instead of {distance:+.3f} m')
    try:
        limb.solve_pose(x=3.0)
    except ValueError as error:
        print(f'a place out of reach is refused: {str(error)[:70]} ...')
    else:
        raise RuntimeError('a place 3 m in front of the robot was not refused')


def together():
    """Both arms and the head at the same time (at_once), then one arm through waypoints."""
    import baxter_interface
    from baxter_interface import Head, Limb

    with baxter_interface.setup():
        left = Limb('left')
        right = Limb('right')
        head = Head()
    for limb in (left, right):
        limb.set_joint_position_speed(0.3)

    began = time.monotonic()
    left.move_to_joint_positions({'left_w2': 0.4})
    right.move_to_joint_positions({'right_w2': -0.4})
    one_by_one = time.monotonic() - began

    began = time.monotonic()
    with baxter_interface.at_once():
        left.move_to_neutral()
        right.move_to_neutral()
        head.set_pan(0.4, speed=40)
        head.set_pan(0.0, speed=40)
    at_once = time.monotonic() - began
    print(f'two wrist moves one after the other: {one_by_one:.1f} s; '
          f'both arms to neutral and two head moves in one at_once block: {at_once:.1f} s')
    if abs(left.joint_angle('left_w2')) > 0.03 or abs(right.joint_angle('right_w2')) > 0.03:
        raise RuntimeError('the arms are not at the neutral pose after the at_once block')

    start = left.joint_angles()
    began = time.monotonic()
    left.move_through(
        [
            {'left_w1': start['left_w1'] - 0.3, 'left_w2': 0.5},
            {'left_e1': start['left_e1'] + 0.2, 'left_w2': -0.5},
            start,
        ],
        speed=0.4
    )
    print(f'left arm through 3 waypoints in one motion: {time.monotonic() - began:.1f} s')
    joint, error = max(((j, abs(a - left.joint_angle(j))) for j, a in start.items()), key=lambda item: item[1])
    print(f'back at the start within {error:.4f} rad ({joint})')


def small_move():
    """Turn each wrist by 0.2 rad (about 11 degrees) and back. Needs the robot enabled."""
    from baxter_interface import Limb

    for side in ('left', 'right'):
        limb = Limb(side)
        joint = f'{side}_w1'
        start = limb.joint_angles()
        step = -0.2 if start[joint] > 1.5 else 0.2  # stay away from the joint's upper limit
        target = dict(start)
        target[joint] = start[joint] + step

        print(f'{side} arm: {joint} from {start[joint]:+.3f} to {target[joint]:+.3f} rad ...')
        limb.move_to_joint_positions(target, timeout=15.0)
        reached = limb.joint_angle(joint)
        print(f'  reached {reached:+.3f}; going back ...')
        limb.move_to_joint_positions(start, timeout=15.0)
        print(f'  back at {limb.joint_angle(joint):+.3f}')


def move_trace():
    """Turn the left wrist by 0.2 rad and back, recording command and measurement.

    For diagnosis: writes every step to move_trace.csv and prints how the move went
    (how fast the loop ran, how close the joint got, how even the motion was). Uses the
    same smoothing as Limb.move_to_joint_positions(), but never gives up early.
    """
    from baxter_interface import Limb

    limb = Limb('left')
    joint = 'left_w1'
    rows = []
    t0 = time.monotonic()

    def run(target, seconds, label):
        cmd = limb.joint_angles()
        goal = dict(cmd)
        goal[joint] = target
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for name in goal:
                cmd[name] = 0.012488 * goal[name] + 0.98751 * cmd[name]
            limb.set_joint_positions(cmd)
            rows.append((time.monotonic() - t0, label, target, cmd[joint], limb.joint_angle(joint),
                         limb.joint_velocity(joint)))
            time.sleep(0.01)

    start = limb.joint_angle(joint)
    step = -0.2 if start > 1.5 else 0.2
    print(f'{joint}: {start:+.3f} -> {start + step:+.3f} rad (8 s), then back (8 s)')
    run(start + step, 8.0, 'out')
    run(start, 8.0, 'back')

    with open(f'{OUT}/move_trace.csv', 'w') as f:
        print('time,phase,target,command,measured,velocity', file=f)
        for row in rows:
            print('%.4f,%s,%.5f,%.5f,%.5f,%.5f' % row, file=f)

    for label in ('out', 'back'):
        part = [r for r in rows if r[1] == label]
        times = [r[0] for r in part]
        gaps = sorted(b - a for a, b in zip(times, times[1:]))
        measured = [r[4] for r in part]
        changes = sum(1 for a, b in zip(measured, measured[1:]) if a != b)
        target = part[0][2]
        within = next((r[0] - part[0][0] for r in part if abs(r[4] - target) < 0.008726646), None)
        reversals = sum(1 for a, b, c in zip(measured, measured[1:], measured[2:])
                        if (b - a) * (c - b) < -1e-8)
        print(f'{label}: loop {len(part) / (times[-1] - times[0]):.0f} per s, '
              f'longest gap {gaps[-1] * 1000:.0f} ms; measurement changed {changes} times; '
              f'final error {abs(measured[-1] - target):.4f} rad; '
              f'within 0.5 degrees after {"%.1f s" % within if within is not None else "never"}; '
              f'direction reversals {reversals}')
    print(f'saved {OUT}/move_trace.csv')


def head_move():
    from baxter_interface import Head

    head = Head()
    for angle in (0.4, -0.4, 0.0):
        print(f'head pan to {angle:+.1f} rad ...')
        head.set_pan(angle, speed=50)
        print(f'  now at {head.pan():+.3f}')
    print('nod ...')
    head.command_nod()
    print('nod finished')


def grippers():
    """Calibrate, close and open each electric gripper; other kinds are only reported."""
    from baxter_interface import Gripper

    tested = 0
    for side in ('left', 'right'):
        gripper = Gripper(side)
        kind = gripper.type()
        if kind != 'electric':
            print(f'{side} gripper: type {kind}; not an electric gripper, nothing commanded')
            continue
        if not gripper.calibrated():
            print(f'{side} gripper: calibrating ...')
            gripper.calibrate()
        print(f'{side} gripper: closing ...')
        gripper.close(True)
        print(f'  position {gripper.position():.0f} %; opening ...')
        gripper.open(True)
        print(f'  position {gripper.position():.0f} %')
        tested += 1
    print(f'{tested} electric gripper(s) tested')


def screen():
    """Show a test picture with the time on Baxter's screen, then the standard one."""
    import cv2
    import numpy as np

    from baxter_interface import Display

    display = Display()
    picture = np.zeros((600, 1024, 3), dtype=np.uint8)
    picture[:] = (120, 60, 0)  # dark blue (OpenCV order: blue, green, red)
    cv2.putText(picture, 'ROS 2 bridge', (160, 260), cv2.FONT_HERSHEY_SIMPLEX, 3.5, (255, 255, 255), 8)
    cv2.putText(picture, time.strftime('%H:%M:%S'), (330, 420), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 220, 255), 6)
    display.show(picture)
    print('LOOK: the screen should now show "ROS 2 bridge" and the time, white and yellow on blue')
    time.sleep(6.0)
    cv2.imwrite(f'{OUT}/screen_test_picture.png', picture)
    display.show(f'{OUT}/screen_test_picture.png')  # the same again, this time from a file
    time.sleep(2.0)


def screen_reset():
    import os

    import baxter_interface
    from baxter_interface import Display

    Display().show(os.path.join(os.path.dirname(baxter_interface.__file__), 'images', 'researchsdk.png'))
    print('screen back to the standard picture')


def _test_camera(camera, display, replace=None, seconds=8.0):
    import cv2

    print(f'{camera.name}: opening at 640 x 400 (closing {replace} first if two others are open) ...')
    camera.open(replace=replace)
    image = camera.frame(timeout=10.0)
    print(f'  first picture: {image.shape[1]} x {image.shape[0]}, {image.shape[2]} colours, '
          f'mean brightness {image.mean():.0f} of 255')
    path = f'{OUT}/{camera.name}.png'
    cv2.imwrite(path, image)
    print(f'  saved as {path}')

    count = [0]

    def on_frame(picture):
        count[0] += 1
        display.show(picture)

    print(f'  LOOK: live picture of {camera.name} on the screen for {seconds:.0f} s')
    camera.frame_received.connect(on_frame)
    time.sleep(seconds)
    camera.frame_received.disconnect(on_frame)
    print(f'  {count[0]} pictures in {seconds:.0f} s shown on the screen: {count[0] / seconds:.1f} per second')
    if count[0] < 2:
        raise RuntimeError(f'{camera.name}: pictures stopped arriving')


def cameras():
    """Left hand camera, then the head camera (only two run at a time), then back to normal."""
    from baxter_interface import Camera, Display
    from baxter_interface.camera import camera_status, list_cameras

    display = Display()
    left = Camera('left_hand')
    head = Camera('head')
    print(f'cameras at the start (True = open): {camera_status(list_cameras())}')
    try:
        # Normally the two hand cameras run. If the head camera runs instead of the left
        # hand camera, it is closed first.
        _test_camera(left, display, replace='head')
        _test_camera(head, display, replace='left_hand')
    finally:
        print('back to normal: both hand cameras on, head camera off')
        left.open(replace='head')
        Camera('right_hand').open(replace='head')
        print(f'cameras at the end (True = open): {camera_status(list_cameras())}')
        time.sleep(0.5)
        screen_reset()


def head_lights():
    """The halo changes colour; the sonar lights run round the ring, then go back to the robot."""
    from baxter_interface import HeadLights

    lights = HeadLights()
    before = lights.halo()
    print(f'halo before: red {before.red:g}, green {before.green:g}; '
          f'sonar lights on before: {lights.sonar_lights()}')
    for red, green, name in ((100, 0, 'red'), (100, 100, 'red and green'), (0, 100, 'green')):
        lights.set_halo(red=red, green=green)
        time.sleep(1.5)
        now = lights.halo()
        print(f'halo set to {name}: the robot reports red {now.red:g}, green {now.green:g}')
        if abs(now.red - red) > 1 or abs(now.green - green) > 1:
            raise RuntimeError(f'the halo did not take the colour {name}')
    lights.set_halo(red=before.red, green=before.green)

    seen = set()
    for _round in range(2):
        for light in range(12):
            lights.set_sonar_lights([light])
            time.sleep(0.15)
            seen.update(lights.sonar_lights())
    lights.set_sonar_lights('all')
    time.sleep(1.0)
    all_on = lights.sonar_lights()
    lights.release_sonar_lights()
    time.sleep(1.0)
    print(f'lights the robot reported on during the round: {sorted(seen)}; with all on: {all_on}; '
          f'after handing them back: {lights.sonar_lights()}')
    if len(all_on) != 12:
        raise RuntimeError(f'with all sonar lights switched on, the robot reported only {all_on}')


def navigator_lights():
    from baxter_interface import Navigator

    nav = Navigator('left')
    print('LOOK: the lights of the navigator on the LEFT ARM blink three times')
    for _ in range(3):
        nav.inner_led = True
        nav.outer_led = True
        time.sleep(0.4)
        nav.inner_led = False
        nav.outer_led = False
        time.sleep(0.4)
    print(f'lights now: inner={nav.inner_led} outer={nav.outer_led}')


def button():
    """Wait for a navigator button; the function connected to it makes the head nod.

    This is the case the per-callback threads exist for: a connected function that
    commands the robot and waits for it.
    """
    from baxter_interface import Head, Navigator

    head = Head()
    pressed = threading.Event()
    nodded = threading.Event()
    result = {}

    def on_button(location):
        def handler(is_pressed):
            if is_pressed and not pressed.is_set():
                result['location'] = location
                result['thread'] = threading.current_thread().name
                pressed.set()
                head.command_nod()  # commands the robot and waits, inside the callback
                nodded.set()
        return handler

    navigators = {location: Navigator(location) for location in NAVIGATORS}
    for location, nav in navigators.items():
        nav.button0_changed.connect(on_button(location))

    print('DO: press the wheel of any navigator (on an arm or on the torso) within 60 s.')
    print('    The head should nod once.', flush=True)
    if not pressed.wait(60.0):
        raise TimeoutError('no button press arrived within 60 s')
    print(f"button press received from the {result['location']} navigator, in thread {result['thread']}")
    if not nodded.wait(15.0):
        raise TimeoutError('the nod commanded from inside the callback did not finish')
    print('the head nodded from inside the callback')


STEPS = {
    function.__name__: function for function in (
        read_version, read_connect_time, read_arms, read_grippers, read_head, read_navigators, read_cameras,
        read_ik, read_sensors, hand_move, together, small_move, move_trace, head_move, grippers, screen, screen_reset, cameras,
        navigator_lights, head_lights, button,
    )
}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in STEPS:
        sys.exit(f'Usage: robot_check_steps.py <step>\nSteps: {", ".join(STEPS)}')
    try:
        STEPS[sys.argv[1]]()
    except Exception as error:
        traceback.print_exc()
        print(f'STEP FAILED: {error}')
        sys.exit(1)


if __name__ == '__main__':
    main()
