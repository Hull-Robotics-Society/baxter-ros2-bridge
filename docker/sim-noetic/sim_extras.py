#!/usr/bin/env python3
"""Parts of the real robot's ROS 1 interface that Rethink's simulator does not have.

Runs next to the simulator (see baxter_sim.launch), so that programs written for the
real robot find the same topics and services in the simulation.

Cameras
    The robot runs two of its three cameras at a time and has services to list, open
    and close them. The simulator's cameras send pictures all the time and have no such
    services. Here the simulated cameras publish under /sim/cameras/... (a patch to the
    robot model does that), and this program passes on only the cameras that are "open":

        /cameras/list    all three names, as the robot answers after a start
        /cameras/open    starts passing a camera on; refused (EAGAIN) while two others
                         are open, EINVAL for an unknown name
        /cameras/close   stops it; EINVAL if it was not open
        /cameras/reset   answers, does nothing

    After a start the two hand cameras are open, as on the robot. The picture size is
    fixed (640 x 400); the settings in an open request are ignored. A closed camera is
    not drawn at all, which saves processor time.

Accelerometers
    /robot/accelerometer/<side>_accelerometer/state   sensor_msgs/Imu, 100 times per second:
        a hand at rest (gravity along z). The simulated hands' real motion is NOT in it.
"""

import errno
import subprocess
import threading
import time

import rospy
from baxter_core_msgs.srv import (
    CloseCamera, CloseCameraResponse, ListCameras, ListCamerasResponse, OpenCamera, OpenCameraResponse,
)
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse

GRAVITY = 9.81
CAMERAS = ('head_camera', 'left_hand_camera', 'right_hand_camera')
OPEN_AT_START = ('left_hand_camera', 'right_hand_camera')
MOST_AT_ONCE = 2


class Cameras:
    """The robot's camera services, for the simulator's always-running cameras."""

    def __init__(self):
        self._lock = threading.Lock()
        self._relays = {}  # camera name -> the programs passing its topics on
        for name in OPEN_AT_START:
            self._start(name)
        rospy.on_shutdown(self._stop_all)
        rospy.Service('/cameras/list', ListCameras, self._list)
        rospy.Service('/cameras/open', OpenCamera, self._open)
        rospy.Service('/cameras/close', CloseCamera, self._close)
        rospy.Service('/cameras/reset', Empty, lambda request: EmptyResponse())

    def _start(self, name):
        # topic_tools' relay passes messages on unopened, whatever their size. "lazy":
        # it only listens to the simulated camera while someone listens to it, so an
        # open camera nobody looks at is not drawn either.
        self._relays[name] = [
            subprocess.Popen([
                'rosrun', 'topic_tools', 'relay',
                '/sim/cameras/%s/%s' % (name, topic), '/cameras/%s/%s' % (name, topic),
                '__name:=camera_relay_%s_%s' % (name, topic), '_lazy:=true',
            ])
            for topic in ('image', 'camera_info')
        ]

    def _stop(self, name):
        for process in self._relays.pop(name, []):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()

    def _stop_all(self):
        with self._lock:
            for name in list(self._relays):
                self._stop(name)

    def _list(self, _request):
        return ListCamerasResponse(cameras=list(CAMERAS))

    def _open(self, request):
        with self._lock:
            if request.name not in CAMERAS:
                return OpenCameraResponse(err=errno.EINVAL)
            if request.name in self._relays:
                return OpenCameraResponse(err=0)  # open already; there are no settings to apply
            if len(self._relays) >= MOST_AT_ONCE:
                return OpenCameraResponse(err=errno.EAGAIN)
            self._start(request.name)
            return OpenCameraResponse(err=0)

    def _close(self, request):
        with self._lock:
            if request.name not in self._relays:
                return CloseCameraResponse(err=errno.EINVAL)
            self._stop(request.name)
            return CloseCameraResponse(err=0)


def main():
    rospy.init_node('baxter_sim_extras')
    cameras = Cameras()  # noqa: F841 (kept alive for its services)

    accelerometers = {
        side: rospy.Publisher('/robot/accelerometer/%s_accelerometer/state' % side, Imu, queue_size=1)
        for side in ('left', 'right')
    }
    at_rest = {}
    for side in accelerometers:
        at_rest[side] = Imu()
        at_rest[side].header.frame_id = '%s_hand_accelerometer' % side
        at_rest[side].linear_acceleration.z = GRAVITY
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for side, publisher in accelerometers.items():
            at_rest[side].header.stamp = now
            publisher.publish(at_rest[side])
        # Real seconds, not rospy's: the simulation's own time stops when it is paused.
        time.sleep(0.01)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
