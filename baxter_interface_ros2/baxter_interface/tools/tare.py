"""Run the robot's tare routine (Tare), which zeroes an arm's force sensing.

ROS 2 version of baxter_tools' tare.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE):
    ros2 run baxter_interface tare -l left

The arm moves by itself for up to 5 minutes and the robot's stored calibration changes.
Remove the gripper first.

Difference from the ROS 1 tool: asks for confirmation before starting (-y skips it).

Needs these topics in bridge_config/topics.yaml, for the arm used:
    /robustcontroller/<limb>/Tare/enable   baxter_maintenance_msgs/msg/TareEnable
    /robustcontroller/<limb>/Tare/status   baxter_core_msgs/msg/RobustControllerStatus
"""

import sys

from baxter_maintenance_msgs.msg import TareEnable

from baxter_interface import RobustController
from baxter_interface.tools._maintenance import run_maintenance


class Tare(RobustController):
    """Wrapper to run the Tare robust controller on one arm."""

    def __init__(self, limb):
        """
        Parameters:
            limb: arm to tare, 'left' or 'right'
        """
        enable_msg = TareEnable(
            is_enabled=True,
            uid='sdk'
        )
        enable_msg.data.tune_gravity_spring = True
        disable_msg = TareEnable(
            is_enabled=False,
            uid='sdk'
        )

        # 5 minute timeout for the Tare process
        super().__init__(
            f'robustcontroller/{limb}/Tare',
            enable_msg,
            disable_msg,
            5 * 60
        )


def main():
    return run_maintenance(
        'Tare',
        (__doc__ or '').splitlines()[0],
        Tare
    )


if __name__ == '__main__':
    sys.exit(main())
