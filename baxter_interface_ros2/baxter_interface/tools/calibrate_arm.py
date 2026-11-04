"""Run the robot's arm calibration routine (CalibrateArm).

ROS 2 version of baxter_tools' calibrate_arm.py (v0.7.0, Copyright (c) 2013,
Rethink Robotics, BSD license; see LICENSE):
    ros2 run baxter_interface calibrate_arm -l left

The arm moves by itself for up to 10 minutes and the robot's stored calibration changes.
Remove the gripper first. Run tare afterwards.

Difference from the ROS 1 tool: asks for confirmation before starting (-y skips it).

Needs these topics in bridge_config/topics.yaml, for the arm used:
    /robustcontroller/<limb>/CalibrateArm/enable   baxter_maintenance_msgs/msg/CalibrateArmEnable
    /robustcontroller/<limb>/CalibrateArm/status   baxter_core_msgs/msg/RobustControllerStatus
"""

import sys

from baxter_maintenance_msgs.msg import CalibrateArmEnable

from baxter_interface import RobustController
from baxter_interface.tools._maintenance import run_maintenance


class CalibrateArm(RobustController):
    """Wrapper to run the CalibrateArm robust controller on one arm."""

    def __init__(self, limb):
        """
        Parameters:
            limb: arm to calibrate, 'left' or 'right'
        """
        enable_msg = CalibrateArmEnable(
            is_enabled=True,
            uid='sdk'
        )
        disable_msg = CalibrateArmEnable(
            is_enabled=False,
            uid='sdk'
        )

        # 10 minute timeout for the CalibrateArm process
        super().__init__(
            f'robustcontroller/{limb}/CalibrateArm',
            enable_msg,
            disable_msg,
            10 * 60
        )


def main():
    return run_maintenance(
        'Calibrate arm',
        (__doc__ or '').splitlines()[0],
        CalibrateArm
    )


if __name__ == '__main__':
    sys.exit(main())
