"""Read information about the robot that only exists as ROS 1 parameters.

The bridge can't carry ROS 1 parameters, so a helper in the bridge container
(docker/bridge-jazzy/robot_info.py) publishes selected ones on /baxter_bridge/robot_info.
Which ones is configured under `robot_info_params` in bridge_config/topics.yaml.

    import baxter_interface
    print(baxter_interface.software_version())    # e.g. '1.2.0.57'
    print(baxter_interface.robot_info())          # every configured parameter

Needs /baxter_bridge/robot_info in bridge_config/topics.yaml, and a bridge image that
includes the helper.
"""

from __future__ import annotations

import json
from typing import Any

from rclpy.node import Node
from std_msgs.msg import String

from ._core import dataflow
from ._core.node import get_node

_TOPIC = 'baxter_bridge/robot_info'


def robot_info(timeout: float = 5.0, node: Node | None = None) -> dict[str, Any]:
    """Return the robot's ROS 1 parameters that the bridge passes on.

    Parameters:
        timeout: seconds to wait for the information (default 5)
        node:    optional ROS 2 node to use (default: the shared background node)

    Returns:
        dict {parameter name: value}. A parameter the robot doesn't have is None; a
        namespace such as '/rethink' is a dict of everything below it.

    Raises:
        TimeoutError if nothing arrives, e.g. because the bridge image is older than the
        helper or the topic is not in bridge_config/topics.yaml.
    """
    msg = dataflow.wait_for_message(
        get_node(node),
        String,
        _TOPIC,
        timeout
    )
    return json.loads(msg.data)['params']


def software_version(timeout: float = 5.0, node: Node | None = None) -> str | None:
    """Return the robot's software version, or None if the robot doesn't report one.

    Looks for the ROS 1 parameter /rethink/software_version.

    Parameters:
        timeout: seconds to wait for the information (default 5)
        node:    optional ROS 2 node to use (default: the shared background node)

    Returns:
        str, e.g. '1.2.0' (anything after an underscore is cut off, as the SDK did), or None
    """
    params = robot_info(timeout, node)

    version = params.get('/rethink/software_version')
    if version is None and isinstance(params.get('/rethink'), dict):
        version = params['/rethink'].get('software_version')

    if version is None:
        return None
    return str(version).split('_')[0]
