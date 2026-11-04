"""Calling a ROS 2 service and waiting for its answer.

Fine from the main program and from functions connected to signals (they run in their own
threads). Not from a raw rclpy callback on the shared node: that runs in the receiving
thread, which is the one that delivers the answer (see node.py).
"""

from __future__ import annotations

import time
from typing import Any

from rclpy.client import Client

SERVICE_TIMEOUT = 10.0


def _full_name(client: Client) -> str:
    """The service's complete name, with the robot's name if it has one (for messages)."""
    try:
        return str(client.service_name)
    except Exception:
        return str(client.srv_name)


def call_service(client: Client, request: Any, timeout: float = SERVICE_TIMEOUT) -> Any:
    """Call a service and return its response.

    Parameters:
        client:  service client created with node.create_client(...)
        request: the request message
        timeout: seconds to wait, both for the service to appear and for its answer

    Returns:
        the service's response message

    Raises:
        TimeoutError if the service is not available or does not answer in time.
    """
    name = _full_name(client)
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(
            f'Service {name} not available. Is the bridge running (docker compose '
            'up -d bridge), and does the robot provide the service? With '
            'BRIDGE_MODE=fixed_list it must also be listed in bridge_config/topics.yaml '
            '(services_2_to_1).')

    future = client.call_async(request)
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            raise TimeoutError(
                f'No answer from {name} within {timeout:g} s. If the robot never '
                'answers, the bridge keeps waiting for it and answers no other service '
                'call; its guard notices that and restarts it within about a minute. If '
                'the same call fails again after that, the program on the robot that '
                'provides the service is stuck (restart the robot).')
        time.sleep(0.01)
    return future.result()
