"""Tests for the parts of "several robots" that need no robot and no ROS 2 traffic:
robot names, finding the robots that are there, and keeping the locks of two robots apart."""

import pytest

from baxter_interface import robot as robot_module
from baxter_interface._core.locks import part_lock
from baxter_interface._core.node import robot_namespace


def test_a_robot_name_becomes_its_namespace():
    assert robot_namespace('one') == '/one'
    assert robot_namespace('baxter_2') == '/baxter_2'
    assert robot_namespace('') == '/'          # the robot without a name


@pytest.mark.parametrize('name', ['2nd', 'left arm', 'one/two', '/one', 'a__b', '_one', 'one_', 'größe'])
def test_names_that_cannot_be_part_of_a_ros_name_are_refused(name):
    with pytest.raises(ValueError, match='cannot be a robot name'):
        robot_namespace(name)


class NodeOfRobot:
    """Stands in for a ROS 2 node: its namespace, and the topics it sees."""

    def __init__(self, namespace, publishers=None):
        self._namespace = namespace
        self._publishers = publishers or {}

    def get_namespace(self):
        return self._namespace

    def get_topic_names_and_types(self):
        return [(topic, ['std_msgs/msg/String']) for topic in self._publishers]

    def count_publishers(self, topic):
        return self._publishers.get(topic, 0)


def test_robots_are_found_by_the_topic_every_bridge_offers():
    node = NodeOfRobot('/', {
        '/two/baxter_bridge/robot_info': 1,
        '/one/baxter_bridge/robot_info': 1,
        '/one/robot/joint_states': 1,
    })

    assert robot_module._robots_seen(node) == ['one', 'two']


def test_a_robot_without_a_name_is_found_as_the_empty_name():
    node = NodeOfRobot('/', {'/baxter_bridge/robot_info': 1, '/one/baxter_bridge/robot_info': 1})

    assert robot_module._robots_seen(node) == ['', 'one']


def test_a_topic_that_only_this_program_listens_to_is_no_robot():
    # Asking for a robot that is not there creates the topic, without a publisher.
    node = NodeOfRobot('/', {'/three/baxter_bridge/robot_info': 0, '/one/baxter_bridge/robot_info': 1})

    assert robot_module._robots_seen(node) == ['one']


def test_a_robot_that_is_not_there_is_explained(monkeypatch):
    node = NodeOfRobot('/three', {'/one/baxter_bridge/robot_info': 1, '/baxter_bridge/robot_info': 1})
    monkeypatch.setattr(robot_module, 'robot_node', lambda name: node)
    monkeypatch.setattr(robot_module, 'LOOKUP_TIME', 0.2)

    with pytest.raises(ConnectionError, match="No robot named 'three'.*Robots found: one without a name, 'one'"):
        robot_module.Robot('three')


def test_a_robot_that_is_there_gets_its_name_and_namespace(monkeypatch):
    node = NodeOfRobot('/one', {'/one/baxter_bridge/robot_info': 1})
    monkeypatch.setattr(robot_module, 'robot_node', lambda name: node)

    robot = robot_module.Robot('one')

    assert (robot.name, robot.namespace, robot.node) == ('one', '/one', node)


def test_the_same_part_of_two_robots_has_two_locks():
    one, also_one, two = NodeOfRobot('/one'), NodeOfRobot('/one'), NodeOfRobot('/two')

    assert part_lock('limb/left', one) is part_lock('limb/left', also_one)
    assert part_lock('limb/left', one) is not part_lock('limb/left', two)
    assert part_lock('limb/left', one) is not part_lock('limb/right', one)
