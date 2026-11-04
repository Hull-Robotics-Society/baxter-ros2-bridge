"""Tests for the rule that never lets a third camera be opened. No robot needed.

Background: after a start the robot lists all three cameras although only two can run.
Opening the head camera next to both hand cameras made the robot's camera program stop
answering (7 Oct 2026).
"""

from baxter_interface import camera as camera_module
from baxter_interface.camera import cameras_in_the_way

HEAD, LEFT, RIGHT = 'head_camera', 'left_hand_camera', 'right_hand_camera'


def stand_in_robot(monkeypatch, listed, running):
    """Replace the two questions to the robot: which cameras it lists, which are open.

    Returns:
        dict with 'status_calls': how often the (slow) question "which are open" was asked
    """
    counts = {'status_calls': 0}

    def list_cameras(node=None):
        return list(listed)

    def camera_status(cameras=None, listen=1.5, node=None):
        counts['status_calls'] += 1
        return {name: name in running for name in cameras}

    monkeypatch.setattr(camera_module, 'list_cameras', list_cameras)
    monkeypatch.setattr(camera_module, 'camera_status', camera_status)
    return counts


def test_all_three_listed_and_two_others_open_blocks_the_third(monkeypatch):
    # The situation after a start of the robot, in which the head camera was opened as a
    # third one.
    stand_in_robot(monkeypatch, listed=[HEAD, LEFT, RIGHT], running=[LEFT, RIGHT])

    assert cameras_in_the_way(HEAD) == [LEFT, RIGHT]


def test_an_open_camera_can_be_opened_again(monkeypatch):
    # Opening an open camera restarts it with new settings; only one other is open.
    stand_in_robot(monkeypatch, listed=[HEAD, LEFT, RIGHT], running=[LEFT, RIGHT])

    assert cameras_in_the_way(LEFT) == []


def test_one_other_open_leaves_room(monkeypatch):
    stand_in_robot(monkeypatch, listed=[HEAD, LEFT, RIGHT], running=[RIGHT])

    assert cameras_in_the_way(HEAD) == []
    assert cameras_in_the_way(LEFT) == []


def test_a_camera_the_robot_does_not_list_is_blocked_by_the_listed_ones(monkeypatch):
    stand_in_robot(monkeypatch, listed=[LEFT, RIGHT], running=[])

    assert cameras_in_the_way(HEAD) == [LEFT, RIGHT]


def test_two_listed_cameras_need_no_listening(monkeypatch):
    # With only one other camera listed there cannot be two others open: no need to
    # spend seconds on listening for messages.
    counts = stand_in_robot(monkeypatch, listed=[HEAD, RIGHT], running=[RIGHT])

    assert cameras_in_the_way(HEAD) == []
    assert counts['status_calls'] == 0
