"""What RobotClient puts on the wire, with the socket replaced.

RobotClient(connect=False) never opens a connection; overriding _send and
_send_and_wait captures the outgoing messages and scripts the replies.
"""
import pytest

from robot_client import RobotClient


class FakeClient(RobotClient):
    def __init__(self, config, replies=None):
        super().__init__(config=config, connect=False)
        self.sent = []
        self.replies = replies or {}

    def _send(self, msg):
        self.sent.append(msg)

    def _send_and_wait(self, msg, response_type, timeout=3.0):
        self.sent.append(msg)
        reply = self.replies.get(response_type)
        return reply(msg) if callable(reply) else reply


@pytest.fixture
def meca(config_path):
    return FakeClient(config_path("meca500"))


def test_config_drives_joint_names(meca):
    assert meca.name == "Meca500"
    assert len(meca.joint_names) == 6


def test_joints_sends_floats_in_order(meca):
    meca.joints(1, 2, 3, 4, 5, 6)
    assert meca.sent == [{"cmd": "setJoints", "angles": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}]


def test_joints_accepts_a_list(meca):
    meca.joints([0, 10, 20, 30, 40, 50])
    assert meca.sent[0]["angles"] == [0, 10, 20, 30, 40, 50]


def test_joints_with_wrong_count_sends_nothing(meca, capsys):
    meca.joints(1, 2, 3)
    assert meca.sent == []
    assert "Expected" in capsys.readouterr().out


def test_joint_by_name_and_prefix(meca):
    first = meca.joint_names[0]
    meca.joint(first, 45)
    meca.joint(first.split()[0].lower(), -45)
    assert [m["index"] for m in meca.sent] == [0, 0]
    assert [m["angle"] for m in meca.sent] == [45.0, -45.0]


def test_unknown_joint_sends_nothing(meca):
    meca.joint("elbow-of-doom", 10)
    assert meca.sent == []
