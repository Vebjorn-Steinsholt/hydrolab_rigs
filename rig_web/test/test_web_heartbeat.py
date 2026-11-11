"""
Tests for the web server's heartbeat to the bridge, and the log of its gaps (TODO 49).

The node is built for real, with its clock, logger and heartbeat publisher replaced; nothing is
served. Needs the workspace sourced (rig_interfaces, nav2_msgs).

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_web/test/test_web_heartbeat.py -v
"""

import importlib.util
import os
import pathlib

import pytest

os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)  # never the lab's domain

rclpy = pytest.importorskip('rclpy')

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'web_server.py'
_spec = importlib.util.spec_from_file_location('web_server', SCRIPT)
web_server = importlib.util.module_from_spec(_spec)
try:
    _spec.loader.exec_module(web_server)
except ModuleNotFoundError as error:  # rig_interfaces or nav2_msgs not sourced
    pytest.skip(f'workspace not sourced: {error}', allow_module_level=True)


class FakeLogger:

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    def error(self, text):
        self.lines.append(('error', text))


class FakePublisher:

    def __init__(self):
        self.count = 0

    def publish(self, msg):
        self.count += 1


@pytest.fixture
def node(monkeypatch):
    """Return a WebNode on a fake clock (node.now[0]), logger and heartbeat publisher."""
    rclpy.init(args=['web', '--ros-args', '-p', 'idle_timeout_s:=0.0'])
    now = [1000.0]
    monkeypatch.setattr(web_server.time, 'monotonic', lambda: now[0])
    n = web_server.WebNode()
    logger = FakeLogger()
    n.get_logger = lambda: logger
    n._heartbeat_pub = FakePublisher()
    n.now, n.logger = now, logger
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def tick(node, seconds=web_server.HEARTBEAT_PERIOD_S):
    node.now[0] += seconds
    node._send_heartbeat()


def test_no_heartbeat_and_no_log_before_any_page(node):
    for _ in range(10):
        tick(node)
    assert node._heartbeat_pub.count == 0
    assert node.logger.lines == []


def test_heartbeat_while_a_page_acknowledges(node):
    for _ in range(10):
        node.acknowledge('192.168.1.110')
        node.event_sent()
        tick(node)
    assert node._heartbeat_pub.count == 10
    assert node.logger.lines == []


def test_lost_acknowledgments_logged_once_with_the_gap(node):
    node._state = 'running'
    node.acknowledge('192.168.1.110')
    for _ in range(10):  # 1 s: events keep going out, no acknowledgment comes back
        node.event_sent()
        tick(node)
    assert node._heartbeat_pub.count == 4  # 0.1-0.4 s; at 0.5 s the page counts as gone
    warnings = [text for level, text in node.logger.lines if level == 'warn']
    assert len(warnings) == 1
    assert 'No page acknowledged for 0.50 s (last from 192.168.1.110)' in warnings[0]
    assert 'last event sent 0.10 s ago' in warnings[0]  # one tick: the server was still sending


def test_server_side_stall_shows_as_old_events(node):
    node._state = 'running'
    node.acknowledge('192.168.1.110')
    node.event_sent()
    for _ in range(5):  # no events go out either
        tick(node)
    (level, text), = node.logger.lines
    assert level == 'warn'
    assert 'last event sent 0.50 s ago' in text


def test_recovery_logs_the_gap(node):
    node._state = 'running'
    node.acknowledge('192.168.1.110')
    for _ in range(8):
        tick(node)
    node.acknowledge('192.168.1.110')
    assert node.logger.lines[-1] == (
        'info', 'Page 192.168.1.110 acknowledged again after 0.80 s')
    tick(node)
    assert node._heartbeat_pub.count == 5  # 4 before the loss, 1 after


def test_idle_loss_is_info_not_warn(node):
    node._state = 'idle'  # a page closed with no wave running is normal
    node.acknowledge('192.168.1.110')
    for _ in range(6):
        tick(node)
    assert [level for level, _ in node.logger.lines] == ['info']


def test_late_timer_tick_is_logged(node):
    tick(node)
    tick(node, 0.45)
    (level, text), = node.logger.lines
    assert level == 'warn'
    assert 'Heartbeat timer ran 0.45 s after the previous tick (this process stalled)' in text


def test_normal_ticks_are_not_late(node):
    for _ in range(20):
        tick(node, 0.25)
    assert node.logger.lines == []
