"""
Tests for src/soft_stop.py: the topics it publishes to (T14) and the button logic.

gpiozero is replaced by a fake, so no GPIO is needed. Needs ROS sourced; the nodes only talk on
this machine.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_soft_stop.py -v
"""

import os
import pathlib
import sys
import types

import pytest

os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)  # never the lab's domain


class FakeButton:
    """gpiozero.Button stand-in. is_pressed True = pin pulled low = the button released."""

    fail = False

    def __init__(self, pin, **kwargs):
        if FakeButton.fail:
            raise RuntimeError('GPIO busy')
        self.pin = pin
        self.is_pressed = False


sys.modules['gpiozero'] = types.SimpleNamespace(Button=FakeButton)
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

import soft_stop  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
# What soft_stop.sh passes on the ladertanken Pi.
PI_TOPICS = ['/wavemakers/ladertanken/soft_stop', '/oscillators/vertical/soft_stop',
             '/oscillators/sloshing/soft_stop']


@pytest.fixture
def ros():
    """Start rclpy with the given command line arguments; shut it down after the test."""
    started = []

    def start(*args):
        rclpy.init(args=['soft_stop', '--ros-args', *args])
        started.append(True)

    yield start
    if started:
        rclpy.try_shutdown()


@pytest.fixture(autouse=True)
def gpio_ok():
    FakeButton.fail = False
    yield
    FakeButton.fail = False


def topics_arg(topics):
    return '-p', 'topics:=[' + ', '.join(f"'{t}'" for t in topics) + ']'


def received(node, topics, ticks=1):
    """Call tick() `ticks` times and return the last value received on each topic."""
    listener = rclpy.create_node('listener')
    last = {}
    for topic in topics:
        listener.create_subscription(
            Bool, topic, lambda msg, topic=topic: last.__setitem__(topic, msg.data), QOS)
    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    for _ in range(ticks):
        node.tick()
    for _ in range(100):  # up to about 5 s for discovery
        executor.spin_once(timeout_sec=0.05)
        if len(last) == len(topics):
            break
    listener.destroy_node()
    return last


# ---------------------------------------------------------------- topics (T14)

def test_default_is_the_relative_topic(ros):
    ros('-r', '__ns:=/wavemakers/ladertanken')
    node = soft_stop.SoftStop()
    assert list(node.topics) == ['soft_stop']
    assert [p.topic_name for p in node.stop_publishers] == ['/wavemakers/ladertanken/soft_stop']
    node.destroy_node()


def test_topics_parameter_gives_one_publisher_each(ros):
    ros('-r', '__ns:=/ladertanken', *topics_arg(PI_TOPICS))
    node = soft_stop.SoftStop()
    assert [p.topic_name for p in node.stop_publishers] == PI_TOPICS
    node.destroy_node()


def test_relative_topics_resolve_in_the_namespace(ros):
    ros('-r', '__ns:=/ladertanken', *topics_arg(['soft_stop', '/oscillators/vertical/soft_stop']))
    node = soft_stop.SoftStop()
    assert [p.topic_name for p in node.stop_publishers] == [
        '/ladertanken/soft_stop', '/oscillators/vertical/soft_stop']
    node.destroy_node()


# ---------------------------------------------------------------- what every topic gets

def test_every_topic_starts_stopped(ros):
    ros('-r', '__ns:=/ladertanken', *topics_arg(PI_TOPICS))
    node = soft_stop.SoftStop()
    assert received(node, PI_TOPICS) == {t: True for t in PI_TOPICS}
    node.destroy_node()


def test_release_reaches_every_topic(ros, monkeypatch):
    ros('-r', '__ns:=/ladertanken', *topics_arg(PI_TOPICS))
    node = soft_stop.SoftStop()
    node.button.is_pressed = True  # released: pin low
    now = [1000.0]
    monkeypatch.setattr(soft_stop.time, 'monotonic', lambda: now[0])
    node.tick()
    now[0] += soft_stop.RELEASE_HOLD_S + 0.01
    assert received(node, PI_TOPICS) == {t: False for t in PI_TOPICS}
    node.destroy_node()


def test_press_reaches_every_topic(ros, monkeypatch):
    ros('-r', '__ns:=/ladertanken', *topics_arg(PI_TOPICS))
    node = soft_stop.SoftStop()
    node.button.is_pressed = True
    now = [1000.0]
    monkeypatch.setattr(soft_stop.time, 'monotonic', lambda: now[0])
    node.tick()
    now[0] += soft_stop.RELEASE_HOLD_S + 0.01
    node.tick()
    assert node.stopped is False
    node.button.is_pressed = False  # pressed: pin high
    node.tick()
    now[0] += soft_stop.TRIP_FILTR_S + 0.01
    assert received(node, PI_TOPICS) == {t: True for t in PI_TOPICS}
    node.destroy_node()


def test_without_gpio_every_topic_stays_stopped(ros):
    FakeButton.fail = True
    ros('-r', '__ns:=/ladertanken', *topics_arg(PI_TOPICS))
    node = soft_stop.SoftStop()
    assert node.button is None
    assert received(node, PI_TOPICS, ticks=3) == {t: True for t in PI_TOPICS}
    node.destroy_node()
