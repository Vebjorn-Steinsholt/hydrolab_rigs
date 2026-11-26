"""
Tests for src/wave_probe_node.py, with a pseudo-terminal as the Arduino's port.

Covers the Range messages, the dropped lines, gaps and resets, reopening the port, the
parameters, SIGINT, and the zero service with the surface elevation.

Lines are written at 20 Hz in real time, so the clock fit sees a steady Arduino. Each test runs
on a domain of its own, so it can't reach a rig. Nothing waits a fixed time for the node: a zero
is fed until it answers, and messages are awaited until they stop coming, so a slow Pi passes.

    cd ~/ros_ws && source install/setup.bash   # wave_probe_interfaces
    cd src/hydrolab_rigs
    python3 -m pytest wave_probe/test/test_wave_probe_node.py -v
"""

import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range

SRC = pathlib.Path(__file__).resolve().parents[1] / 'src'
sys.path.insert(0, str(SRC))
from probe_serial import range_m  # noqa: E402, I100
from wave_probe_interfaces.msg import SurfaceElevation  # noqa: E402
from wave_probe_interfaces.srv import ZeroMeasurement  # noqa: E402
import wave_probe_node  # noqa: E402

AIR_TEMP_C = 21           # passed as an int, as `-p air_temp_c:=21` does
ECHO_US = 1744            # 0.300 m at 21 °C
NOISY_US = 35             # ± this alternating gives a std of 6 mm
PERIOD_S = 0.05
REOPEN_S = 0.5
DOMAIN_ID = 100 + os.getpid() % 100


# ---------------------------------------------------------------- the port

class Port:
    """A pty behind a symlink, written like the Arduino: seq,t_us,echo_us at 20 Hz."""

    def __init__(self, link):
        self.link = link
        self.seq = 0
        self.connect()

    def connect(self):
        self.master, self.slave = os.openpty()
        tmp = self.link.with_suffix('.new')
        tmp.symlink_to(os.ttyname(self.slave))
        os.replace(tmp, self.link)

    def disconnect(self):
        os.close(self.master)
        os.close(self.slave)

    def node_has_it(self) -> bool:
        # Read from /proc/locks: taking the lock to test it would make the node's open fail.
        """Tell whether some process holds the flock that serial's exclusive=True takes."""
        st = os.stat(self.link)
        dev = f'{os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x}:{st.st_ino}'
        with open('/proc/locks') as f:
            return any(ln.split()[1] == 'FLOCK' and dev in ln.split() for ln in f)

    def wait_open(self, timeout=5.0):
        wait_until(self.node_has_it, timeout)
        time.sleep(0.1)          # past the node's reset_input_buffer

    def write(self, raw: bytes):
        os.write(self.master, raw)

    def line(self, echo_us=ECHO_US, seq=None) -> bytes:
        self.seq = self.seq + 1 if seq is None else seq
        return f'{self.seq},{(self.seq * 50_000) % 2**32},{echo_us}\r\n'.encode()

    def feed(self, n=None, echo=lambda i: ECHO_US, until=None, timeout=20.0):
        """Write n lines at 20 Hz, or until until() is true; echo(i) gives line i's echo_us."""
        t0 = time.monotonic()
        i = 0
        while i < n if until is None else not until():
            if time.monotonic() - t0 > timeout:
                raise TimeoutError('fed lines for too long')
            self.write(self.line(echo(i)))
            i += 1
            time.sleep(max(0.0, t0 + i * PERIOD_S - time.monotonic()))


def wait_until(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            raise TimeoutError('condition not met')
        time.sleep(0.01)


# ---------------------------------------------------------------- the node

class RecordingLogger:
    """Keep the node's log calls as (level, message), and print them to stderr."""

    # In-process, so nothing is lost: /rosout can drop under load, and polling pytest's capfd
    # loses a line written between its read and its truncate. Not passed on to rclpy, which
    # keys each call by its source line and refuses one line with several severities.
    LEVELS = {'debug': 'DEBUG', 'info': 'INFO', 'warning': 'WARN', 'warn': 'WARN',
              'error': 'ERROR', 'fatal': 'FATAL'}

    def __init__(self, logger, records):
        self.logger, self.records = logger, records

    def __getattr__(self, name):
        if name not in self.LEVELS:
            return getattr(self.logger, name)

        def log(message, **kwargs):
            self.records.append((self.LEVELS[name], message))
            print(f'[{self.LEVELS[name]}] {message}', file=sys.stderr)
        return log


class CountingNode(wave_probe_node.WaveProbeNode):
    status_calls = 0

    def __init__(self):
        self.records = []        # before super().__init__: the reader thread logs from the start
        super().__init__()

    def get_logger(self):
        return RecordingLogger(super().get_logger(), self.records)

    def log_status(self):
        self.status_calls += 1
        super().log_status()


class Probe:
    """The node under test, and a helper with a listener and a client."""

    # Each spins in an executor of its own, the node's as in main. Sharing one
    # MultiThreadedExecutor, the newer rclpy on the RPi4 busy-loops while the zero's callback
    # sleeps, and the reader thread starves (5 lines/s).

    def __init__(self, port):
        self.port = port
        self.node = CountingNode()
        self.helper = Node('test_helper')
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.ranges, self.elevations = [], []
        self.helper.create_subscription(Range, 'probe_range', self.ranges.append, qos)
        self.helper.create_subscription(
            SurfaceElevation, 'surface_elevation', self.elevations.append, qos)
        self.client = self.helper.create_client(ZeroMeasurement, 'zero_measurement')
        self.executors = [MultiThreadedExecutor(num_threads=4),   # as on the RPi4, any host
                          SingleThreadedExecutor()]
        for executor, node in zip(self.executors, [self.node, self.helper]):
            executor.add_node(node)
            threading.Thread(target=executor.spin, daemon=True).start()
        wait_until(lambda: self.node.pub.get_subscription_count() > 0
                   and self.node.elev_pub.get_subscription_count() > 0)
        assert self.client.wait_for_service(timeout_sec=5.0)

    def log_text(self, level=None):
        return [m for lv, m in list(self.node.records) if level in (None, lv)]

    def wait_log(self, cond, timeout=5.0):
        """Wait until cond(log texts) is true; fail with the log if it never is."""
        try:
            wait_until(lambda: cond(self.log_text()), timeout)
        except TimeoutError:
            pytest.fail(f'log never matched: {self.log_text()}')

    def settle(self, quiet_s=0.3, timeout=5.0):
        """Wait until no Range or elevation has arrived for quiet_s."""
        end = time.monotonic() + timeout
        last, since = None, time.monotonic()
        while time.monotonic() < end:
            now = (len(self.ranges), len(self.elevations))
            if now != last:
                last, since = now, time.monotonic()
            elif time.monotonic() - since >= quiet_s:
                return
            time.sleep(0.02)
        raise TimeoutError('messages kept coming')

    def zero(self, duration, feed=True, echo=lambda i: ECHO_US):
        """Call the zero service, and write lines until it answers (if feed)."""
        future = self.call_zero(duration)
        if feed:
            self.port.feed(echo=echo, until=future.done)
        return self.result(future)

    def call_zero(self, duration):
        return self.client.call_async(ZeroMeasurement.Request(duration=float(duration)))

    @staticmethod
    def result(future, timeout=20.0):
        wait_until(future.done, timeout)
        return future.result()

    def shutdown(self):
        self.node.stop()
        for executor in self.executors:
            executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def port(tmp_path):
    p = Port(tmp_path / 'probe')
    yield p
    try:
        p.disconnect()
    except OSError:
        pass


@pytest.fixture
def probe(port):
    rclpy.init(args=['--ros-args', '-p', f'port:={port.link}', '-p', f'reopen_s:={REOPEN_S}',
                     '-p', f'air_temp_c:={AIR_TEMP_C}'], domain_id=DOMAIN_ID)
    p = Probe(port)
    port.wait_open()
    yield p
    p.shutdown()
    rclpy.try_shutdown()


@pytest.fixture
def zeroed(probe):
    """Give a probe with a good zero, and its response."""
    res = probe.zero(1.0)
    assert res.success, res.message
    return probe, res


def r_m(echo_us=ECHO_US):
    return range_m(echo_us, AIR_TEMP_C)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ---------------------------------------------------------------- Range

def test_int_air_temp_starts(probe):
    assert probe.node.air_temp_c == 21.0 and isinstance(probe.node.air_temp_c, float)


def test_one_range_per_line(probe):
    probe.port.feed(20, lambda i: 0 if i == 10 else ECHO_US)
    probe.settle()
    assert len(probe.ranges) == 19                       # the first line after the open dropped
    values = [m.range for m in probe.ranges]
    assert values[9] == math.inf
    assert values[:9] + values[10:] == pytest.approx([r_m()] * 18)
    assert probe.ranges[0].radiation_type == Range.ULTRASOUND


def test_stamps_follow_the_lines(probe):
    probe.port.feed(20)
    probe.settle()
    stamps = [stamp_s(m) for m in probe.ranges]
    steps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert steps == pytest.approx([PERIOD_S] * len(steps), abs=0.01)
    now = probe.node.get_clock().now().nanoseconds * 1e-9
    assert now - 1.0 < stamps[-1] < now


def test_line_cut_by_read_timeout_is_dropped(probe):
    probe.port.feed(3)
    raw = probe.port.line()
    probe.port.write(raw[:5])
    time.sleep(wave_probe_node.READ_TIMEOUT_S + 0.1)
    probe.port.write(raw[5:])
    probe.port.feed(3)
    probe.settle()
    seqs = [round((stamp_s(m) - stamp_s(probe.ranges[0])) / PERIOD_S) for m in probe.ranges]
    assert seqs == [0, 1, 3, 4, 5]                       # neither half of line 4 got through


def test_no_echo_lines_stay_out_of_the_fit(probe):
    calls = []
    add = probe.node.clock.add
    probe.node.clock.add = lambda *a: calls.append(a) or add(*a)
    probe.port.feed(20, lambda i: ECHO_US if i % 2 else 0)   # line 0, dropped, has none
    probe.settle()
    assert len(probe.ranges) == 19
    assert len(calls) == 10                              # the echoes only


def test_gap_and_reset(probe):
    resets = []
    reset = probe.node.clock.reset
    probe.node.clock.reset = lambda: resets.append(1) or reset()
    probe.port.feed(5)
    probe.port.seq += 2                                  # two lines lost
    probe.port.feed(5)
    probe.port.seq = -1                                  # the board resets: seq 0 next
    probe.port.feed(5)
    probe.settle()
    probe.wait_log(lambda text: any('2 lines lost' in t for t in text))
    probe.wait_log(lambda text: any('restart' in t and 'seq 0' in t for t in text))
    assert resets == [1]


def test_reopen(probe):
    probe.port.feed(3)
    probe.port.disconnect()

    def warnings():
        link = str(probe.port.link)
        return [t for t in probe.log_text('WARN') if link in t]

    probe.wait_log(lambda text: warnings())
    time.sleep(3 * REOPEN_S)                             # several retries, all quiet
    assert len(warnings()) == 1, probe.log_text()
    probe.port.connect()
    t0 = time.monotonic()
    probe.port.wait_open(timeout=10.0)
    took = time.monotonic() - t0
    assert took < REOPEN_S + 0.5, (took, probe.log_text())
    n = len(probe.ranges)
    probe.port.feed(5)
    probe.settle()
    assert len(probe.ranges) == n + 4


def test_air_temp_param(probe):
    node = probe.node
    assert node.set_parameters([Parameter('air_temp_c', value=30.0)])[0].successful
    probe.port.feed(5)
    probe.settle()
    assert probe.ranges[-1].range / r_m() - 1 == pytest.approx(0.0159, abs=0.001)
    assert not node.set_parameters([Parameter('air_temp_c', value=80.0)])[0].successful
    assert not node.set_parameters([Parameter('port', value='/dev/null')])[0].successful
    assert node.air_temp_c == 30.0


def test_sigint_stops_within_2_s(port):
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN_ID))
    proc = subprocess.Popen(
        [sys.executable, str(SRC / 'wave_probe_node.py'), '--ros-args', '-p',
         f'port:={port.link}'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        port.wait_open(timeout=15.0)
        port.feed(5)
        t0 = time.monotonic()
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5.0)
        assert time.monotonic() - t0 < 2.0   # 1.3 s on the RPi4: the reader join takes up to 1 s
        assert b'Traceback' not in proc.stderr.read()
    finally:
        proc.kill()
        proc.wait()


# ---------------------------------------------------------------- the zero

def test_no_elevation_before_a_zero(probe):
    probe.port.feed(10)
    probe.settle()
    assert probe.ranges and not probe.elevations
    assert probe.node.zero == 0.0


def test_zero_constant_range(zeroed):
    probe, res = zeroed
    assert res.zero_measurement == pytest.approx(r_m())
    assert res.std == pytest.approx(0.0, abs=1e-9)
    assert 15 <= res.num_samples <= 25 and res.num_rejected_samples == 0
    assert probe.node.zero == res.zero_measurement
    probe.port.feed(5)
    probe.settle()
    assert [e.surface_elevation for e in probe.elevations[-4:]] == pytest.approx([0.0] * 4)
    by_stamp = {stamp_s(m): m.range for m in probe.ranges}
    for e in probe.elevations:
        assert stamp_s(e) in by_stamp                    # same stamp as its probe_range sample


def test_elevation_sign(zeroed):
    probe, res = zeroed
    lower = ECHO_US + 58                                 # the water 10 mm further away
    probe.port.feed(3, lambda i: lower)
    probe.settle()
    assert probe.elevations[-1].surface_elevation == pytest.approx(res.zero_measurement
                                                                   - r_m(lower))
    assert probe.elevations[-1].surface_elevation == pytest.approx(-0.010, abs=0.0005)


def test_no_echo_after_zero_is_nan(zeroed):
    probe, _ = zeroed
    probe.port.feed(3, lambda i: 0)
    probe.settle()
    assert all(math.isnan(e.surface_elevation) for e in probe.elevations[-3:])


def test_zero_too_many_rejected(zeroed):
    probe, old = zeroed
    res = probe.zero(1.0, echo=lambda i: 0 if i % 5 == 0 else ECHO_US + 100)
    assert not res.success
    assert res.num_samples + res.num_rejected_samples >= 15
    assert 'rejected' in res.message
    assert probe.node.zero == old.zero_measurement


def test_zero_noisy(zeroed):
    probe, old = zeroed
    res = probe.zero(1.0, echo=lambda i: ECHO_US + (NOISY_US if i % 2 else -NOISY_US))
    assert not res.success
    assert res.num_samples >= 15
    assert res.std == pytest.approx(0.006, abs=0.0005)
    assert 'not still' in res.message
    assert probe.node.zero == old.zero_measurement


def test_zero_second_call_refused(probe):
    first = probe.call_zero(1.0)
    time.sleep(0.2)
    second = probe.call_zero(1.0)
    probe.port.feed(until=lambda: first.done() and second.done())
    res2, res1 = probe.result(second), probe.result(first)
    assert not res2.success and 'already running' in res2.message
    assert res1.success, res1.message


@pytest.mark.parametrize('duration', [0.0, 500.0])
def test_zero_duration_refused(probe, duration):
    t0 = time.monotonic()
    res = probe.result(probe.call_zero(duration))
    assert not res.success and 'duration' in res.message
    assert time.monotonic() - t0 < 0.5


def test_zero_without_lines(probe):
    res = probe.zero(1.0, feed=False)
    assert not res.success and 'no samples' in res.message
    assert probe.node.zero == 0.0


def test_zero_is_not_a_parameter(zeroed):
    probe, _ = zeroed
    assert not probe.node.has_parameter('zero')
    with pytest.raises(ParameterNotDeclaredException):
        probe.node.set_parameters([Parameter('zero', value=0.3)])


def test_status_timer_runs_during_a_zero(probe):
    future = probe.call_zero(3.0)
    time.sleep(0.2)
    before = probe.node.status_calls
    probe.port.feed(until=future.done)
    assert probe.result(future).success
    assert probe.node.status_calls - before >= 2
