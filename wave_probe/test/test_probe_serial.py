"""
Tests for src/probe_serial.py, without ROS.

Covers the line parser, the range, the seq/t_us bookkeeping and the clock fit.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest wave_probe/test/test_probe_serial.py -v
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from probe_serial import ClockFit, Line, parse_line, range_m, Sequence, WRAP  # noqa: E402, I100

PERIOD_S = 0.05
EPS = 300e-6             # the simulated Arduino clock's rate error
FLOOR_S = 0.002          # the simulated latency floor, = ClockFit's default latency_floor_s


# ---------------------------------------------------------------- parse_line

def test_parse_line():
    assert parse_line(b'12,3456,5824\r\n') == Line(12, 3456, 5824)


@pytest.mark.parametrize('raw', [b'3456,5824\r\n', b'Distance: 12\r\n', b'', b'1,-2,3\r\n',
                                 b'1,2,3,4\r\n'])
def test_parse_line_rejects(raw):
    assert parse_line(raw) is None


# ---------------------------------------------------------------- range_m

def test_range_m():
    assert range_m(5824, 20.0) == pytest.approx(1.000, abs=1e-4)


def test_range_m_no_echo():
    assert range_m(0, 20.0) == math.inf


def test_range_m_temperature():
    assert range_m(5824, 30.0) / range_m(5824, 20.0) - 1 == pytest.approx(0.018, abs=0.001)


# ---------------------------------------------------------------- Sequence

def push_all(lines):
    seq = Sequence()
    return seq, [seq.push(Line(*line)) for line in lines]


def test_sequence_wrap():
    t0 = WRAP - 120_000
    seq, steps = push_all([(i, (t0 + i * 50_000) % WRAP, 1000) for i in range(6)])
    assert [s.kind for s in steps] == ['first'] + ['ok'] * 5
    t = [s.t_s for s in steps]
    assert np.allclose(np.diff(t), PERIOD_S)
    assert seq.wraps == 1


def test_sequence_gap():
    _, steps = push_all([(1, 0, 1000), (4, 150_000, 1000)])
    assert (steps[1].kind, steps[1].lost) == ('gap', 2)


def test_sequence_reset():
    _, steps = push_all([(5, 0, 1000), (6, 50_000, 1000), (0, 7, 1000)])
    assert steps[2].kind == 'reset'


def test_sequence_bad_then_gap():
    _, steps = push_all([(1, 0, 1000), (2, 52_000, 1000), (3, 100_000, 1000)])
    assert steps[1].kind == 'bad'
    assert (steps[2].kind, steps[2].lost) == ('gap', 1)   # the bad line counts as lost


def test_sequence_backwards_is_bad_not_wrap():
    seq, steps = push_all([(1, 1_000_000, 1000), (2, 500_000, 1000)])
    assert steps[1].kind == 'bad'
    assert seq.wraps == 0


def test_sequence_corrupt_reference():
    lines = [(1, 7_777, 1000)] + [(i, i * 50_000, 1000) for i in range(2, 6)]
    _, steps = push_all(lines)
    assert [s.kind for s in steps] == ['first', 'bad', 'bad', 'reset', 'ok']


# ---------------------------------------------------------------- ClockFit

def simulate(fit, duration_s, offset_s=1000.0, step_at_s=None, step_s=0.0, seed=1):
    """Feed fit; return (t_send, error of to_ros against the truth, jump) per line."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(round(duration_s / PERIOD_S)):
        t_send = i * PERIOD_S + rng.uniform(1e-3, 6e-3)        # trigger + an echo of 1-6 ms
        latency = rng.exponential(3e-3) + (0.030 if rng.random() < 0.01 else 0.0)
        truth = t_send * (1 + EPS) + offset_s
        step = step_s if step_at_s is not None and t_send > step_at_s else 0.0
        jump = fit.add(t_send, truth + FLOOR_S + latency + step)
        out.append((t_send, fit.to_ros(t_send) - truth - step, jump))
    return out


@pytest.mark.parametrize('offset_s', [1000.0, 1.8e9])   # 1.8e9: ROS time today
def test_clock_fit_converges(offset_s):
    fit = ClockFit()
    out = simulate(fit, 300.0 + PERIOD_S, offset_s)
    late = [abs(e) for t, e, _ in out if t > 290.0]
    assert max(late) < 0.5e-3
    assert fit.status()['eps_ppm'] == pytest.approx(EPS * 1e6, abs=20)


@pytest.mark.parametrize('offset_s', [1000.0, 1.8e9])
def test_clock_fit_early(offset_s):
    out = simulate(ClockFit(), 20.0, offset_s)
    for t, e, _ in out:
        assert abs(e) <= EPS * t + 5e-3


def test_clock_fit_steady_has_no_jump():
    out = simulate(ClockFit(), 600.0)
    assert [j for _, _, j in out if j is not None] == []


def test_clock_fit_step_gives_jump():
    # Each refit moves the stamps by about 13 % of a step: 20 ms gives 2.7 ms, past
    # jump_warn_s (1 ms); 5 ms gives 0.6 ms and no jump.
    out = simulate(ClockFit(), 400.0, step_at_s=300.0, step_s=0.020)
    jumps = [j for t, _, j in out if j is not None]
    assert jumps and all(t > 300.0 for t, _, j in out if j is not None)
    assert jumps[0] > 0.001


def test_clock_fit_reset():
    fit = ClockFit()
    simulate(fit, 60.0)
    assert fit.ready
    fit.reset()
    assert not fit.ready and fit.d_ref is None
    with pytest.raises(RuntimeError):
        fit.to_ros(0.0)
    assert simulate(fit, 60.0, offset_s=5000.0)[-1][1] == pytest.approx(0.0, abs=5e-3)
