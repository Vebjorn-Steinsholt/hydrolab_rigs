#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
import math
import re

import numpy as np

WRAP = 2**32              # micros() and seq are unsigned long on the Arduino
PERIOD_US = 50_000        # the firmware's PERIOD_US (20 Hz)
_LINE = re.compile(rb'(\d{1,10}),(\d{1,10}),(\d{1,6})')


@dataclass(frozen=True)
class Line:
    seq: int
    t_us: int
    echo_us: int


def parse_line(raw: bytes) -> Line | None:
    m = _LINE.fullmatch(raw.rstrip(b'\r\n'))
    if m is None:
        return None
    seq, t_us, echo_us = (int(g) for g in m.groups())
    if seq >= WRAP or t_us >= WRAP:
        return None
    return Line(seq, t_us, echo_us)


def speed_of_sound(air_temp_c: float) -> float:
    """Return the speed of sound in m/s at the given air temperature in Celsius."""
    return 331.3 + 0.606 * air_temp_c


def range_m(echo_us: int, air_temp_c: float) -> float:
    if echo_us == 0:
        return math.inf
    return echo_us * 1e-6 * speed_of_sound(air_temp_c) / 2


@dataclass(frozen=True)
class Step:
    kind: str
    t_s: float
    lost: int = 0


class Sequence:
    def __init__(self, period_us=PERIOD_US, tol_us=1_000, max_bad=3):
        self.period_us, self.tol_us, self.max_bad = period_us, tol_us, max_bad
        self.reset()

    def reset(self):
        self.prev: Line | None = None
        self.wraps = 0
        self.bad_run = 0

    def push(self, line: Line) -> Step:
        if self.prev is None:
            return self._start(line, 'first')
        dseq = line.seq - self.prev.seq
        if dseq <= 0:
            return self._start(line, 'reset')
        dt_us = (line.t_us - self.prev.t_us) % WRAP
        if abs(dt_us - dseq * self.period_us) > self.tol_us:
            self.bad_run += 1
            if self.bad_run >= self.max_bad:
                return self._start(line, 'reset')
            return Step('bad', math.nan)
        if line.t_us < self.prev.t_us:
            self.wraps += 1
        self.prev, self.bad_run = line, 0
        t_s = (self.wraps * WRAP + line.t_us) * 1e-6
        return Step('gap', t_s, dseq - 1) if dseq > 1 else Step('ok', t_s)

    def _start(self, line: Line, kind: str) -> Step:
        self.reset()
        self.prev = line
        return Step(kind, line.t_us * 1e-6)


class ClockFit:
    def __init__(self, window_s=300.0, block_s=10.0, latency_floor_s=0.002,
                 jump_warn_s=0.001):
        self.block_s = block_s
        self.latency_floor_s = latency_floor_s
        self.jump_warn_s = jump_warn_s
        self.minima = deque(maxlen=max(2, round(window_s / block_s)))
        self.reset()

    def reset(self):
        self.minima.clear()
        self.block = None
        self.low = None

        self.t_first = None
        self.d_ref = None
        self.eps, self.c = 0.0, 0.0
        self.rms = math.nan

    @property
    def ready(self) -> bool:
        return len(self.minima) >= 2

    def add(self, t_send: float, arrival: float) -> float | None:
        if self.t_first is None or self.d_ref is None:
            self.t_first, self.d_ref = t_send, arrival - t_send

        d = arrival - t_send - self.d_ref
        k = int((t_send - self.t_first) // self.block_s)
        jump = None
        if self.block is not None and k != self.block:
            was_ready = self.ready
            before = self.to_ros(t_send)
            self.minima.append(self.low)
            self.low = None
            if self.ready:
                self._fit()
                shift = self.to_ros(t_send) - before
                if was_ready and abs(shift) > self.jump_warn_s:
                    jump = shift
        self.block = k
        if self.low is None or d < self.low[1]:
            self.low = (t_send, d)
        if not self.ready:
            self.c = min([m[1] for m in self.minima] + [self.low[1]])
        return jump

    def _fit(self):
        t, d = np.array(self.minima).T
        eps, c = np.polyfit(t, d, 1)
        self.eps, self.c = float(eps), float(c)
        self.rms = float(np.sqrt(np.mean((d - (c + eps * t)) ** 2)))

    def to_ros(self, t_arduino: float) -> float:
        if self.d_ref is None:
            raise RuntimeError('ClockFit.to_ros before the first add')
        return t_arduino + self.d_ref + self.c + self.eps * t_arduino - self.latency_floor_s

    def status(self) -> dict:
        return {'ready': self.ready, 'blocks': len(self.minima),
                'span_s': len(self.minima) * self.block_s,
                'eps_ppm': self.eps * 1e6, 'rms_s': self.rms,
                'floor_s': self.latency_floor_s}
