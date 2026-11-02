#!/usr/bin/env python3
"""
Write a small motion file (.dat) and its run table (.csv) for trying play_dat.

A test tool, not the wave generator: the runs are plain ramped sines with a fixed paddle
amplitude, no wave theory or gain. The layout is the one play_dat reads: int32 step in ms,
int32 channel count, interleaved little-endian float32; channel 0 is the position in metres
relative to upright, channel 1 the trigger (1 while a run plays).

    python3 rig_controller/test/make_test_dat.py /tmp/test_runs.dat
    python3 rig_controller/test/make_test_dat.py /tmp/test_runs.dat --runs 3 --no-table

Defaults: 6 runs of 8 periods (0.8, 1.0, 1.2, 1.5, 0.8, 1.0 s), 0.02 m amplitude, 2 s lead-in,
3 s between runs, 0.01 s step; about 68 s in all, peak speed 0.157 m/s.
"""

import argparse
import csv
import os
import sys

import numpy as np

PERIODS_S = (0.8, 1.0, 1.2, 1.5)
RAMP_PERIODS = 2


def regular_run(period_s, amplitude_m, periods, dt_s):
    """Return a sin^2-ramped sine of whole periods that starts and ends at exactly 0."""
    n = int(round(periods * period_s / dt_s))
    t_s = np.arange(n + 1) * dt_s
    ramp_s = RAMP_PERIODS * period_s
    envelope = np.sin(np.pi / 2 * np.clip(np.minimum(t_s, t_s[-1] - t_s) / ramp_s, 0, 1)) ** 2
    x = amplitude_m * envelope * np.sin(2 * np.pi * t_s / period_s)
    x[0] = x[-1] = 0.0
    return x


def build(runs, amplitude_m, periods, lead_in_s, gap_s, dt_s):
    """Return the position and trigger channels and one run table row per run."""
    position, trigger, rows = [np.zeros(int(round(lead_in_s / dt_s)))], [], []
    trigger.append(np.zeros_like(position[0]))
    t_s = lead_in_s
    for index in range(runs):
        period_s = PERIODS_S[index % len(PERIODS_S)]
        x = regular_run(period_s, amplitude_m, periods, dt_s)
        rows.append({'run': index, 'start_s': f'{t_s:.2f}',
                     'end_s': f'{t_s + len(x) * dt_s:.2f}', 'kind': 'regular',
                     'period_s': period_s, 'amplitude_m': amplitude_m, 'status': 'ok'})
        position.append(x)
        trigger.append(np.ones_like(x))
        t_s += len(x) * dt_s
        if index < runs - 1:
            rest = np.zeros(int(round(gap_s / dt_s)))
            position.append(rest)
            trigger.append(rest)
            t_s += len(rest) * dt_s
    return np.concatenate(position), np.concatenate(trigger), rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('path', help='output .dat; the run table goes next to it as .csv')
    parser.add_argument('--runs', type=int, default=6)
    parser.add_argument('--amplitude-m', type=float, default=0.02)
    parser.add_argument('--periods', type=int, default=8, help='whole periods per run')
    parser.add_argument('--lead-in-s', type=float, default=2.0)
    parser.add_argument('--gap-s', type=float, default=3.0)
    parser.add_argument('--dt-s', type=float, default=0.01)
    parser.add_argument('--no-table', action='store_true', help='do not write the .csv')
    args = parser.parse_args(argv)

    step_ms = round(args.dt_s * 1000)
    if step_ms <= 0 or abs(step_ms - args.dt_s * 1000) > 1e-9:
        parser.error('--dt-s must be a whole number of ms')
    position, trigger, rows = build(args.runs, args.amplitude_m, args.periods,
                                    args.lead_in_s, args.gap_s, args.dt_s)
    with open(args.path, 'wb') as f:
        np.array([step_ms, 2], dtype='<i4').tofile(f)
        np.column_stack([position, trigger]).astype('<f4').tofile(f)
    print(f'{args.path}: {len(rows)} runs, {len(position) * args.dt_s:.1f} s, '
          f'max |x| {np.abs(position).max():.3f} m, '
          f'peak speed {np.abs(np.diff(position)).max() / args.dt_s:.3f} m/s')

    table = os.path.splitext(args.path)[0] + '.csv'
    if args.no_table:
        if os.path.exists(table):
            os.remove(table)  # an old table would no longer match the file
        return 0
    with open(table, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'{table}: run table')
    return 0


if __name__ == '__main__':
    sys.exit(main())
