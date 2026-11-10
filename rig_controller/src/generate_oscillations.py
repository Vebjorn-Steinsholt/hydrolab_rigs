#!/usr/bin/env python3
"""
Generate oscillator motion files (.dat, run table .csv, record .txt, plot .png) for one oscillator.

Uses the limits from oscillator.yaml. The goal is compatibility: every file plays with
play_dat.py and every run is accepted by the controller on that oscillator.

The input is the slide's stroke; there is no wave physics, unlike generate_waves. Positions are
metres in stroke space, relative to upright.

This file holds what is about oscillators: the test matrix, the signals and the jerk warning.
Writing, assembling, checking and reading the file back, and the file contract with play_dat and
the controller, are in motion_files (shared with generate_waves).
"""

import argparse
import csv
from dataclasses import dataclass
import math
import os
import sys

import motion_files
import numpy as np
import play_dat
import rig_limits


def default_config_path() -> str:
    """
    Return the path of oscillator.yaml.

    From the installed oscillator_bringup share directory (ament_index_python), falling back to
    the copy in the repository.
    """
    return rig_limits.default_config_path('oscillator')


@dataclass
class RunSpec:
    """One row of the test matrix; the file order is the play order."""

    n_ramp: int        # periods (regular) or peak periods (irregular) of ramp
    pause_s: float     # rest after the run, >= play_dat.MIN_GAP_S (1 s)


@dataclass
class RegularSpec(RunSpec):
    """A sine of one stroke and period."""

    period_s: float
    stroke_m: float    # peak to peak
    n_periods: int     # including ramps


@dataclass
class IrregularSpec(RunSpec):
    """A JONSWAP-shaped random motion of one significant stroke."""

    sig_stroke_m: float  # 4 sigma of the position
    tp_s: float
    gamma: float         # 3.3 typical; 1.0 = Pierson-Moskowitz
    duration_s: float    # whole run, ramps included
    seed: int


def check_spec(spec: RunSpec, row: int):
    """Refuse a matrix row that cannot become a run; errors name the row and column."""
    def positive(key: str):
        if not getattr(spec, key) > 0.0:
            raise ValueError(f'row {row}: {key} must be > 0, not {getattr(spec, key)}')
    if spec.n_ramp < 1:
        raise ValueError(f'row {row}: n_ramp must be at least 1: a run starts and ends at rest')
    if spec.pause_s < play_dat.MIN_GAP_S:
        raise ValueError(f'row {row}: pause_s {spec.pause_s} is below {play_dat.MIN_GAP_S} s; '
                         'play_dat would join this run to the next')
    if isinstance(spec, RegularSpec):
        positive('period_s')
        positive('stroke_m')
        if spec.n_periods < 2*spec.n_ramp:
            raise ValueError(f'row {row}: n_periods {spec.n_periods} is shorter than the two '
                             f'ramps ({2 * spec.n_ramp} periods)')
    elif isinstance(spec, IrregularSpec):
        positive('sig_stroke_m')
        positive('tp_s')
        positive('gamma')
        positive('duration_s')
        if spec.duration_s < 2 * spec.n_ramp * spec.tp_s:
            raise ValueError(f'row {row}: duration_s {spec.duration_s} is shorter than the two '
                             f'ramps ({2 * spec.n_ramp * spec.tp_s} s)')


def read_test_matrix(path: str) -> list[RunSpec]:
    """
    Read the test matrix: CSV, one row per run, played in file order.

    Columns per kind (others left empty):
      all:        kind (default regular), n_ramp, pause_s
      regular:    period_s, stroke_m, n_periods
      irregular:  sig_stroke_m, tp_s, gamma (default 3.3), duration_s, seed
    """
    specs = []
    with open(path, newline='') as f:
        for row, r in enumerate(csv.DictReader(f)):
            kind = (r.get('kind') or 'regular').strip()
            common = {'n_ramp': motion_files.number(r, 'n_ramp', row, int),
                      'pause_s': motion_files.number(r, 'pause_s', row)}
            if kind == 'regular':
                specs.append(RegularSpec(
                    **common,
                    period_s=motion_files.number(r, 'period_s', row),
                    stroke_m=motion_files.number(r, 'stroke_m', row),
                    n_periods=motion_files.number(r, 'n_periods', row, int)))
            elif kind == 'irregular':
                specs.append(IrregularSpec(
                    **common,
                    sig_stroke_m=motion_files.number(r, 'sig_stroke_m', row),
                    tp_s=motion_files.number(r, 'tp_s', row),
                    gamma=float((r.get('gamma') or '').strip() or 3.3),
                    duration_s=motion_files.number(r, 'duration_s', row),
                    seed=motion_files.number(r, 'seed', row, int)))
            else:
                raise ValueError(f"row {row}: kind '{kind}' must be regular or irregular")
            check_spec(specs[-1], row)
    return specs


# ---------------------------------------------------------------- signals
# Sine or JONSWAP-shaped motion of the slide, cosine ramps at both ends.

F_MIN_FACTOR = 0.5   # band: F_MIN_FACTOR * fp .. F_MAX_FACTOR * fp, as the wavemaker's
F_MAX_FACTOR = 3.0


def irregular_signal(spec: IrregularSpec, dt: float):
    """Positions relative to upright for one irregular run, and its run-table fields."""
    fp = 1.0 / spec.tp_s
    n, f, df, n_samples = motion_files.frequency_grid(
        spec.duration_s, dt, F_MIN_FACTOR * fp, F_MAX_FACTOR * fp)
    spectrum = motion_files.jonswap(f, df, spec.sig_stroke_m, spec.tp_s, spec.gamma)
    a = motion_files.wave_amplitudes(spectrum, df)
    x = motion_files.synthesize(n, a, motion_files.random_phases(spec.seed, len(f)), n_samples)
    x = motion_files.apply_ramps(x, round(spec.n_ramp * spec.tp_s / dt), shape='cosine')
    row = {'kind': 'irregular', 'peak_period_s': spec.tp_s, 'sig_stroke_m': spec.sig_stroke_m,
           'gamma': spec.gamma, 'seed': spec.seed,
           'f_min_hz': round(float(f[0]), 4), 'f_max_hz': round(float(f[-1]), 4),
           'std_m': round(float(x.std()), 5), 'peak_m': round(float(np.abs(x).max()), 5)}
    return x, row


def regular_signal(spec: RegularSpec, dt: float):
    """Positions relative to upright for one regular run, and its run-table fields."""
    amplitude = spec.stroke_m / 2.0
    n = round(spec.n_periods * spec.period_s / dt)
    t = np.arange(1, n + 1) * dt
    x = amplitude * np.sin(2.0 * math.pi / spec.period_s * t)
    x = motion_files.apply_ramps(x, round(spec.n_ramp * spec.period_s / dt), shape='cosine')
    row = {'kind': 'regular', 'period_s': spec.period_s, 'stroke_m': spec.stroke_m,
           'amplitude_m': round(amplitude, 5)}
    return x, row


def peak_jerk(x, dt: float) -> float:
    """Return max |third difference| / dt^3 [m/s^3], from the samples as play_dat reads them."""
    return float(np.abs(np.diff(x, 3)).max() / dt ** 3)


# ---------------------------------------------------------------- assemble and write
# The shared parts are in motion_files.


OSCILLATION_COLUMNS = ['run', 'start_s', 'end_s', 'kind',
                       'period_s', 'stroke_m', 'amplitude_m',                    # regular
                       'peak_period_s', 'sig_stroke_m', 'gamma', 'seed',         # irregular
                       'f_min_hz', 'f_max_hz', 'std_m', 'peak_m',
                       'peak_jerk_mps3', 'status']


def run_signal(spec: RunSpec, dt: float):
    """(x, row, pause_s) for one spec of the test matrix, as motion_files.assemble takes it."""
    if isinstance(spec, RegularSpec):
        x, row = regular_signal(spec, dt)
    elif isinstance(spec, IrregularSpec):
        x, row = irregular_signal(spec, dt)
    else:
        raise TypeError(f'unknown spec {type(spec).__name__}')
    row['peak_jerk_mps3'] = round(peak_jerk(x, dt), 3)
    return x, row, spec.pause_s


def assemble(limits: rig_limits.RigLimits, specs: list[RunSpec], dt: float,
             max_velocity_mps: float):
    """Frames (n x 2: position, trigger) and run-table rows, in file order."""
    runs = [run_signal(spec, dt) for spec in specs]
    return motion_files.assemble(
        runs, dt, lambda x: motion_files.check_run(limits, x, dt, max_velocity_mps))


def write_run_table(path: str, rows: list[dict]):
    """Run table for play_dat: the columns of both kinds; a column a row lacks is left empty."""
    motion_files.write_run_table(path, rows, OSCILLATION_COLUMNS)


def write_record(path: str, argv: list[str], args, limits: rig_limits.RigLimits,
                 config: str, max_velocity_mps: float, summary: str):
    """<output>.txt: what the file was made from, to make it again."""
    speed = f'{max_velocity_mps:.4f} m/s'
    if args.max_velocity_mps:
        speed += ' (--max-velocity-mps)'
    motion_files.write_record(
        path, 'generate_oscillations', argv, config,
        [f'oscillator   {limits.name}',
         f'max jerk     {args.max_jerk_mps3} m/s^3 (warning only)'],
        args.dt, speed, summary, args.test_matrix)


# ---------------------------------------------------------------- command line

def main(argv=None):
    """Parse the arguments, write the four files and print the play_dat command."""
    parser = argparse.ArgumentParser(description='Generate oscillator motion files for play_dat.')
    parser.add_argument('oscillator', help='name in oscillator.yaml (vertical, sloshing)')
    parser.add_argument('test_matrix', help='CSV, one row per run (read_test_matrix)')
    parser.add_argument('-o', '--output', required=True,
                        help='base name for .dat, .csv, .txt and .png')
    parser.add_argument('--config', help='another oscillator.yaml; default the installed one')
    parser.add_argument('--dt', type=float, default=0.02, help='sample interval [s], whole ms')
    parser.add_argument('--max-velocity-mps', type=float,
                        help="override the oscillator's speed limit from the yaml")
    parser.add_argument('--max-jerk-mps3', type=float, default=3.0,
                        help="warn above this jerk [m/s^3]; the drive's S-0-0193 is 3.0")
    parser.add_argument('--no-plot', action='store_true', help='do not write <output>.png')
    parser.add_argument('--show', action='store_true', help='also open the plot in a window')
    argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(argv)

    if not os.path.isfile(args.test_matrix):
        parser.error(f'test matrix {args.test_matrix} not found (relative to {os.getcwd()})')
    try:
        config = args.config or default_config_path()
    except FileNotFoundError as e:
        parser.error(str(e))
    try:
        specs = read_test_matrix(args.test_matrix)
    except ValueError as e:
        parser.error(str(e))

    limits = rig_limits.load_limits('oscillator', args.oscillator, config)
    max_velocity = args.max_velocity_mps or limits.max_velocity_mps

    frames, rows = assemble(limits, specs, args.dt, max_velocity)
    for row in rows:
        if row['status'] != 'skipped' and row['peak_jerk_mps3'] > args.max_jerk_mps3:
            print(f"run {row['run']}: peak jerk {row['peak_jerk_mps3']} m/s^3 above "
                  f'{args.max_jerk_mps3}: the drive rounds the motion off')

    dat_path = args.output + '.dat'
    motion_files.write_motion(dat_path, frames, args.dt)
    write_run_table(args.output + '.csv', rows)
    played = motion_files.verify_with_play_dat(dat_path, limits, max_velocity)
    summary = (f'{played} runs, {len(rows) - played} skipped, '
               f'{len(frames) * args.dt / 60:.1f} min')
    write_record(args.output + '.txt', argv, args, limits, config, max_velocity, summary)
    if not args.no_plot:
        motion_files.plot_motion(args.output + '.png', limits, frames, rows, args.dt,
                                 max_velocity, args.show)
    print(f'{dat_path}: {summary}')
    command = (f'ros2 run rig_controller play_dat {dat_path} {limits.name} '
               '--rig oscillator --dry-run')

    if args.max_velocity_mps:  # check against the same speed the file was made for
        command += f' --max-velocity-mps {max_velocity:.4f}'
    if args.config is not None:
        command += f' --config {args.config}'
    print(command)

    return 0


if __name__ == '__main__':
    sys.exit(main())
