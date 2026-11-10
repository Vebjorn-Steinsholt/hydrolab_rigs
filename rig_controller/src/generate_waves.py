#!/usr/bin/env python3
"""
Generate wave motion files (.dat, run table .csv, record .txt, plot .png) for one wavemaker.

Uses the wavemaker's geometry and limits from wavemakers.yaml. The goal is compatibility: every
file plays with play_dat.py and every run is accepted by the controller on that wavemaker.

The wave physics is the controller's (wave_math.cpp, WaveTrajectory::configure), so a run in a
file and the same H and T sent as a regular goal give the same stroke. Positions are metres at
the attachment point, relative to upright, with the transfer gain applied.

This file holds what is about waves: the wavemaker, the test matrix, the wave physics and the
signals. Writing, assembling, checking and reading the file back, and the file contract with
play_dat and the controller, are in motion_files (shared with generate_oscillations).
"""
import argparse
import csv
from dataclasses import dataclass
import math
import os
import sys

import motion_files
from motion_files import (  # noqa: F401  re-exported: test_generate_waves uses gw.<name>
    apply_ramps, check_run, frequency_grid, G, jonswap, number, random_phases, SIGMA_1,
    SIGMA_2, source_commit, synthesize, synthesize_direct, verify_with_play_dat,
    wave_amplitudes, write_motion)
import numpy as np
import rig_limits

# ---------------------------------------------------------------- wavemaker from the yaml


@dataclass
class Wavemaker:
    """What generation needs from /wavemakers/<name>/controller: ros__parameters."""

    name: str
    type: str  # noqa: A003  wavemaker_type: 'flap' | 'piston'
    water_depth: float
    hinge_height: float      # flap only
    attachment_height: float  # flap_attachment_height, flap only
    gain: float
    gain_periods: list       # wavemaker_transfer_gain_periods (optional table)
    gain_values: list
    min_m: float             # absolute position limits, as the controller derives them
    max_m: float
    upright_m: float         # upright_position_m
    max_velocity_mps: float
    max_acceleration_mps2: float | None  # max_goal_acceleration_mps2; None when 0 (off)


def default_config_path() -> str:
    """
    Return the path of wavemakers.yaml.

    From the installed wavemaker_bringup share directory (ament_index_python), falling back to
    the copy in the repository.
    """
    return rig_limits.default_config_path('wavemaker')


def load_wavemaker(name: str, config_path: str) -> Wavemaker:
    """
    Load the wavemaker's geometry, gain and limits.

    Geometry and gain come from its yaml block, the limits from rig_limits (the same derivation
    play_dat uses). Keys that set the geometry are required, as the limits are.
    """
    params = rig_limits.read_block('wavemaker', name, config_path)
    source = os.path.basename(config_path)
    limits = rig_limits.limits_from_parameters('wavemaker', name, params, source)

    wavemaker_type = rig_limits.required(params, 'wavemaker_type', name, source)
    if wavemaker_type == 'flap':
        attachment_height = rig_limits.required(params, 'flap_attachment_height', name, source)
    else:
        attachment_height = 0.0  # not used for piston

    return Wavemaker(
        name=name,
        type=wavemaker_type,
        water_depth=rig_limits.required(params, 'water_depth', name, source),
        hinge_height=params.get('hinge_height', 0.0),  # optional, default 0.0
        attachment_height=attachment_height,
        gain=params.get('wavemaker_transfer_gain', 1.0),  # optional, default 1.0
        gain_periods=list(params.get('wavemaker_transfer_gain_periods', [])),  # optional table
        gain_values=list(params.get('wavemaker_transfer_gain_values', [])),
        min_m=limits.min_m,
        max_m=limits.max_m,
        upright_m=limits.upright_m,
        max_velocity_mps=limits.max_velocity_mps,
        max_acceleration_mps2=limits.max_acceleration_mps2)


def transfer_gain(wm: Wavemaker, period_s: float) -> float:
    """
    Return the transfer gain for a period: the table if set, else wm.gain.

    As interpolate_gain() in wave_math.cpp: linear between points, nearest end value outside
    (np.interp does both).

    Test, ladertanken: 1.05 s -> 0.755; 0.5 s -> 0.64; 2.5 s -> 0.76.
    """
    if wm.gain_periods:
        return float(np.interp(period_s, wm.gain_periods, wm.gain_values))
    return wm.gain


# ---------------------------------------------------------------- wave physics (as the controller)

def wavenumber_depth(period_s: float, depth: float) -> float:
    """
    Return k*h with the controller's Beji 2013 approximation (solve_dispersion).

    With g = 9.81:
      mu0  = omega^2 h / g
      mu_a = mu0 / sqrt(tanh(mu0))
      kh   = mu_a * (1 + mu0^1.09 * exp(-(1.55 + 1.30 mu0 + 0.216 mu0^2)))
    Not an exact solver: files and live goals must agree. An exact Newton solution of
    omega^2 = g k tanh(kh) is within ~0.03 % here; use it in a test with tolerance ~1e-3.

    Test: T 1.0 s, h 1.0 m -> 4.0262; T 1.5 s -> 1.8753.
    """
    omega = 2.0 * math.pi / period_s
    mu0 = omega * omega * depth / G
    mu_a = mu0 / math.sqrt(math.tanh(mu0))
    return mu_a * (1.0 + mu0 ** 1.09 * math.exp(-(1.55 + 1.30 * mu0 + 0.216 * mu0 * mu0)))


def stroke_ratio(kh: float, wm: Wavemaker) -> float:
    """
    Return H/S at the still-water level, as compute_stroke() in wave_math.cpp.

      piston: 4 sinh^2(kh) / (sinh(2kh) + 2kh)
      flap, hm = kh * hinge_height / water_depth:
        4 sinh(kh) [sinh(kh) + (cosh(hm) - cosh(kh)) / (kh - hm)] / (sinh(2kh) + 2kh)

    Test: T 1.0 s, h 1.0 m: piston 1.9885, flap at the floor 1.5120.
    """
    if wm.type == 'piston':
        return 4.0 * math.sinh(kh) ** 2 / (math.sinh(2.0 * kh) + 2.0 * kh)
    hm = kh * wm.hinge_height / wm.water_depth
    bracket = math.sinh(kh) + (math.cosh(hm) - math.cosh(kh)) / (kh - hm)
    return 4.0 * math.sinh(kh) * bracket / (math.sinh(2.0 * kh) + 2.0 * kh)


def paddle_amplitude(wm: Wavemaker, period_s: float, height_m: float) -> float:
    """
    Return the amplitude at the attachment point, as WaveTrajectory::configure.

    In this order:
      stroke = H / stroke_ratio / gain     (the gain divides)
      amplitude = stroke / 2
      flap: amplitude *= (attachment_height - hinge) / (water_depth - hinge)

    Test, ladertanken:
      T 1.0 s, H 0.05 m  -> 0.0283 m
      T 1.5 s, H 0.10 m  -> 0.0955 m
      T 1.3 s, H 0.171 m -> 0.1315 m (+-248 deg, 200 rpm, as the yaml comment of 2026-10-20)
    """
    kh = wavenumber_depth(period_s, wm.water_depth)
    amplitude = height_m / stroke_ratio(kh, wm) / transfer_gain(wm, period_s) / 2.0
    if wm.type == 'flap':
        hinge = wm.hinge_height
        amplitude *= (wm.attachment_height - hinge) / (wm.water_depth - hinge)
    return amplitude


# ---------------------------------------------------------------- test matrix

@dataclass
class RunSpec:
    """One row of the test matrix; the file order is the play order."""

    n_ramp: int                       # periods (regular) or peak periods (irregular) of ramp
    pause_s: float                    # rest after the run, >= MIN_GAP_S (1 s)


@dataclass
class RegularSpec(RunSpec):
    period_s: float
    n_periods: int     # including ramps
    height_m: float | None = None     # exactly one of height_m and steepness
    steepness: float | None = None    # x in H/lambda = 1/x, e.g. 60 for 1/60


@dataclass
class IrregularSpec(RunSpec):
    hs_m: float
    tp_s: float
    gamma: float        # 3.3 typical; 1.0 = Pierson-Moskowitz
    duration_s: float   # whole run, ramps included
    seed: int


def parse_steepness(text: str, row: int) -> float:
    """'1/60' -> 60. Only the 1/x form, so a bare 60 (or 0.0167) cannot be misread."""
    numerator, slash, denominator = text.partition('/')
    try:
        x = float(denominator)
    except ValueError:
        x = 0.0
    if numerator.strip() != '1' or not slash or not x > 1.0:
        raise ValueError(f"row {row}: steepness '{text}' must be written 1/x, e.g. 1/60")
    return x


def read_test_matrix(path: str) -> list[RunSpec]:
    """
    Read the test matrix: CSV, one row per run, played in file order.

    Columns per kind (others left empty):
      all:        kind (default regular), n_ramp, pause_s
      regular:    period_s, height_m or steepness (1/x), n_periods
      irregular:  hs_m, tp_s, gamma (default 3.3), duration_s, seed
    """
    specs = []
    with open(path, newline='') as f:
        for row, r in enumerate(csv.DictReader(f)):
            kind = (r.get('kind') or 'regular').strip()
            common = {'n_ramp': number(r, 'n_ramp', row, int),
                      'pause_s': number(r, 'pause_s', row)}
            if kind == 'regular':
                height = (r.get('height_m') or '').strip()
                steepness = (r.get('steepness') or '').strip()
                if bool(height) == bool(steepness):
                    raise ValueError(f'row {row}: give height_m or steepness, not both or neither')
                specs.append(RegularSpec(
                    **common,
                    period_s=number(r, 'period_s', row),
                    height_m=float(height) if height else None,
                    steepness=parse_steepness(steepness, row) if steepness else None,
                    n_periods=number(r, 'n_periods', row, int)))

            elif kind == 'irregular':
                specs.append(IrregularSpec(
                    **common,
                    hs_m=number(r, 'hs_m', row),
                    tp_s=number(r, 'tp_s', row),
                    gamma=float((r.get('gamma') or '').strip() or 3.3),
                    duration_s=number(r, 'duration_s', row),
                    seed=number(r, 'seed', row, int)))
            else:
                raise ValueError(f"row {row}: kind '{kind}' must be regular or irregular")
    return specs


def wavelength(wm: Wavemaker, period_s: float) -> float:
    """Return the wavelength 2 pi h / kh, with the controller's dispersion approximation."""
    return 2.0 * math.pi * wm.water_depth / wavenumber_depth(period_s, wm.water_depth)


def target_height(wm: Wavemaker, spec: RegularSpec) -> float:
    """H as given, or lambda / x for a steepness 1/x."""
    if spec.steepness is not None:
        return wavelength(wm, spec.period_s) / spec.steepness
    assert spec.height_m is not None
    return spec.height_m


# ---------------------------------------------------------------- signals
# ---------------------------------------------------------------- irregular waves
# JONSWAP spectrum, random phases, ramps.
F_MIN_FACTOR = 0.5   # band: F_MIN_FACTOR * fp .. F_MAX_FACTOR * fp (README, Irregular waves)
F_MAX_FACTOR = 3.0


def paddle_amplitudes(wm: Wavemaker, f, a):
    """
    Return the paddle amplitude of each component.

    Each as a regular wave of height 2 a_n: same transfer function, gain and scaling.
    """
    return np.array([paddle_amplitude(wm, 1.0 / fn, 2.0 * an) for fn, an in zip(f, a)])


def irregular_signal(wm: Wavemaker, spec: IrregularSpec, dt: float):
    """Positions relative to upright for one irregular run, and its run-table fields."""
    fp = 1.0 / spec.tp_s
    n, f, df, n_samples = frequency_grid(spec.duration_s, dt,
                                         F_MIN_FACTOR * fp, F_MAX_FACTOR * fp)
    a = wave_amplitudes(jonswap(f, df, spec.hs_m, spec.tp_s, spec.gamma), df)
    p = paddle_amplitudes(wm, f, a)
    x = synthesize(n, p, random_phases(spec.seed, len(f)), n_samples)
    x = apply_ramps(x, round(spec.n_ramp * spec.tp_s / dt))
    row = {'kind': 'irregular', 'peak_period_s': spec.tp_s, 'hs_m': spec.hs_m,
           'gamma': spec.gamma, 'seed': spec.seed,
           'f_min_hz': round(float(f[0]), 4), 'f_max_hz': round(float(f[-1]), 4),
           'paddle_std_m': round(float(x.std()), 5),
           'paddle_peak_m': round(float(np.abs(x).max()), 5)}
    return x, row


# ---------------------------------------------------------------- regular waves
# Sine wave, ramps, paddle amplitude from the controller's physics.

def regular_signal(wm: Wavemaker, spec: RegularSpec, dt: float):
    """Positions relative to upright for one run, and its run-table fields."""
    height = target_height(wm, spec)
    amplitude = paddle_amplitude(wm, spec.period_s, height)
    n = round(spec.n_periods * spec.period_s / dt)
    t = np.arange(1, n + 1) * dt
    x = amplitude * np.sin(2.0 * math.pi / spec.period_s * t)
    x = apply_ramps(x, round(spec.n_ramp * spec.period_s / dt))
    row = {'kind': 'regular', 'period_s': spec.period_s, 'height_m': round(height, 5),
           'steepness': f'1/{wavelength(wm, spec.period_s) / height:.1f}',
           'gain': round(transfer_gain(wm, spec.period_s), 4),
           'amplitude_m': round(amplitude, 5)}
    return x, row


# ---------------------------------------------------------------- assemble and write
# The shared parts are in motion_files; these keep the wavemaker's signatures.

WAVE_COLUMNS = ['run', 'start_s', 'end_s', 'kind',
                'period_s', 'height_m', 'steepness', 'amplitude_m',              # regular
                'peak_period_s', 'hs_m', 'gamma', 'seed', 'f_min_hz', 'f_max_hz',  # irregular
                'paddle_std_m', 'paddle_peak_m',
                'gain', 'status']


def run_signal(wm: Wavemaker, spec: RunSpec, dt: float):
    """(x, row, pause_s) for one spec of the test matrix, as motion_files.assemble takes it."""
    if isinstance(spec, RegularSpec):
        x, row = regular_signal(wm, spec, dt)
    elif isinstance(spec, IrregularSpec):
        x, row = irregular_signal(wm, spec, dt)
    else:
        raise TypeError(f'unknown spec {type(spec).__name__}')
    return x, row, spec.pause_s


def assemble(wm: Wavemaker, specs: list[RunSpec], dt: float, max_velocity_mps: float):
    """Frames (n x 2: position, trigger) and run-table rows, in file order."""
    runs = [run_signal(wm, spec, dt) for spec in specs]
    return motion_files.assemble(
        runs, dt, lambda x: check_run(wm, x, dt, max_velocity_mps))


def write_run_table(path: str, rows: list[dict]):
    """Run table for play_dat: the columns of both kinds; a column a row lacks is left empty."""
    motion_files.write_run_table(path, rows, WAVE_COLUMNS)


def write_record(path: str, argv: list[str], args, wm: Wavemaker, yaml_depth: float,
                 config: str, max_velocity_mps: float, summary: str):
    """<output>.txt: what the file was made from, to read probe data and to make it again."""
    if args.water_depth is None:
        depth = f'{wm.water_depth} m (the yaml)'
    else:
        depth = f'{wm.water_depth} m (--water-depth; the yaml has {yaml_depth} m)'
    speed = f'{max_velocity_mps:.4f} m/s'
    if args.max_velocity_mps:
        speed += ' (--max-velocity-mps)'
    motion_files.write_record(
        path, 'generate_waves', argv, config,
        [f'wavemaker    {wm.name} ({wm.type})', f'water depth  {depth}'],
        args.dt, speed, summary, args.test_matrix)


# ---------------------------------------------------------------- command line

def main(argv=None):
    parser = argparse.ArgumentParser(description='Generate wave motion files for play_dat.')
    parser.add_argument('wavemaker', help='name in wavemakers.yaml')
    parser.add_argument('test_matrix', help='CSV, one row per run (read_test_matrix)')
    parser.add_argument('-o', '--output', required=True,
                        help='base name for .dat, .csv, .txt and .png')
    parser.add_argument('--config', help='another wavemakers.yaml; default the installed one')
    parser.add_argument('--dt', type=float, default=0.02, help='sample interval [s], whole ms')
    parser.add_argument('--water-depth', type=float, help='override the yaml for this file')
    parser.add_argument('--max-velocity-mps', type=float,
                        help="override the wavemaker's speed limit from the yaml")
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

    wm = load_wavemaker(args.wavemaker, config)
    yaml_depth = wm.water_depth
    if args.water_depth is not None:
        if wm.gain_periods and args.water_depth != wm.water_depth:
            print(f'warning: the gain table is for {wm.water_depth} m, not {args.water_depth} m')
        wm.water_depth = args.water_depth
    max_velocity = args.max_velocity_mps or wm.max_velocity_mps

    frames, rows = assemble(wm, read_test_matrix(args.test_matrix), args.dt, max_velocity)
    dat_path = args.output + '.dat'
    write_motion(dat_path, frames, args.dt)
    write_run_table(args.output + '.csv', rows)
    played = verify_with_play_dat(dat_path, wm, max_velocity)
    summary = (f'{played} runs, {len(rows) - played} skipped, '
               f'{len(frames) * args.dt / 60:.1f} min')
    write_record(args.output + '.txt', argv, args, wm, yaml_depth, config, max_velocity, summary)
    if not args.no_plot:
        motion_files.plot_motion(args.output + '.png', wm, frames, rows, args.dt, max_velocity,
                                 args.show)
    print(f'{dat_path}: {summary}')
    command = f'ros2 run rig_controller play_dat {dat_path} {wm.name} --dry-run'
    if args.max_velocity_mps:  # check against the same speed the file was made for
        command += f' --max-velocity-mps {max_velocity:.4f}'
    if args.config is not None:
        command += f' --config {args.config}'
    print(command)

    return 0


if __name__ == '__main__':
    sys.exit(main())
