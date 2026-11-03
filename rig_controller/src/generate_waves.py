#!/usr/bin/env python3
"""
Generate wave motion files (.dat + run table .csv) for one wavemaker, using its geometry and
limits from wavemakers.yaml. The goal is compatibility: every file plays with play_dat.py and
every run is accepted by the controller on that wavemaker.

The wave physics is the controller's (wave_math.cpp, WaveTrajectory::configure), so a run in a
file and the same H and T sent as a regular goal give the same stroke. The old MATLAB WaveGen
scripts are background only, not a design to follow.

Contract with play_dat.py
  .dat (read_motion)
    - Little-endian, as play_dat reads it ('<i4', '<f4'): int32 time step [ms] > 0, int32
      channel count > 0, then float32 frames row by row; the size must fit whole frames.
    - All values finite. Channel 0 'position', channel 1 'trigger' (CHANNELS); the position
      channel must not be all zeros.
    - Position: metres at the attachment point, relative to upright, transfer gain applied.
      play_dat sends it unchanged and the controller applies no gain to pregenerated goals.
  Runs (rest_mask, split_runs, check_run)
    - Rest is |x| <= gap_threshold_m (default 0) and speed <= 1e-6 m/s: rest must be exactly 0.0.
    - Runs are separated by >= MIN_GAP_S (1 s) of rest; shorter rest stays inside the run.
    - Each run starts and ends within UPRIGHT_TOLERANCE_M (2 mm) of upright.
  .csv run table (read_run_table, apply_run_table), next to the .dat with the same base name
    - Required columns run, start_s, end_s; kind and period_s optional (display only).
    - Rows with status 'skipped' are dropped; the rest must match the detected runs in number
      and order, boundaries within RUN_TABLE_TOLERANCE_S (0.05 s).

Contract with the controller (pregenerated goal, controller_node.cpp ~800-950)
  - sample_interval finite and > 0 (from the header).
  - Every absolute position (upright_m + x) within the limits it derives from the yaml.
  - Peak speed, max|diff(x)|/dt, <= the actuator's max_velocity_mps().
  - Peak acceleration, max|second difference|/dt^2, <= max_goal_acceleration_mps2 if set (> 0)
    and <= the actuator's limit (IndraDrive: none; LinMot: has one).
  - A run starting at upright needs no blend from the paddle's position.
"""
import argparse
import csv
from dataclasses import dataclass
import math
import os
import sys

import numpy as np
import yaml

import play_dat

G = 9.81  # m/s^2, the controller's value

# ---------------------------------------------------------------- wavemaker from the yaml

@dataclass
class Wavemaker:
    """What generation needs from /wavemakers/<name>/controller: ros__parameters."""
    name: str
    type: str                # wavemaker_type: 'flap' | 'piston'
    water_depth: float
    hinge_height: float      # flap only
    attachment_height: float  # flap_attachment_height, flap only
    gain: float
    gain_periods: list       # wavemaker_transfer_gain_periods (optional table)
    gain_values: list
    min_m: float             # absolute position limits, as the controller derives them
    max_m: float
    upright_m: float         # upright_position_m
    max_velocity_mps: float | None
    max_acceleration_mps2: float | None  # max_goal_acceleration_mps2; None when 0 (off)


def default_config_path() -> str:
    """
    wavemakers.yaml from the installed wavemaker_bringup share directory
    (ament_index_python), falling back to the copy in the repository.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('wavemaker_bringup')
    except Exception:  # not sourced: the copy in the repository
        share = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', '..', 'wavemaker_bringup')
    return os.path.join(share, 'config', 'wavemakers.yaml')


def required(parameter: dict, key: str, name: str):
    """
    A key whose controller default would give the wrong waves for this wavemaker (e.g.
    wavemaker_type 'piston', water_depth 0.6, limits of +-500 deg): refuse instead of guessing.
    """
    if key not in parameter:
        raise ValueError(f"wavemaker '{name}': set {key} in wavemakers.yaml")
    return parameter[key]


def load_wavemaker(name: str, config_path: str) -> Wavemaker:
    """
    Read the yaml (yaml.safe_load) and pick '/wavemakers/<name>/controller' -> 'ros__parameters';
    list the available names if it is missing.

    required(): no safe default; the type, depth, geometry and limits must be stated for each
    wavemaker, not taken from the controller's generic defaults. params.get(): the controller's
    default, so a key left out of the yaml means the same here as on the rig. Keep these defaults
    in step with controller_node.cpp and indradrive_actuator.cpp (declare_parameter calls).

    Limits, as controller_node.cpp sets goal_position_minimum_/maximum_ (~210-245):
      - drive_motion 'angular': through DriveScale (drive_scale.hpp)
          m = upright_m + (deg - actuator_upright_angle_deg) * actuator_lead_m_per_degree
          max_velocity_mps = max_velocity_rpm * 6 * lead (IndraDriveActuator::max_velocity_mps)
      - drive_motion 'linear': travel_minimum_m / travel_maximum_m. Speed is not defined yet
        (TODO item 19): None, and require it on the command line.

    Test, ladertanken: min_m -0.14575, max_m +0.14575, max_velocity_mps 0.7632.
    """

    with open(config_path) as f:
        config = yaml.safe_load(f)
    key = f'/wavemakers/{name}/controller'
    if key not in config:
        names = [k.split('/')[2] for k in config if k.startswith('/wavemakers/')]
        raise ValueError(f"no wavemaker '{name}' in {config_path}; have {', '.join(names)}")
    params = config[key]['ros__parameters']

    wavemaker_type = required(params, 'wavemaker_type', name)
    upright_m = params.get('upright_position_m', 0.0)

    if params.get('drive_motion', 'linear') == 'angular':
        lead = required(params, 'actuator_lead_m_per_degree', name)
        upright_deg = params.get('actuator_upright_angle_deg', 0.0)
        min_m = upright_m + (required(params, 'min_position_deg', name) - upright_deg) * lead
        max_m = upright_m + (required(params, 'max_position_deg', name) - upright_deg) * lead
        max_velocity_mps = required(params, 'max_velocity_rpm', name) * 6.0 * lead
    else:
        min_m = required(params, 'travel_minimum_m', name)
        max_m = required(params, 'travel_maximum_m', name)
        max_velocity_mps = None  # rpm -> m/s not defined for linear drives (TODO item 19)

    if wavemaker_type == 'flap':
        attachment_height = required(params, 'flap_attachment_height', name)
    else:
        attachment_height = 0.0  # unused for a piston
    max_acceleration = params.get('max_goal_acceleration_mps2', 0.0)

    return Wavemaker(
        name=name,
        type=wavemaker_type,
        water_depth=required(params, 'water_depth', name),
        hinge_height=params.get('hinge_height', 0.0),
        attachment_height=attachment_height,
        gain=params.get('wavemaker_transfer_gain', 1.0),
        gain_periods=list(params.get('wavemaker_transfer_gain_periods', [])),
        gain_values=list(params.get('wavemaker_transfer_gain_values', [])),
        min_m=min_m, max_m=max_m, upright_m=upright_m,
        max_velocity_mps=max_velocity_mps,
        max_acceleration_mps2=max_acceleration if max_acceleration > 0.0 else None)


def transfer_gain(wm: Wavemaker, period_s: float) -> float:
    """
    The table if set, else wm.gain. As interpolate_gain() in wave_math.cpp: linear between
    points, nearest end value outside (np.interp does both).

    Test, ladertanken: 1.05 s -> 0.755; 0.5 s -> 0.64; 2.5 s -> 0.76.
    """

    if wm.gain_periods:
        return float(np.interp(period_s, wm.gain_periods, wm.gain_values))
    return wm.gain


# ---------------------------------------------------------------- wave physics (as the controller)

def wavenumber_depth(period_s: float, depth: float) -> float:
    """
    k*h, the controller's Beji 2013 approximation (solve_dispersion), g = 9.81:
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
    H/S at the still-water level, compute_stroke() in wave_math.cpp:
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
    Amplitude at the attachment point, as WaveTrajectory::configure, in this order:
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
    kind: str                         # 'regular' | 'irregular'
    n_ramp: int                       # periods (regular) or peak periods (irregular) of ramp
    pause_s: float                    # rest after the run, >= MIN_GAP_S (1 s)
    # regular
    period_s: float | None = None
    height_m: float | None = None     # exactly one of height_m and steepness
    steepness: float | None = None    # x in H/lambda = 1/x, e.g. 60 for 1/60
    n_periods: int | None = None      # including ramps
    # irregular
    hs_m: float | None = None
    tp_s: float | None = None
    gamma: float | None = None        # 3.3 typical; 1.0 = Pierson-Moskowitz
    duration_s: float | None = None   # whole run, ramps included
    seed: int | None = None


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


def number(r: dict, key: str, row: int, kind=float):
    """A required number from a test-matrix row, with a message naming the row and column."""
    text = (r.get(key) or '').strip()
    if not text:
        raise ValueError(f'row {row}: {key} is required')
    try:
        return kind(text)
    except ValueError:
        raise ValueError(f"row {row}: {key} '{text}' is not a number") from None

def read_test_matrix(path: str) -> list[RunSpec]:
    """
    CSV, one row per run, played in file order. Columns per kind (others left empty):
      all:        kind (default regular), n_ramp, pause_s
      regular:    period_s, height_m or steepness (1/x), n_periods
      irregular:  hs_m, tp_s, gamma (default 3.3), duration_s, seed
    """
    specs = []
    with open(path, newline='') as f:
        for row, r in enumerate(csv.DictReader(f)):
            kind = (r.get('kind') or 'regular').strip()
            common = {'kind': kind, 'n_ramp': number(r, 'n_ramp', row, int),
                      'pause_s': number(r, 'pause_s', row)}
            if kind == 'regular':
                height = (r.get('height_m') or '').strip()
                steepness = (r.get('steepness') or '').strip()
                if bool(height) == bool(steepness):
                    raise ValueError(f"row {row}: give height_m or steepness, not both or neither")
                specs.append(RunSpec(
                    **common,
                    period_s=number(r, 'period_s', row),
                    height_m=float(height) if height else None,
                    steepness=parse_steepness(steepness, row) if steepness else None,
                    n_periods=number(r, 'n_periods', row, int)))

            elif kind == 'irregular':
                specs.append(RunSpec(
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
    """lambda = 2 pi h / kh, with the controller's dispersion approximation."""
    return 2.0 * math.pi * wm.water_depth / wavenumber_depth(period_s, wm.water_depth)

def target_height(wm: Wavemaker, spec: RunSpec) -> float:
    """H as given, or lambda / x for a steepness 1/x."""
    if spec.height_m is not None:
        return spec.height_m
    return wavelength(wm, spec.period_s) / spec.steepness

def apply_ramps(x, n_ramp_samples: int):
    """Ramp 0 -> 1 at the start and 1 -> 0 at the end; the run ends exactly at upright."""
    if n_ramp_samples < 2 or 2 * n_ramp_samples > len(x):
        raise ValueError(f'{n_ramp_samples} ramp samples do not fit a run of {len(x)}')
    ramp = np.linspace(0.0, 1.0, n_ramp_samples)
    x=x.copy()  # avoid modifying the caller's array
    x[:n_ramp_samples] *= ramp
    x[-n_ramp_samples:] *= ramp[::-1]
    x[-1] = 0.0
    return x
         
# ---------------------------------------------------------------- signals
# ---------------------------------------------------------------- Irregular waves: JONSWAP spectrum, random phases, ramps
F_MIN_FACTOR = 0.5   # band: F_MIN_FACTOR * fp .. F_MAX_FACTOR * fp (README, Irregular waves)
F_MAX_FACTOR = 3.0
SIGMA_1 = 0.07       # JONSWAP peak width below fp
SIGMA_2 = 0.09       # and above




def frequency_grid(duration_s: float, dt: float, f_min: float, f_max: float):
    """The synthesis frequencies f_n = n df, df = 1 / duration, within f_min..f_max and below
    Nyquist. Returns (n, f, df, n_samples); n are the FFT bin numbers.
    """
    n_samples = round(duration_s / dt)
    if abs(n_samples * dt - duration_s) > 1e-9:
        raise ValueError(f'duration {duration_s} s is not a whole number of {dt} s samples')
    df = 1.0/(n_samples * dt)
    n_all = np.arange(1,n_samples//2+1)
    f_all = n_all * df
    mask = (f_all >= f_min) & (f_all <= f_max) & (f_all < 0.5/dt)
    if not mask.any():
        raise ValueError(f'no frequencies between {f_min:.3f} and {f_max:.3f} Hz')
    return n_all[mask], f_all[mask], df, n_samples

def jonswap(f, df: float, hs_m: float, tp_s: float, gamma: float,
            sigma1: float = SIGMA_1, sigma2: float = SIGMA_2):
    """
    S(f) [m^2/Hz] on the grid. alpha is chosen with the same sum the synthesis uses, so the
    generated Hs is exactly hs_m.
    """
    fm = 1.0 / tp_s
    sigma = np.where(f <= fm, sigma1, sigma2)
    b = np.exp(-((f / fm - 1.0) ** 2) / (2.0 * sigma ** 2))
    shape = (G ** 2 / (16.0 * math.pi ** 4) * f ** -5.0
             * np.exp(-1.25 * (f / fm) ** -4.0) * gamma ** b)
    m0 = shape.sum() * df
    return shape * (hs_m ** 2 / 16.0) / m0

def wave_amplitudes(spectrum, df: float):
    """a_n = sqrt(2 S(f_n) df); then 4 sqrt(sum(a_n^2) / 2) = Hs."""
    return np.sqrt(2.0 * spectrum * df)

def paddle_amplitudes(wm: Wavemaker, f, a):
    """Each component as a regular wave of height 2 a_n: same transfer function, gain and scaling."""
    return np.array([paddle_amplitude(wm, 1.0 / fn, 2.0 * an) for fn, an in zip(f, a)])

def random_phases(seed: int, count: int):
    """Uniform in [0, 2 pi); the same seed gives the same run."""
    return np.random.default_rng(seed).uniform(0.0, 2.0 * math.pi, count)

def synthesize(n, p, phases, n_samples: int):
    """
    x_k = sum_n p_n cos(2 pi n k / n_samples + phi_n), by inverse FFT. irfft divides by
    n_samples and doubles the positive frequencies, hence the factor n_samples / 2.
    """
    spectrum = np.zeros(n_samples // 2 + 1, dtype=complex)
    spectrum[n] = p * np.exp(1j * phases) * (n_samples / 2.0)
    return np.fft.irfft(spectrum, n_samples)

def synthesize_direct(f, p, phases, n_samples: int, dt: float):
    """The same sum, term by term: slow, for testing synthesize() on short runs."""
    t = np.arange(n_samples) * dt
    x = np.zeros(n_samples)
    for fn, pn, phi in zip(f, p, phases):
        x += pn * np.cos(2.0 * math.pi * fn * t + phi)
    return x


def irregular_signal(wm: Wavemaker, spec: RunSpec, dt: float):
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


# ---------------------------------------------------------------- regular waves: sine wave, ramps, paddle amplitude from the controller's physics

def regular_signal(wm: Wavemaker, spec: RunSpec, dt: float):
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





def check_run(wm: Wavemaker, x, dt: float, max_velocity_mps: float) -> list[str]:
    """
    Reasons the controller would reject the run or play_dat would refuse it; empty if OK.
    Compute each as play_dat and the controller do, so the numbers match:
      - travel: upright_m + x within min_m..max_m
      - speed: max|np.diff(x)| / dt <= max_velocity_mps
      - acceleration: max|x[i+1] - 2 x[i] + x[i-1]| / dt^2 <= wm.max_acceleration_mps2 if set;
        report it always, so a limit set later is no surprise
      - start and end within 2 mm of upright
    Hard limits are the yaml values; a margin on top is optional.
    """
    issues = []
    position = wm.upright_m + x
    if position.min() < wm.min_m or position.max() > wm.max_m:
        issues.append(f'travel {position.min():+.4f}..{position.max():+.4f} m outside '
                      f'{wm.min_m:+.4f}..{wm.max_m:+.4f} m')
    speed = np.abs(np.diff(x)).max() / dt
    if speed > max_velocity_mps:
        issues.append(f'peak speed {speed:.3f} > {max_velocity_mps:.3f} m/s')
    acceleration = np.abs(np.diff(x, 2)).max() / dt ** 2
    if wm.max_acceleration_mps2 is not None and acceleration > wm.max_acceleration_mps2:
        issues.append(f'peak acceleration {acceleration:.2f} > '
                      f'{wm.max_acceleration_mps2:.2f} m/s^2')
    for where, value in (('starts', x[0]), ('ends', x[-1])):
        if abs(value) > play_dat.UPRIGHT_TOLERANCE_M:
            issues.append(f'{where} {value:+.4f} m from upright')
    return issues


# ---------------------------------------------------------------- assemble and write

def assemble(wm: Wavemaker, specs: list[RunSpec], dt: float, max_velocity_mps: float):
    """Frames (n x 2: position, trigger) and run-table rows, in file order."""
    lead_in = np.zeros((round(max(play_dat.MIN_GAP_S, 2.0) / dt),2))
    chunks  = [lead_in]
    sample = len(lead_in)
    rows = []
    for index, spec in enumerate(specs):
        if spec.kind == 'regular':
            x, row = regular_signal(wm, spec, dt)
        else:
            x, row = irregular_signal(wm, spec, dt)
        row['run'] = index
        issues = check_run(wm, x, dt, max_velocity_mps)
        if issues:
            row.update(start_s='', end_s='', status='skipped')
            print(f"run {index} skipped: {'; '.join(issues)}")
        else:
            row.update(start_s=round(sample * dt, 3), end_s=round((sample + len(x)) * dt, 3),
                       status='')
            pause = np.zeros((round(spec.pause_s / dt), 2))
            chunks += [np.column_stack((x, np.ones_like(x))),pause]
            sample += len(x) + len(pause)
        rows.append(row)
    return np.vstack(chunks), rows


def write_motion(path: str, frames, dt: float):
    """play_dat's layout, little-endian; dt must be whole milliseconds."""
    interval_ms = round(dt * 1000.0)
    if abs(interval_ms - dt*1000.0) > 1e-9:
        raise ValueError(f'dt {dt} s is not a whole number of milliseconds')
    with open(path, 'wb') as f:
        np.array([interval_ms, frames.shape[1]], dtype='<i4').tofile(f)
        frames.astype('<f4').tofile(f)


def write_run_table(path: str, rows: list[dict]):
    """Run table for play_dat: the columns of both kinds; a column a row lacks is left empty."""
    columns = ['run', 'start_s', 'end_s', 'kind',
               'period_s', 'height_m', 'steepness', 'amplitude_m',              # regular
               'peak_period_s', 'hs_m', 'gamma', 'seed', 'f_min_hz', 'f_max_hz',  # irregular
               'paddle_std_m', 'paddle_peak_m',
               'gain', 'status']
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, restval='')
        writer.writeheader()
        writer.writerows(rows)


def verify_with_play_dat(dat_path: str, wm: Wavemaker, max_velocity_mps: float):
    """Read the file back with play_dat's own functions; raises if play_dat would refuse it."""
    motion = play_dat.read_motion(dat_path)
    values = play_dat.select_channel(motion, 'position')
    runs = play_dat.split_runs(values, motion.sample_interval_s, wm.upright_m)
    play_dat.apply_run_table(runs, play_dat.read_run_table(dat_path), motion.sample_interval_s)
    for run in runs:
        issues = play_dat.check_run(run, motion.sample_interval_s, wm.upright_m,
                                    wm.min_m, wm.max_m, max_velocity_mps)
        if issues:
            raise ValueError(f"run {run.index}: {'; '.join(issues)}")
    return len(runs)

def plot_motion(png_path: str, wm: Wavemaker, frames, rows: list[dict], dt: float,
                 max_velocity_mps: float, show: bool = False):
    """Paddle position and speed over the whole file, runs shaded, limits dashed."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    x = frames[:,0]
    t = np.arange(len(x)) * dt
    speed = np.abs(np.diff(x, prepend=x[0])) / dt
    fig, (ax_x, ax_v) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    ax_x.plot(t, x, lw=0.6)
    for limit in (wm.min_m - wm.upright_m, wm.max_m - wm.upright_m):
        ax_x.axhline(limit, ls='--', color='tab:red', lw=0.8)
    ax_v.plot(t, speed, lw=0.6)
    ax_v.axhline(max_velocity_mps, ls='--', color='tab:red', lw=0.8)

    for row in rows:
        if row['status'] == 'skipped':
            continue
        for ax in (ax_x, ax_v):
            ax.axvspan(row['start_s'], row['end_s'], color='tab:blue', alpha=0.08)
        ax_x.text((row['start_s'] + row['end_s']) / 2, wm.max_m - wm.upright_m, str(row['run']),
                  ha='center', va='bottom', fontsize=8)

    skipped = [str(r['run']) for r in rows if r['status'] == 'skipped']
    title = f'{wm.name}: {len(rows) - len(skipped)} runs'
    if skipped:
        title += f', skipped {", ".join(skipped)}'
    ax_x.set_title(title)
    ax_x.set_ylabel('position from upright [m]')
    ax_v.set_ylabel('speed [m/s]')
    ax_v.set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------- command line

def main(argv=None):
    parser = argparse.ArgumentParser(description='Generate wave motion files for play_dat.')
    parser.add_argument('wavemaker', help='name in wavemakers.yaml')
    parser.add_argument('test_matrix', help='CSV, one row per run (read_test_matrix)')
    parser.add_argument('-o', '--output', required=True, help='base name for .dat and .csv')
    parser.add_argument('--config', default=default_config_path())
    parser.add_argument('--dt', type=float, default=0.02, help='sample interval [s], whole ms')
    parser.add_argument('--water-depth', type=float, help='override the yaml for this file')
    parser.add_argument('--max-velocity-mps', type=float, help='required for linear drives')
    parser.add_argument('--no-plot', action='store_true', help='do not write <output>.png')
    parser.add_argument('--show', action='store_true', help='also open the plot in a window')
    args = parser.parse_args(argv)
    if not os.path.isfile(args.test_matrix):
        parser.error(f'test matrix {args.test_matrix} not found (relative to {os.getcwd()})')

    wm = load_wavemaker(args.wavemaker, args.config)
    if args.water_depth is not None:
        if wm.gain_periods:
            print(f'warning: the gain table is for {wm.water_depth} m, not {args.water_depth} m')
        wm.water_depth = args.water_depth
    max_velocity = args.max_velocity_mps or wm.max_velocity_mps
    if max_velocity is None:
        parser.error(f'{wm.name} has a linear drive: give --max-velocity-mps')

    frames, rows = assemble(wm, read_test_matrix(args.test_matrix), args.dt, max_velocity)
    dat_path = args.output + '.dat'
    write_motion(dat_path, frames, args.dt)
    write_run_table(args.output + '.csv', rows)
    played = verify_with_play_dat(dat_path, wm, max_velocity)
    if not args.no_plot:
        plot_motion(args.output + '.png', wm, frames, rows, args.dt, max_velocity, args.show)
    print(f'{dat_path}: {played} runs, {len(rows) - played} skipped, '
          f'{len(frames) * args.dt / 60:.1f} min')
    print(f'ros2 run rig_controller play_dat {dat_path} --min-m {wm.min_m:.5f} '
          f'--max-m {wm.max_m:.5f} --max-velocity-mps {max_velocity:.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
