#!/usr/bin/env python3
"""
SKETCH - structure to implement, not a working script.

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
                    period_s=float(r['period_s']),
                    height_m=float(height) if height else None,
                    steepness=parse_steepness(steepness, row) if steepness else None,
                    n_periods=number(r, 'n_periods', row, int)))

            elif kind == 'irregular':
                specs.append(RunSpec(
                    **common,
                    hs_m=number(r, 'hs_m', row),
                    tp_s=number(r, 'tp_s', row),
                    gamma=float((r.get('gamma') or '').strip or 3.3),
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

def irregular_signal(wm: Wavemaker, spec: RunSpec, dt: float):
    """Later: spectrum -> per-frequency paddle amplitudes through the same transfer function and
    gain -> sum with random phases (fixed seed) -> ramps. Same contract as a regular run."""
    raise NotImplementedError

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
    raise NotImplementedError


# ---------------------------------------------------------------- assemble and write

def assemble(wm: Wavemaker, specs: list[RunSpec], dt: float, max_velocity_mps: float):
    """
    Frames (n x 2: position, trigger) and run-table rows, in file order.
      - Lead-in rest of exact zeros, >= 1 s.
      - Per spec: signal -> check_run -> append the run and its pause (exact zeros, >= 1 s),
        or add a row with status 'skipped' and append nothing.
      - start_s = time of the run's first sample; end_s = start_s + len(x) * dt.
        play_dat accepts +-0.05 s, so a sample or two off passes; more fails.
      - Trigger: decide what external equipment needs (play_dat does not play it yet).
    """
    raise NotImplementedError


def write_motion(path: str, frames, dt: float):
    """
    np.array([round(dt * 1000), n_channels], dtype='<i4'), then frames.astype('<f4'), row by
    row. Reject a dt that is not whole milliseconds (e.g. 0.0125): the header would be wrong.
    """
    raise NotImplementedError


def write_run_table(path: str, rows: list[dict]):
    """CSV: run, start_s, end_s, kind, period_s, height_m, gain, amplitude_m, status."""
    raise NotImplementedError


def verify_with_play_dat(dat_path: str, wm: Wavemaker, max_velocity_mps: float):
    """
    Read the file back with play_dat's own functions: read_motion, select_channel('position'),
    split_runs, read_run_table + apply_run_table, and check_run with this wavemaker's limits.
    If these pass, play_dat will accept the file.
    """
    raise NotImplementedError


# ---------------------------------------------------------------- command line

def main(argv=None):
    """
    Arguments:
      wavemaker            name in wavemakers.yaml (ladertanken, lilletanken, mclab)
      test_matrix          file read by read_test_matrix
      -o / --output        base name for .dat and .csv
      --config             default default_config_path()
      --dt                 sample interval, whole milliseconds, e.g. 0.02 s
      --water-depth        override for this file; warn that the gain table is only valid at
                           the depth it was measured at
      --max-velocity-mps   required for linear drives
    Print a summary and the play_dat command with this wavemaker's limits
    (--min-m, --max-m, --max-velocity-mps).
    """
    raise NotImplementedError


# ---------------------------------------------------------------- tests (rig_controller/test/)
# test_generate_waves.py, next to test_play_dat.py:
#   - load_wavemaker / transfer_gain: the ladertanken values above
#   - paddle_amplitude: the ladertanken table above, relative tolerance 1e-3
#   - regular_signal: starts and ends at 0, expected sample count
#   - check_run: a too-large run gets a reason; assemble leaves no gap where it was skipped
#   - write -> verify_with_play_dat round trip: boundaries match within 0.05 s


if __name__ == '__main__':
    main()
