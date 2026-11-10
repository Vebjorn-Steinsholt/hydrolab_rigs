"""
Motion files for play_dat: what generate_waves and generate_oscillations have in common.

Writing the file (.dat), its run table (.csv), the record of how it was made (.txt) and the plot
(.png); assembling runs with rest between them; checking each run against the rig's limits; the
spectrum and synthesis of irregular runs; reading the file back with play_dat's own functions.
Nothing here knows about waves or a particular rig: the generators make each run's positions
and pass in the rig's limits (generate_waves.Wavemaker or rig_limits.RigLimits, which share the
fields used here).

Contract with play_dat.py
  .dat (read_motion)
    - Little-endian, as play_dat reads it ('<i4', '<f4'): int32 time step [ms] > 0, int32
      channel count > 0, then float32 frames row by row; the size must fit whole frames.
    - All values finite. Channel 0 'position', channel 1 'trigger' (CHANNELS); the position
      channel must not be all zeros.
    - Position: metres relative to upright, as the controller takes it (for a wavemaker at the
      attachment point, transfer gain applied). play_dat sends it unchanged and the controller
      applies no gain to pregenerated goals.
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
  - A run starting at upright needs no blend from the actuator's position.
"""
from collections.abc import Callable
import csv
from datetime import datetime
import math
import os
import shlex
import subprocess
from typing import TypeVar

import numpy as np

import play_dat


G = 9.81  # m/s^2, the controller's value; jonswap's scale (it cancels in the normalisation)

# ---------------------------------------------------------------- test matrix

T = TypeVar('T')


def number(r: dict, key: str, row: int, kind: Callable[[str], T] = float) -> T:
    """Read a required number from a test-matrix row; errors name the row and column."""
    text = (r.get(key) or '').strip()
    if not text:
        raise ValueError(f'row {row}: {key} is required')
    try:
        return kind(text)
    except ValueError:
        raise ValueError(f"row {row}: {key} '{text}' is not a number") from None


# ---------------------------------------------------------------- signals

def apply_ramps(x, n_ramp_samples: int, shape: str = 'linear'):
    """
    Ramp 0 -> 1 at the start and 1 -> 0 at the end; the run ends exactly at upright.

    shape 'linear' (the wavemaker's) or 'cosine' (the oscillator's: its slope is 0 at both ends,
    so the acceleration does not step where the ramp meets the steady part).
    """
    if n_ramp_samples < 2 or 2 * n_ramp_samples > len(x):
        raise ValueError(f'{n_ramp_samples} ramp samples do not fit a run of {len(x)}')
    ramp = np.linspace(0.0, 1.0, n_ramp_samples)
    if shape == 'cosine':
        ramp = (1.0 - np.cos(np.pi * ramp)) / 2.0
    elif shape != 'linear':
        raise ValueError(f"ramp shape '{shape}' must be linear or cosine")
    x = x.copy()  # avoid modifying the caller's array
    x[:n_ramp_samples] *= ramp
    x[-n_ramp_samples:] *= ramp[::-1]
    x[-1] = 0.0
    return x


# ---------------------------------------------------------------- irregular runs
# JONSWAP spectrum, random phases, synthesis by inverse FFT.
SIGMA_1 = 0.07       # JONSWAP peak width below fp
SIGMA_2 = 0.09       # and above


def frequency_grid(duration_s: float, dt: float, f_min: float, f_max: float):
    """
    Return the synthesis frequencies f_n = n df, df = 1 / duration.

    Only those within f_min..f_max and below Nyquist. Returns (n, f, df, n_samples); n are the
    FFT bin numbers.
    """
    n_samples = round(duration_s / dt)
    if abs(n_samples * dt - duration_s) > 1e-9:
        raise ValueError(f'duration {duration_s} s is not a whole number of {dt} s samples')
    df = 1.0/(n_samples * dt)
    n_all = np.arange(1, n_samples//2+1)
    f_all = n_all * df
    mask = (f_all >= f_min) & (f_all <= f_max) & (f_all < 0.5/dt)
    if not mask.any():
        raise ValueError(f'no frequencies between {f_min:.3f} and {f_max:.3f} Hz')
    return n_all[mask], f_all[mask], df, n_samples


def jonswap(f, df: float, hs_m: float, tp_s: float, gamma: float,
            sigma1: float = SIGMA_1, sigma2: float = SIGMA_2):
    """
    Return the JONSWAP spectrum S(f) [m^2/Hz] on the grid.

    alpha is chosen with the same sum the synthesis uses, so the generated 4 sqrt(m0) is exactly
    hs_m: the significant wave height for the water, the significant stroke for an oscillator.
    """
    fm = 1.0 / tp_s
    sigma = np.where(f <= fm, sigma1, sigma2)
    b = np.exp(-((f / fm - 1.0) ** 2) / (2.0 * sigma ** 2))
    shape = (G ** 2 / (16.0 * math.pi ** 4) * f ** -5.0
             * np.exp(-1.25 * (f / fm) ** -4.0) * gamma ** b)
    m0 = shape.sum() * df
    return shape * (hs_m ** 2 / 16.0) / m0


def wave_amplitudes(spectrum, df: float):
    """Return the amplitudes of a spectrum, a_n = sqrt(2 S(f_n) df); 4 sqrt(sum(a_n^2)/2) = Hs."""
    return np.sqrt(2.0 * spectrum * df)


def random_phases(seed: int, count: int):
    """Uniform in [0, 2 pi); the same seed gives the same run."""
    return np.random.default_rng(seed).uniform(0.0, 2.0 * math.pi, count)


def synthesize(n, p, phases, n_samples: int):
    """
    Return x_k = sum_n p_n cos(2 pi n k / n_samples + phi_n), by inverse FFT.

    irfft divides by n_samples and doubles the positive frequencies, hence the factor
    n_samples / 2.
    """
    spectrum = np.zeros(n_samples // 2 + 1, dtype=complex)
    spectrum[n] = p * np.exp(1j * phases) * (n_samples / 2.0)
    return np.fft.irfft(spectrum, n_samples)


def synthesize_direct(f, p, phases, n_samples: int, dt: float):
    """Compute the same sum term by term: slow, for testing synthesize() on short runs."""
    t = np.arange(n_samples) * dt
    x = np.zeros(n_samples)
    for fn, pn, phi in zip(f, p, phases):
        x += pn * np.cos(2.0 * math.pi * fn * t + phi)
    return x


# ---------------------------------------------------------------- checks

def check_run(limits, x, dt: float, max_velocity_mps: float) -> list[str]:
    """
    Return the reasons the controller would reject the run or play_dat refuse it; empty if OK.

    limits: anything with upright_m, min_m, max_m and max_acceleration_mps2 (None when off), such
    as generate_waves.Wavemaker or rig_limits.RigLimits.

    Compute each as play_dat and the controller do, so the numbers match:
      - travel: upright_m + x within min_m..max_m
      - speed: max|np.diff(x)| / dt <= max_velocity_mps
      - acceleration: max|x[i+1] - 2 x[i] + x[i-1]| / dt^2 <= limits.max_acceleration_mps2 if
        set; report it always, so a limit set later is no surprise
      - start and end within 2 mm of upright
    Hard limits are the yaml values; a margin on top is optional.
    """
    issues = []
    position = limits.upright_m + x
    if position.min() < limits.min_m or position.max() > limits.max_m:
        issues.append(f'travel {position.min():+.4f}..{position.max():+.4f} m outside '
                      f'{limits.min_m:+.4f}..{limits.max_m:+.4f} m')
    speed = np.abs(np.diff(x)).max() / dt
    if speed > max_velocity_mps:
        issues.append(f'peak speed {speed:.3f} > {max_velocity_mps:.3f} m/s')
    acceleration = np.abs(np.diff(x, 2)).max() / dt ** 2
    if limits.max_acceleration_mps2 is not None and acceleration > limits.max_acceleration_mps2:
        issues.append(f'peak acceleration {acceleration:.2f} > '
                      f'{limits.max_acceleration_mps2:.2f} m/s^2')
    for where, value in (('starts', x[0]), ('ends', x[-1])):
        if abs(value) > play_dat.UPRIGHT_TOLERANCE_M:
            issues.append(f'{where} {value:+.4f} m from upright')
    return issues


# ---------------------------------------------------------------- assemble and write

def assemble(runs, dt: float, check):
    """
    Frames (n x 2: position, trigger) and run-table rows, in file order.

    runs: (x, row, pause_s) per run, x relative to upright, row the generator's own fields.
    check(x): the reasons the run would be refused (check_run); a run with any is skipped.
    """
    lead_in = np.zeros((round(max(play_dat.MIN_GAP_S, 2.0) / dt), 2))
    chunks = [lead_in]
    sample = len(lead_in)
    rows = []
    for index, (x, row, pause_s) in enumerate(runs):
        row['run'] = index
        issues = check(x)
        if issues:
            row.update(start_s='', end_s='', status='skipped')
            print(f"run {index} skipped: {'; '.join(issues)}")
        else:
            row.update(start_s=round(sample * dt, 3), end_s=round((sample + len(x)) * dt, 3),
                       status='')
            pause = np.zeros((round(pause_s / dt), 2))
            chunks += [np.column_stack((x, np.ones_like(x))), pause]
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


def write_run_table(path: str, rows: list[dict], columns: list[str]):
    """Run table for play_dat in the generator's columns; a column a row lacks is left empty."""
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, restval='')
        writer.writeheader()
        writer.writerows(rows)


def source_commit() -> str:
    """Return the repository's commit, '-dirty' for local changes; 'unknown' without git."""
    here = os.path.dirname(os.path.realpath(__file__))
    try:  # an installed copy: CMakeLists.txt writes where the repo is.
        with open(os.path.join(here, 'source_dir.txt')) as f:
            here = f.read().strip()
    except OSError:
        pass
    try:
        result = subprocess.run(['git', '-C', here, 'describe', '--always', '--dirty',
                                 '--exclude=*'],
                                capture_output=True, text=True, timeout=5, check=True)
    except (OSError, subprocess.SubprocessError):
        return 'unknown'
    return result.stdout.strip()


def write_record(path: str, program: str, argv: list[str], config: str, rig_lines: list[str],
                 dt: float, speed: str, summary: str, matrix_path: str):
    """
    <output>.txt: what the file was made from, to read the measurements and to make it again.

    rig_lines: the generator's own lines (rig, water depth, ...), after config.
    """
    with open(matrix_path) as f:
        matrix = f.read().rstrip('\n')
    lines = [
        f'generated    {datetime.now().astimezone().isoformat(timespec="seconds")}',
        f'command      {program} {shlex.join(argv)}',
        f'folder       {os.getcwd()}',
        f'commit       {source_commit()}',
        f'config       {os.path.realpath(config)}',
        *rig_lines,
        f'dt           {dt} s',
        f'max speed    {speed}',
        f'result       {summary}',
        '',
        f'test matrix {matrix_path}:',
        matrix,
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def verify_with_play_dat(dat_path: str, limits, max_velocity_mps: float):
    """Read the file back with play_dat's own functions; raises if play_dat would refuse it."""
    motion = play_dat.read_motion(dat_path)
    values = play_dat.select_channel(motion, 'position')
    runs = play_dat.split_runs(values, motion.sample_interval_s, limits.upright_m)
    table = play_dat.read_run_table(dat_path)
    if table is None:
        raise ValueError(f'{dat_path}: no run table next to it')
    play_dat.apply_run_table(runs, table, motion.sample_interval_s)
    for run in runs:
        issues = play_dat.check_run(run, motion.sample_interval_s, limits.upright_m,
                                    limits.min_m, limits.max_m, max_velocity_mps)
        if issues:
            raise ValueError(f"run {run.index}: {'; '.join(issues)}")
    return len(runs)


def plot_motion(png_path: str, limits, frames, rows: list[dict], dt: float,
                max_velocity_mps: float, show: bool = False):
    """Position and speed over the whole file, runs shaded, limits dashed (limits.name titles)."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    x = frames[:, 0]
    t = np.arange(len(x)) * dt
    speed = np.abs(np.diff(x, prepend=x[0])) / dt
    fig, (ax_x, ax_v) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    ax_x.plot(t, x, lw=0.6)
    for limit in (limits.min_m - limits.upright_m, limits.max_m - limits.upright_m):
        ax_x.axhline(limit, ls='--', color='tab:red', lw=0.8)
    ax_v.plot(t, speed, lw=0.6)
    ax_v.axhline(max_velocity_mps, ls='--', color='tab:red', lw=0.8)

    for row in rows:
        if row['status'] == 'skipped':
            continue
        for ax in (ax_x, ax_v):
            ax.axvspan(row['start_s'], row['end_s'], color='tab:blue', alpha=0.08)
        ax_x.text((row['start_s'] + row['end_s']) / 2, limits.max_m - limits.upright_m,
                  str(row['run']), ha='center', va='bottom', fontsize=8)

    skipped = [str(r['run']) for r in rows if r['status'] == 'skipped']
    title = f'{limits.name}: {len(rows) - len(skipped)} runs'
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
