#!/usr/bin/env python3
"""
Transfer function of a wavemaker from one run of a motion file and a catman record.

G_cmd (command -> wave, the gain table's number), and with the flap's stroke G_drive
(command -> flap) and G_water (flap -> wave). Use: LAB_PERSONNEL.md, *Transfer gain from a run*;
the design: git show c6bba03:transfer_function_sketch.md.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import os
import re
import shlex
import sys

# One group, alphabetical (ament_flake8's import order treats the local modules as third party).
import generate_waves as gw
import motion_files
import numpy as np
import play_dat
from scipy import ndimage, signal

COHERENCE_MIN = 0.8      # bins below are dropped
STILL_S = 10.0           # probe checked over this long before the run
STILL_RATIO = 0.1        # warn if the probe's std before the run is above this x the run's
STILL_FLOOR_M = 0.001    # but never below this: ~3x the probe's noise (check on a still record)
MATCH_MIN = 2.0          # offset match: peak / next-best peak below this is refused
COARSE_S = 120.0         # the anchor matches at most this much of the first run
CHUNK_S = 60.0           # the walk: pieces of the run at most this long
SEARCH_TP = 0.4          # each piece is searched within +/- this many periods of the last
DRIFT_MIN_S = 180.0      # fit the clock drift only from at least this much of long runs
DRIFT_MAX_PPM = 1000.0   # refuse a drift above this
DRIFT_RES_MAX_S = 0.01   # warn when the pieces scatter more than this about the fit
F_TOP_FP = 1.5           # irregular: trust up to this x fp; second-order waves sit at 2 fp
ENVELOPE_SMOOTH_TP = 3   # smooth the envelope over this many wave periods


# ---------------------------------------------------------------- inputs

def read_command(dat_path: str, run: int) -> tuple[np.ndarray, float, list[dict], dict]:
    """Return the whole file's x [m], dt, the run table and the run's row in it."""
    motion = play_dat.read_motion(dat_path)
    x_file = play_dat.select_channel(motion, 'position')
    dt = motion.sample_interval_s
    table = play_dat.read_run_table(dat_path)
    if table is None:
        raise ValueError(f'{dat_path}: no run table {os.path.splitext(dat_path)[0]}.csv')
    row = next((r for r in table if int(r['run']) == run), None)
    if row is None:
        raise ValueError(f'run {run} is not in the run table; it has '
                         f'{", ".join(r["run"] for r in table)}')
    if not row.get('ramp_s'):
        raise ValueError(f'{dat_path}: the run table has no ramp_s; generate the file again '
                         'from its test matrix')
    return x_file, dt, table, row


def read_record(path: str) -> tuple[float, dict[str, np.ndarray]]:
    """
    Return the sample interval [s] and {channel name: values} of a record.

    catman's ASCII export (first line HBM_CATMAN_DATAFILE...) or a CSV with time [s] first.
    """
    with open(path, newline='', encoding='cp1252') as f:
        lines = f.read().splitlines()
    if lines and lines[0].startswith('HBM_CATMAN_DATAFILE'):
        return read_catman_ascii(path, lines)
    return read_csv(path, lines)


def read_catman_ascii(path: str, lines: list[str]) -> tuple[float, dict[str, np.ndarray]]:
    """
    Return dt [s] and {name: values} from catman's ASCII export.

    The names follow the blank line after CHANNELS/SEPARATOR/MAXLINES, the data the next blank
    line. dt comes from the header's "dt =" line: catman rounds its time column (to 0.01 s at
    200 Hz), so that column is left out.
    """
    i_channels = next((i for i, line in enumerate(lines) if line.startswith('CHANNELS:')), None)
    if i_channels is None:
        raise ValueError(f'{path}: no CHANNELS line in the catman header')
    sep = '\t'
    for line in lines[i_channels:i_channels + 4]:
        if line.startswith('SEPARATOR:'):
            sep = chr(int(line.split(':')[1]))
    i_names = lines.index('', i_channels) + 1
    names = [n.strip() for n in lines[i_names].split(sep)]
    i_data = lines.index('', i_names) + 1
    dt_line = next((line for line in lines[i_names:i_data] if line.startswith('dt =')), '')
    match = re.match(r'dt =\s*([\d,.]+)\s*(ms|s)\b', dt_line)
    if match is None:
        raise ValueError(f'{path}: no "dt =" line in the catman header')
    dt = float(match.group(1).replace(',', '.')) * (0.001 if match.group(2) == 'ms' else 1.0)
    values = np.array([[float(v.replace(',', '.')) for v in line.split(sep) if v.strip()]
                       for line in lines[i_data:] if line.strip()])
    return dt, {n: values[:, i] for i, n in enumerate(names) if not n.lower().startswith('time')}


def read_csv(path: str, lines: list[str]) -> tuple[float, dict[str, np.ndarray]]:
    """Return dt [s] and {name: values} from a CSV, time [s] first. Refuse a non-uniform time."""
    rows = list(csv.reader(lines))
    names = rows[0]
    values = np.array([[float(v) for v in r] for r in rows[1:] if r])
    t = values[:, 0]
    step = np.diff(t)
    if len(t) < 2 or step.min() <= 0 or np.abs(step - step.mean()).max() > 1e-3 * step.mean():
        raise ValueError(f'{path}: the time column ({names[0]}) is not uniform')
    return float(step.mean()), {name: values[:, i] for i, name in enumerate(names) if i > 0}


def record_channel(channels: dict, name: str, scale: float, decimation: int) -> np.ndarray:
    """
    Return one channel at the file's rate, in m (decimate by decimation, then * scale).

    Refuse a name not in the header, listing those that are.
    """
    if name not in channels:
        raise ValueError(f"no column '{name}' in the record; it has {', '.join(channels)}")
    y = channels[name]
    if decimation > 1:
        y = signal.decimate(y, decimation, ftype='fir', zero_phase=True)
    return y * scale


# ---------------------------------------------------------------- linear theory

def group_velocity(f_hz: float, depth_m: float) -> float:
    """c_g = c/2 (1 + 2kh / sinh 2kh), with k from gw.wavenumber_depth."""
    kh = gw.wavenumber_depth(1 / f_hz, depth_m)
    c = 2 * np.pi * f_hz * depth_m / kh
    return c / 2 * (1 + 2 * kh / np.sinh(2 * kh))


def linear_ratio(f: np.ndarray, ok: np.ndarray, wm: gw.Wavemaker,
                 depth_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(kh, a/x at the waterline, a/x at the attachment); NaN outside ok."""
    kh = np.full(len(f), np.nan)
    g_lin_wl = np.full(len(f), np.nan)
    g_lin = np.full(len(f), np.nan)
    for i in np.flatnonzero(ok):
        kh[i] = gw.wavenumber_depth(1.0 / float(f[i]), depth_m)
        g_lin_wl[i] = gw.stroke_ratio(float(kh[i]), wm)
    g_lin = g_lin_wl * to_waterline(1.0, wm.attachment_height, wm, depth_m)
    return kh, g_lin_wl, g_lin


def to_waterline(y: float | np.ndarray, height_m: float, wm: gw.Wavemaker,
                 depth_m: float) -> float | np.ndarray:
    """Return y * (depth - hinge) / (height - hinge): the flap's geometry."""
    if wm.type != 'flap':
        return y
    return y * (depth_m - wm.hinge_height) / (height_m - wm.hinge_height)


# ---------------------------------------------------------------- where the file is in the record

def local_lag(x_file, y_all, a, n, guess_s, w, dt) -> tuple[float, float, bool]:
    """
    Return (lag_s, r, at_edge) of y_all against x_file[a:a + n], within guess_s +- w samples.

    r is the normalised correlation at the peak; at_edge says the peak is on the window's
    edge (the prediction was off by more than the window).
    """
    xs = x_file[a:a + n]
    lo = max(0, a + round(guess_s / dt) - w)
    hi = min(len(y_all) - n, a + round(guess_s / dt) + w)
    if hi < lo:
        raise ValueError('the run is not inside the record at this offset')
    seg = y_all[lo:hi + n]
    c = signal.correlate(seg - seg.mean(), xs - xs.mean(), mode='valid')
    i = int(np.argmax(c))
    at_edge = i in (0, len(c) - 1)
    d = 0.0
    if not at_edge:                                   # parabola through the peak
        y0, y1, y2 = c[i - 1], c[i], c[i + 1]
        d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    r = c[i] / (n * xs.std() * y_all[lo + i:lo + i + n].std())
    return (lo + i + d - a) * dt, r, at_edge


def first_part(x_file, dt, first, coarse_s=COARSE_S) -> np.ndarray:
    """Return the file from sample 0 to the first run's end, or its start + coarse_s if sooner."""
    end_s = min(float(first['end_s']), float(first['start_s']) + coarse_s)
    return x_file[:round(end_s / dt) + 1]


def anchor_from_stroke(x_file, s_all, dt, first, min_sep) -> tuple[float, float]:
    """(lag_s, match quality) of the first run: the whole-record match, on <= COARSE_S of it."""
    x0 = first_part(x_file, dt, first)
    lags = signal.correlation_lags(len(s_all), len(x0), mode='valid')
    corr = signal.correlate(s_all - s_all.mean(), x0 - x0.mean(), mode='valid')
    i_peak = int(np.argmax(np.abs(corr)))
    if corr[i_peak] < 0:
        raise ValueError("the stroke matches the command upside down: the command's + is "
                         'towards the beach; check the sign of --stroke-scale')
    return lags[i_peak] * dt, match_quality(corr, i_peak, min_sep)


def envelope(y, dt, smooth_s) -> np.ndarray:
    """Envelope of a signal, using the Hilbert transform and a moving average."""
    analytic = np.asarray(signal.hilbert(y - y.mean()))
    return ndimage.uniform_filter1d(np.abs(analytic), max(1, round(smooth_s / dt)))


def anchor_from_envelopes(x_file, eta_travel, dt, first, tp_s, min_sep) -> tuple[float, float]:
    """
    Return (lag_s, match quality) of the first run from the envelopes, refined on their slopes.

    eta_travel has the travel undone, so no travel time is subtracted. The envelopes find the
    run and give the quality; a regular run's plateau makes their peak a few per mille high
    over a second either side, and any tilt moves it by about T/2 (the test's 1.3 s run). The
    slopes (the ramps) then fix it to within a smoothing length, but alone they can't tell one
    regular run's ramps from another's. The command's envelope is of the whole file, then cut,
    so the cut leaves no step in it. The whole first run, not COARSE_S of it: 120 s inside an
    irregular run is too flat to find (2026-11-17: 1.0x), and drift doesn't matter at this
    smoothing.
    """
    smooth_s = ENVELOPE_SMOOTH_TP * tp_s
    env_x = envelope(x_file, dt, smooth_s)[:len(first_part(x_file, dt, first, np.inf))]
    env_eta = envelope(eta_travel, dt, smooth_s)
    lags = signal.correlation_lags(len(env_eta), len(env_x), mode='valid')
    corr = signal.correlate(env_eta - env_eta.mean(), env_x - env_x.mean(), mode='valid')
    i_peak = int(np.argmax(corr))
    quality = match_quality(corr, i_peak, min_sep)
    w = round(smooth_s / dt)
    lo = max(0, i_peak - w)
    d_eta = np.diff(env_eta[lo:i_peak + w + len(env_x)])
    i_fine = lo + int(np.argmax(signal.correlate(d_eta, np.diff(env_x), mode='valid')))
    return lags[i_fine] * dt, quality


def lag_90(eta) -> np.ndarray:
    """
    Return the probe turned back by the water's 90 deg lead (theory; see residual_delay).

    So the walk on the probe (no stroke) finds about the lag the stroke would give, not a
    quarter period off it, and the walk's window holds across runs of different periods.
    """
    n = len(eta)
    return np.fft.irfft(np.fft.rfft(eta - eta.mean()) * -1j, n)


@dataclass
class Timeline:
    """Where each run is in the record (decision 1, git show 88363ba:run_timeline_sketch.md)."""

    a: dict[int, float]        # run number -> lag at its start [s]
    eps: float                 # drift; 0 if not measured
    eps_pieces: int            # pieces in runs of >= 2 pieces (0: not measured)
    eps_runs: int              # those runs
    rms_s: float


def pieces(row, dt) -> list[tuple[int, int]]:
    """(first sample, length) of each piece of the run's inner part (ramps off), <= CHUNK_S."""
    ramp = round(float(row['ramp_s']) / dt)
    a, b = round(float(row['start_s']) / dt) + ramp, round(float(row['end_s']) / dt) - ramp
    n = max(1, int(np.ceil((b - a) * dt / CHUNK_S - 1e-9)))
    edges = np.linspace(a, b, n + 1).round().astype(int)
    return [(int(p), int(q - p)) for p, q in zip(edges, edges[1:])]


def run_period(row) -> float:
    """Return the run's period: the peak period of an irregular run."""
    return float(row['peak_period_s'] or row['period_s'])


def walk(x_file, y_all, dt, rows, start_s) -> list[tuple[int, float, float, float]]:
    """
    Return (run, t_mid - start_k, lag, piece length [s]) of every piece, from start_s on.

    Each piece is searched within SEARCH_TP periods of the last; play_dat starts each run
    as a goal of its own, ~50 ms later than the file says (2026-11-17), and catman's clock
    drifts against the Pi's (+253 ppm), so one offset for the whole file doesn't hold. The
    walk stops where the record does.
    """
    out, lag = [], start_s
    for r in rows:
        w = round(SEARCH_TP * run_period(r) / dt)
        for p, n in pieces(r, dt):
            if p + round(lag / dt) + w + n > len(y_all):
                return out                              # the record stops before this piece
            lag, _, edge = local_lag(x_file, y_all, p, n, lag, w, dt)
            if edge:
                raise ValueError(f"run {r['run']}, piece at {p * dt:.0f} s in the file: the "
                                 f'lag left the +-{SEARCH_TP} T window; give --offset-s')
            out.append((int(r['run']), (p + n / 2) * dt - float(r['start_s']), lag, n * dt))
    return out


def fit_timeline(rows) -> Timeline:
    """lag = a_k + eps * (t - start_k): one intercept per run, one slope (decision 4)."""
    runs = sorted({k for k, _, _, _ in rows})
    count = {k: sum(1 for kk, _, _, _ in rows if kk == k) for k in runs}
    long_s = sum(n_s for k, _, _, n_s in rows if count[k] >= 2)
    measured = long_s >= DRIFT_MIN_S
    A = np.zeros((len(rows), len(runs) + 1))
    y = np.array([lag for _, _, lag, _ in rows])
    for i, (k, t, _, _) in enumerate(rows):
        A[i, runs.index(k)] = 1.0
        A[i, -1] = t
    if not measured:
        A = A[:, :-1]                                   # eps not measured: 0
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    eps = float(sol[-1]) if measured else 0.0
    res = y - A @ sol
    if abs(eps) * 1e6 > DRIFT_MAX_PPM:
        raise ValueError(f'drift {eps * 1e6:+.0f} ppm, above {DRIFT_MAX_PPM:.0f}: '
                         'the wrong record, or a gap in it; give --offset-s and --drift-ppm')
    long_runs = [k for k in runs if count[k] >= 2]
    return Timeline(a={k: float(sol[j]) for j, k in enumerate(runs)}, eps=eps,
                    eps_pieces=sum(count[k] for k in long_runs) if measured else 0,
                    eps_runs=len(long_runs) if measured else 0,
                    rms_s=float(np.sqrt(np.mean(res ** 2))))


def on_file_time(y_all, dt, a_s, eps, start_s, i_from, i_to) -> np.ndarray:
    """
    Return y_all at the record times of file samples i_from..i_to of a run starting at start_s.

    eps = 0 cuts at whole samples, as an offset alone always did; else linear interpolation
    (< 0.05 % amplitude lost at 1.8 s, 50 Hz).
    """
    if eps == 0:
        i0 = round((a_s + start_s) / dt) - round(start_s / dt)
        if i_from + i0 < 0 or i_to + i0 > len(y_all):
            raise ValueError('the run is not inside the record at this offset')
        return y_all[i_from + i0:i_to + i0]
    t = np.arange(i_from, i_to) * dt
    t_rec = a_s + t + eps * (t - start_s)
    y = np.interp(t_rec / dt, np.arange(len(y_all)), y_all, left=np.nan, right=np.nan)
    if np.isnan(y).any():
        raise ValueError('the run is not inside the record at this offset')
    return y


def match_quality(c: np.ndarray, i_peak: int, min_sep: int) -> float:
    """Return the peak over the next-best peak at least min_sep samples away."""
    peaks, _ = signal.find_peaks(c)
    others = peaks[np.abs(peaks - i_peak) > min_sep]
    if len(others) == 0:
        return np.inf
    return abs(c[i_peak]) / np.abs(c[others]).max()


def cut_window(i0, n_run, skip, *series) -> list[np.ndarray | None]:
    """
    Return each record series from i0, n_run samples, then skip at both ends.

    A None series (no stroke) stays None.
    """
    if 2 * skip >= n_run:
        raise ValueError(f'the run ({n_run} samples) is shorter than what is skipped '
                         f'at its ends (2 x {skip})')
    out = []
    for y in series:
        if y is None:
            out.append(None)
            continue
        if i0 < 0 or i0 + n_run > len(y):
            raise ValueError('the run is not inside the record at this offset')
        out.append(y[i0 + skip:i0 + n_run - skip])
    return out


def check_still(eta_all, i0, dt, eta_run_std) -> None:
    """
    Warn if the probe moved before the run.

    That is its std over STILL_S before the run above STILL_RATIO x the run's (eta_run_std), or
    above STILL_FLOOR_M if that is larger.
    """
    n = round(STILL_S / dt)
    before = eta_all[max(0, i0 - n):i0]
    if len(before) < n:
        print(f'warning: only {len(before) * dt:.1f} s of record before the run '
              f'(the stillness check wants {STILL_S:.0f} s)')
    if len(before) > 1 and before.std() > max(STILL_RATIO * eta_run_std, STILL_FLOOR_M):
        print(f'warning: the probe moved {before.std() * 1000:.1f} mm (std) in the '
              f'{STILL_S:.0f} s before the run, {before.std() / eta_run_std:.0%} of the '
              "run's; the water may not have been still")


# ---------------------------------------------------------------- estimate

@dataclass
class Frf:
    f: np.ndarray
    h1: np.ndarray     # S_uy / S_uu
    h2: np.ndarray     # S_yy / S_yu
    coh: np.ndarray
    eps: np.ndarray    # relative error on |G|, Bendat & Piersol
    n_avg: int


def frf(u, y, kw) -> Frf:
    """One function for all three pairs, so they share every setting."""
    nperseg = kw['nperseg']
    if len(u) < nperseg:
        raise ValueError(f'the window ({len(u)} samples) is shorter than one segment '
                         f'({nperseg}); lower --segment-s')
    f, suu = signal.welch(u, **kw)
    _, syy = signal.welch(y, **kw)
    _, suy = signal.csd(u, y, **kw)
    n_avg = (len(u) - nperseg) // (nperseg - nperseg // 2) + 1   # welch's default 50 % overlap
    with np.errstate(divide='ignore', invalid='ignore'):
        coh = np.abs(suy)**2 / (suu * syy)
        eps = np.sqrt(1 - coh) / (np.sqrt(coh) * np.sqrt(2 * n_avg))
        return Frf(f, suy / suu, syy / np.conj(suy), coh, eps, n_avg)


def band_mask(f, coh, row) -> np.ndarray:
    """
    Return the bins to report.

    Irregular: f_min_hz..f_max_hz and coh >= COHERENCE_MIN. Regular: the one bin at 1 / period_s.
    """
    if row['kind'] == 'regular':
        ok = np.zeros(len(f), bool)
        ok[np.argmin(np.abs(f - 1 / float(row['period_s'])))] = True
        return ok
    if row['kind'] == 'irregular':
        f_top = min(float(row['f_max_hz']), F_TOP_FP / float(row['peak_period_s']))
        in_band = (f >= float(row['f_min_hz'])) & (f <= f_top)
        return in_band & (coh >= COHERENCE_MIN)
    raise ValueError(f"run {row['run']} is of kind '{row['kind']}'; only regular and irregular")


def smooth(values, ok, n_smooth) -> np.ndarray:
    """Return the moving average over the ok bins only (NaN outside)."""

    def mean(y):
        return np.asarray(ndimage.uniform_filter1d(y, n_smooth, mode='constant'), float)

    num = mean(np.where(ok, values, 0.0))
    den = mean(ok.astype(float))
    out = np.full(len(values), np.nan)
    out[ok] = num[ok] / den[ok]
    return out


def undo_travel(y, dt, depth_m, distance_m) -> np.ndarray:
    """Return y advanced by linear theory's travel over distance_m, per frequency (all-pass)."""
    n = len(y)
    pad = 2 ** int(np.ceil(np.log2(2 * n)))
    f = np.fft.rfftfreq(pad, dt)
    k = np.zeros(len(f))
    for i in range(1, len(f)):
        k[i] = gw.wavenumber_depth(1.0 / f[i], depth_m) / depth_m
    return np.fft.irfft(np.fft.rfft(y - y.mean(), pad) * np.exp(1j * k * distance_m), pad)[:n]


def residual_delay(f, G, ok) -> tuple[np.ndarray, float]:
    """Return (residual phase, delay_s); G is measured on a wave with the travel removed."""
    res = np.unwrap(np.angle(G[ok])) - np.pi / 2   # theory: the wave leads the stroke by 90 deg
    res -= 2 * np.pi * np.round(np.median(res) / (2 * np.pi))
    if ok.sum() < 2:
        return res, np.nan
    return res, -np.polyfit(2 * np.pi * f[ok], res, 1)[0]


# ---------------------------------------------------------------- output

def print_table(cols, wm) -> None:
    """Print one row per yaml table period in the band."""
    T = cols['period_s']
    stroke = 'drive' in cols
    periods = T if len(T) == 1 else [p for p in wm.gain_periods if T.min() <= p <= T.max()]
    head = 'T [s]  gain   ±      coh   table (yaml)'
    if stroke:
        head += '   drive  ±      water  ±      coh'
    print(head)
    for p in periods:
        i = int(np.argmin(np.abs(T - p)))
        line = (f"{p:<6.2f} {cols['gain'][i]:.3f}  {cols['gain_err'][i]:.3f}  "
                f"{cols['coh'][i]:.2f}  {gw.transfer_gain(wm, p):<13.3f}")
        if stroke:
            line += (f"  {cols['drive'][i]:.3f}  {cols['drive_err'][i]:.3f}  "
                     f"{cols['water'][i]:.3f}  {cols['water_err'][i]:.3f}  "
                     f"{cols['coh_water'][i]:.2f}")
        print(line)


def write_csv(path, cols):
    """Write all ok bins, raw and smoothed."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(cols)
        writer.writerows(zip(*(np.round(v, 6) for v in cols.values())))


def write_record(path, argv, info) -> None:
    """Write <output>.txt: what the result was made from (motion_files' needs a matrix)."""
    lines = [
        f'generated    {datetime.now().astimezone().isoformat(timespec="seconds")}',
        f'command      transfer_function {shlex.join(argv)}',
        f'folder       {os.getcwd()}',
        f'commit       {motion_files.source_commit()}',
        *info,
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def plot(path, cols, wm) -> None:
    """Plot against period: gain, drive and water, coherence, residual phase."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    T = cols['period_s']
    stroke = 'drive' in cols
    fig, axes = plt.subplots(4 if stroke else 3, 1, sharex=True, figsize=(9, 11 if stroke else 8))
    ax_g, *ax_dw, ax_c, ax_p = axes

    ax_g.plot(T, cols['gain_raw'], '.', ms=3, color='0.6', label='raw')
    ax_g.plot(T, cols['gain'], label='smoothed')
    ax_g.fill_between(T, cols['gain'] - cols['gain_err'], cols['gain'] + cols['gain_err'],
                      alpha=0.3)
    if wm.gain_periods:
        ax_g.plot(wm.gain_periods, wm.gain_values, 'o', label='table (yaml)')
    ax_g.set_ylabel('gain = |G_cmd| / G_lin')

    if stroke:
        for name in ('drive', 'water'):
            ax_dw[0].plot(T, cols[name], label=name)
            ax_dw[0].fill_between(T, cols[name] - cols[f'{name}_err'],
                                  cols[name] + cols[f'{name}_err'], alpha=0.3)
        ax_dw[0].axhline(1.0, ls='--', color='0.5', lw=0.8)
        ax_dw[0].set_ylabel('1.0 = ideal (waterline)')

    ax_c.plot(T, cols['coh'], label='command → wave')
    if stroke:
        ax_c.plot(T, cols['coh_water'], label='stroke → wave')
    ax_c.axhline(COHERENCE_MIN, ls='--', color='tab:red', lw=0.8)
    ax_c.set_ylabel('coherence')

    ax_p.plot(T, cols['phase_res_rad'], label='G_cmd')
    if stroke:
        ax_p.plot(T, cols['phase_res_water_rad'], label='G_water')
    ax_p.set_ylabel('residual phase [rad]')
    ax_p.set_xlabel('period [s]')

    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def analyse(args, argv, config) -> None:
    # 1. wavemaker; depth from the command line, warn if it differs from the yaml
    wm = gw.load_wavemaker(args.wavemaker, config)
    yaml_depth = wm.water_depth
    if wm.gain_periods and args.depth_m != yaml_depth:
        print(f'warning: the gain table is for {yaml_depth} m, not {args.depth_m} m')
    wm.water_depth = args.depth_m                 # the flap's hinge term uses it too

    # 2. command
    x_file, dt, table, row = read_command(args.file, args.run)
    x = x_file[round(float(row['start_s']) / dt):round(float(row['end_s']) / dt)]

    # 3. record
    record_dt, channels = read_record(args.record)
    ratio = dt / record_dt
    q = round(ratio)
    if q < 1 or abs(ratio - q) > 1e-3:
        raise ValueError(f'the record at {1 / record_dt:g} Hz is not a whole multiple '
                         f'of the file at {1 / dt:g} Hz')
    eta_all = record_channel(channels, args.probe, args.probe_scale, q)
    s_all = record_channel(channels, args.stroke, args.stroke_scale, q) if args.stroke else None
    eta_travel = undo_travel(eta_all, dt, args.depth_m, args.probe_distance_m)

    # 4. timeline: anchor on the first run, walk the runs, fit the drift; or --offset-s
    played = [r for r in table if r['end_s']]
    first = played[0]
    y_walk = s_all if s_all is not None else lag_90(eta_travel)    # the probe: decision 6
    rests = [float(b['start_s']) - float(a['end_s']) for a, b in zip(table, table[1:])]
    min_sep = round(min(rests, default=10 * run_period(first)) / dt)
    walked: Timeline | None = None
    if args.offset_s is not None:
        a_k, how = args.offset_s, '--offset-s'
        own = None
        if args.drift_ppm is None:                  # the drift from this run's own pieces
            try:
                own_pieces = walk(x_file, y_walk, dt, [row], a_k)
                own = fit_timeline(own_pieces) if own_pieces else None
            except ValueError as e:
                print(f'warning: no drift from run {args.run} at --offset-s ({e}); '
                      'taking 0')
        eps_fit = own
    else:
        if s_all is not None:
            anchor_s, quality = anchor_from_stroke(x_file, s_all, dt, first, min_sep)
            how = 'from the stroke'
        else:
            anchor_s, quality = anchor_from_envelopes(x_file, eta_travel, dt, first,
                                                      run_period(first), min_sep)
            how = 'from the envelopes'
        how += (f"; anchor the only match, run {first['run']}" if np.isinf(quality)
                else f"; anchor {quality:.1f}x the next, run {first['run']}")
        if quality < MATCH_MIN:
            raise ValueError(f'anchor {anchor_s:.1f} s ({how}): an unclear match, below '
                             f'{MATCH_MIN}x; give --offset-s')
        pieces_walked = walk(x_file, y_walk, dt, played, anchor_s)
        if not any(k == args.run for k, _, _, _ in pieces_walked):
            raise ValueError(f'run {args.run} is not inside the record (it stops at '
                             f'{len(eta_all) * dt:.1f} s)')
        walked = fit_timeline(pieces_walked)
        a_k = walked.a[args.run]
        eps_fit = walked
    if args.drift_ppm is not None:
        eps, drift = args.drift_ppm * 1e-6, f'{args.drift_ppm:+.0f} ppm (--drift-ppm)'
    elif eps_fit is not None and eps_fit.eps_pieces:
        eps = eps_fit.eps
        drift = (f"{eps * 1e6:+.0f} ppm (catman's clock against the Pi's), "
                 f'{eps_fit.eps_pieces} pieces in {eps_fit.eps_runs} '
                 f"run{'s' if eps_fit.eps_runs > 1 else ''}, rms {eps_fit.rms_s * 1000:.1f} ms")
        if eps_fit.rms_s > DRIFT_RES_MAX_S:
            print(f'warning: the pieces scatter {eps_fit.rms_s * 1000:.0f} ms (rms) about the '
                  'drift fit; something moved the timeline inside a run')
    else:
        eps, drift = 0.0, f'not measured (less than {DRIFT_MIN_S:.0f} s in runs of 2+ pieces)'

    # 5. window, read on the file's time base
    start_s = float(row['start_s'])
    i_a, i_b = round(start_s / dt), round(float(row['end_s']) / dt)
    n_before = max(0, min(round(STILL_S / dt), int((a_k + start_s) / dt) - 1))
    skip = round(float(row['ramp_s']) / dt)
    eta_run = on_file_time(eta_travel, dt, a_k, eps, start_s, i_a - n_before, i_b)
    s_run = (on_file_time(s_all, dt, a_k, eps, start_s, i_a - n_before, i_b)
             if s_all is not None else None)
    eta, s = cut_window(n_before, len(x), skip, eta_run, s_run)
    assert eta is not None                  # the probe is never None, only the stroke
    x = x[skip:len(x) - skip]
    check_still(eta_run, n_before, dt, eta.std())

    # 6. estimate (Changes 6, 8); regular: a whole number of periods per segment
    if row['kind'] == 'regular':   # whole periods, at most half the window: >= 3 averages
        period_s = float(row['period_s'])
        segment_s = min(args.segment_s, len(x) * dt / 2)
        nperseg = round(max(1, int(segment_s / period_s)) * period_s / dt)
    else:
        nperseg = round(args.segment_s / dt)
    kw = {'fs': 1 / dt, 'window': 'hann', 'nperseg': nperseg, 'detrend': 'constant'}
    cmd = frf(x, eta, kw)

    # 7. gain and smoothing
    ok = band_mask(cmd.f, cmd.coh, row)
    if not ok.any():
        raise ValueError(f'no bin in the band has coherence >= {COHERENCE_MIN}')
    kh, g_lin_wl, g_lin = linear_ratio(cmd.f, ok, wm, args.depth_m)
    df = cmd.f[1] - cmd.f[0]
    n_smooth = 1 if row['kind'] == 'regular' else max(1, round(args.smooth_hz / df))

    def gain_cols(name, G, frf_, g_ref):
        g = np.abs(G) / g_ref
        g_s = smooth(g, ok, n_smooth)
        return {f'{name}_raw': g[ok], name: g_s[ok],
                f'{name}_err': (g_s * smooth(frf_.eps, ok, n_smooth) / np.sqrt(n_smooth))[ok]}

    cols = {'period_s': 1 / cmd.f[ok], 'f_hz': cmd.f[ok],
            **gain_cols('gain', cmd.h1, cmd, g_lin), 'coh': cmd.coh[ok]}

    # 8. residual phase
    cols['phase_res_rad'], delay_s = residual_delay(cmd.f, cmd.h1, ok)
    stroke_info: list[str] = []
    if s is not None:          # drive and water, with the same settings as cmd
        x_wl = to_waterline(x, wm.attachment_height, wm, args.depth_m)
        s_wl = to_waterline(s, args.stroke_height_m, wm, args.depth_m)
        drive = frf(x_wl, s_wl, kw)        # 1.0 = the flap follows the command
        water = frf(s_wl, eta, kw)
        cols |= gain_cols('drive', drive.h1, drive, 1.0)
        cols |= gain_cols('water', water.h1, water, g_lin_wl)
        cols['water_h2'] = (np.abs(water.h2) / g_lin_wl)[ok]
        cols['coh_water'] = water.coh[ok]
        cols['phase_res_water_rad'], delay_water_s = residual_delay(water.f, water.h1, ok)
        cmd_wl = cmd.h1 / to_waterline(1.0, wm.attachment_height, wm, args.depth_m)
        chain = np.max(np.abs(cmd_wl - drive.h1 * water.h1)[ok] / np.abs(cmd_wl)[ok])
        stroke_info = [
            f'water delay  {delay_water_s:.2f} s (G_water: 0 if distance and depth are right)',
            f'chain        G_cmd vs drive x water: largest difference {chain:.0%} in the band']

    # 9. output
    info = [
        f'wavemaker    {wm.name} ({wm.type}), {os.path.realpath(config)}',
        f'water depth  {args.depth_m} m (--depth-m; the yaml has {yaml_depth} m)',
        f'motion file  {args.file}, run {args.run} ({row["kind"]}), dt {dt} s',
        f'record       {args.record}, probe {args.probe} x {args.probe_scale}, '
        f'{args.probe_distance_m} m from the wavemaker',
    ]
    if s is not None:
        info.append(f'stroke       {args.stroke} x {args.stroke_scale}, sensor at '
                    f'{args.stroke_height_m} m')
    info += [
        f"offset       {a_k:.3f} s at the run's start ({how})",
    ]
    if walked is not None:
        runs = sorted(walked.a)
        a0, s0 = walked.a[runs[0]], {int(r['run']): float(r['start_s']) for r in table}
        info += [
            'runs         at each start [s]: ' + ' '.join(f'{walked.a[k]:.3f}' for k in runs),
            'late         goal start after the file, drift removed [ms]: ' + ' '.join(
                f'{(walked.a[k] - a0 - eps * (s0[k] - s0[runs[0]])) * 1000:.0f}' for k in runs),
        ]
    info += [
        f'drift        {drift}',
        f'window       {len(x) * dt:.1f} s, {skip * dt:.1f} s skipped at each end',
        f'estimate     H1, {nperseg * dt:.1f} s Hann segments, {cmd.n_avg} averages, '
        f'df {df:.4f} Hz',
        f'smoothing    {n_smooth} bins ({n_smooth * df:.3f} Hz)',
        f"delay        {delay_s:.2f} s (G_cmd: the drive plus the offset's error)",
    ]
    info += stroke_info

    print_table(cols, wm)
    print('\n'.join(info))
    write_csv(args.output + '.csv', cols)
    write_record(args.output + '.txt', argv, info)
    if args.plot:
        plot(args.output + '.png', cols, wm)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Transfer function from a run and a record.')
    parser.add_argument('file', help='.dat with its run table <file>.csv')
    parser.add_argument('wavemaker', help='name in wavemakers.yaml')
    parser.add_argument('--run', type=int, required=True, help='run number in the run table')
    parser.add_argument('--record', required=True, help='catman ASCII export (.ASC) or CSV')
    parser.add_argument('--probe', required=True, help='column name of the wave probe')
    parser.add_argument('--probe-distance-m', type=float, required=True)
    parser.add_argument('--probe-scale', type=float, default=1.0, help='to m, e.g. 0.001 for mm')
    parser.add_argument('--stroke', help="column name of the flap's draw wire")
    parser.add_argument('--stroke-height-m', type=float, help='above the floor; with --stroke')
    parser.add_argument('--stroke-scale', type=float, default=1.0)
    parser.add_argument('--depth-m', type=float, required=True, help='measured; no yaml default')
    parser.add_argument('--config', help='another wavemakers.yaml')
    parser.add_argument('--offset-s', type=float,
                        help="this run's start in the record less its file time; skips the walk")
    parser.add_argument('--drift-ppm', type=float,
                        help="catman's clock against the Pi's; skips its fit (0: none)")
    parser.add_argument('--segment-s', type=float, default=40.0)
    parser.add_argument('--smooth-hz', type=float, default=0.05, help='0: raw gain')
    parser.add_argument('-o', '--output', required=True, help='base name for .csv, .txt, .png')
    parser.add_argument('--plot', action='store_true')

    argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(argv)
    if args.stroke and args.stroke_height_m is None:
        parser.error('--stroke needs --stroke-height-m (the sensor above the tank floor)')
    try:
        config = args.config or gw.default_config_path()
    except FileNotFoundError as e:
        parser.error(str(e))
    try:
        analyse(args, argv, config)
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
