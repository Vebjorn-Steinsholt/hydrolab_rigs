#!/usr/bin/env python3
"""
Transfer function of a wavemaker from one run of a motion file and a catman record.

G_cmd (command -> wave, the gain table's number), and with the flap's stroke G_drive
(command -> flap) and G_water (flap -> wave). See transfer_function_sketch.md.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import os
import shlex
import sys

# One group, alphabetical (ament_flake8's import order treats the local modules as third party).
import generate_waves as gw
import motion_files
import numpy as np
import play_dat
from scipy import ndimage, signal

COHERENCE_MIN = 0.8      # decision 8
STILL_S = 10.0           # probe checked over this long before the run (Change 5)
STILL_RATIO = 0.1        # warn if the probe's std before the run is above this x the run's
STILL_FLOOR_M = 0.001    # but never below this: ~3x the probe's noise (check on a still record)
MATCH_MIN = 2.0          # offset match: peak / next-best peak below this is refused
ENVELOPE_SMOOTH_TP = 3   # smooth the envelope over this many wave periods (Change 4)


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


def read_record_csv(path: str) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Time [s] and {column header: values}. Refuse a non-uniform time column."""
    with open(path, newline='') as f:
        reader = csv.reader(f)
        names = next(reader)
        values = np.array([[float(v) for v in r] for r in reader if r])
    t = values[:, 0]
    step = np.diff(t)
    if len(t) < 2 or step.min() <= 0 or np.abs(step - step.mean()).max() > 1e-3 * step.mean():
        raise ValueError(f'{path}: the time column ({names[0]}) is not uniform')
    return t, {name: values[:, i] for i, name in enumerate(names) if i > 0}


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

def offset_from_stroke(x_file, s_all, dt, min_sep) -> tuple[float, float]:
    """
    Return (offset_s, match quality) by cross-correlating command and stroke.

    Refuse a negative peak (stroke and command point opposite ways).
    """
    lags = signal.correlation_lags(len(s_all), len(x_file), mode='valid')
    corr = signal.correlate(s_all - s_all.mean(), x_file - x_file.mean(), mode='valid')
    i_peak = int(np.argmax(np.abs(corr)))
    if corr[i_peak] < 0:
        raise ValueError("the stroke matches the command upside down: the command's + is "
                         'towards the beach; check the sign of --stroke-scale')
    return lags[i_peak] * dt, match_quality(corr, i_peak, min_sep)


def envelope(y, dt, smooth_s) -> np.ndarray:
    """Envelope of a signal, using the Hilbert transform and a moving average."""
    analytic = np.asarray(signal.hilbert(y - y.mean()))
    return ndimage.uniform_filter1d(np.abs(analytic), max(1, round(smooth_s / dt)))


def offset_from_envelopes(x_file, eta_all, dt, tp_s, travel_s, min_sep) -> tuple[float, float]:
    """(offset_s, match quality) from the envelopes, minus the travel time."""
    env_x = envelope(x_file, dt, ENVELOPE_SMOOTH_TP * tp_s)
    env_eta = envelope(eta_all, dt, ENVELOPE_SMOOTH_TP * tp_s)
    lags = signal.correlation_lags(len(env_eta), len(env_x), mode='valid')
    corr = signal.correlate(env_eta - env_eta.mean(), env_x - env_x.mean(), mode='valid')
    i_peak = int(np.argmax(corr))
    return lags[i_peak] * dt - travel_s, match_quality(corr, i_peak, min_sep)


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
        in_band = (f >= float(row['f_min_hz'])) & (f <= float(row['f_max_hz']))
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
    res = np.unwrap(np.angle(G[ok]))
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
    # 1. wavemaker; depth from the command line, warn if it differs from the yaml (Change 7)
    wm = gw.load_wavemaker(args.wavemaker, config)
    yaml_depth = wm.water_depth
    if wm.gain_periods and args.depth_m != yaml_depth:
        print(f'warning: the gain table is for {yaml_depth} m, not {args.depth_m} m')
    wm.water_depth = args.depth_m                 # the flap's hinge term uses it too

    # 2. command (Change 3)
    x_file, dt, table, row = read_command(args.file, args.run)
    x = x_file[round(float(row['start_s']) / dt):round(float(row['end_s']) / dt)]

    # 3. record (Change 3)
    t, channels = read_record_csv(args.record)
    ratio = dt / (t[1] - t[0])
    q = round(ratio)
    if q < 1 or abs(ratio - q) > 1e-3:
        raise ValueError(f'the record at {1 / (t[1] - t[0]):g} Hz is not a whole multiple '
                         f'of the file at {1 / dt:g} Hz')
    eta_all = record_channel(channels, args.probe, args.probe_scale, q)
    s_all = record_channel(channels, args.stroke, args.stroke_scale, q) if args.stroke else None
    # the record may stop after the last run, before the file's final rest
    x_match = x_file[:round(max(float(r['end_s']) for r in table if r['end_s']) / dt) + 1]

    # 4. offset: --offset-s, else offset_from_stroke, else offset_from_envelopes (Change 4)
    tp_s = float(row['peak_period_s'] or row['period_s'])
    travel_s = args.probe_distance_m / group_velocity(1 / tp_s, args.depth_m)
    rests = [float(b['start_s']) - float(a['end_s']) for a, b in zip(table, table[1:])]
    min_sep = round(min(rests, default=10 * tp_s) / dt)
    if args.offset_s is not None:
        offset_s, how = args.offset_s, '--offset-s'
    else:
        if len(eta_all) < len(x_match):
            raise ValueError(f'the record ({len(eta_all) * dt:.1f} s) is shorter than the file '
                             f'({len(x_match) * dt:.1f} s); give --offset-s')
        if s_all is not None:
            offset_s, quality = offset_from_stroke(x_match, s_all, dt, min_sep)
            how = 'from the stroke'
        else:
            offset_s, quality = offset_from_envelopes(x_match, eta_all, dt, tp_s, travel_s,
                                                      min_sep)
            how = 'from the envelopes'
        how += f', match {quality:.1f}x the next'
        if quality < MATCH_MIN:
            raise ValueError(f'offset {offset_s:.1f} s ({how}): an unclear match, below '
                             f'{MATCH_MIN}x; give --offset-s')

    # 5. window (Change 5)
    i0 = round((offset_s + float(row['start_s'])) / dt)
    skip = round(float(row['ramp_s']) / dt)
    eta_all = undo_travel(eta_all, dt, args.depth_m, args.probe_distance_m)

    eta, s = cut_window(i0, len(x), skip, eta_all, s_all)
    assert eta is not None                  # the probe is never None, only the stroke
    x = x[skip:len(x) - skip]
    check_still(eta_all, i0, dt, eta.std())

    # 6. estimate (Changes 6, 8); regular: a whole number of periods per segment
    if row['kind'] == 'regular':
        period_s = float(row['period_s'])
        nperseg = round(max(1, round(args.segment_s / period_s)) * period_s / dt)
    else:
        nperseg = round(args.segment_s / dt)
    kw = {'fs': 1 / dt, 'window': 'hann', 'nperseg': nperseg, 'detrend': 'constant'}
    cmd = frf(x, eta, kw)

    # 7. gain and smoothing (Change 7)
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

    # 8. residual phase (Change 9)
    cols['phase_res_rad'], delay_s = residual_delay(cmd.f, cmd.h1, ok)
    stroke_info: list[str] = []
    if s is not None:          # drive and water (Change 8), with the same settings as cmd
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

    # 9. output (Change 10)
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
        f"offset       {offset_s:.2f} s from the record's first sample ({how})",
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
    parser.add_argument('--record', required=True, help='catman CSV export')
    parser.add_argument('--probe', required=True, help='column name of the wave probe')
    parser.add_argument('--probe-distance-m', type=float, required=True)
    parser.add_argument('--probe-scale', type=float, default=1.0, help='to m, e.g. 0.001 for mm')
    parser.add_argument('--stroke', help="column name of the flap's draw wire")
    parser.add_argument('--stroke-height-m', type=float, help='above the floor; with --stroke')
    parser.add_argument('--stroke-scale', type=float, default=1.0)
    parser.add_argument('--depth-m', type=float, required=True, help='measured (decision 7)')
    parser.add_argument('--config', help='another wavemakers.yaml')
    parser.add_argument('--offset-s', type=float, help='file start in the record; skips the match')
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
