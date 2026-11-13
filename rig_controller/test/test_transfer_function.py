"""
Tests for src/transfer_function.py, on a synthetic record with a known drive and water.

The motion file comes from generate_waves (a regular run, a rest, an irregular run). The record
is made from it as catman would see it: 30 s of still water, then the file, at 200 Hz, with
  - the flap's stroke: the command through a known drive (gain 0.9, a first-order lag of 77 ms
    and a 100 ms delay, as the oscillator measured), at a sensor 1.20 m above the floor, in mm,
    with an arbitrary zero;
  - the probe: that stroke at the waterline through a known water (0.83 x linear theory and the
    travel phase to 3 m), plus the probe's noise, in mm.
The tool must give those back. If it doesn't, the error is in the code, not in the tank.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_transfer_function.py -v
"""

import csv
import importlib.util
import pathlib
import re
import sys

import numpy as np
import pytest
from scipy import signal

SRC = pathlib.Path(__file__).resolve().parents[1] / 'src'
CONFIG = str(pathlib.Path(__file__).resolve().parents[2]
             / 'wavemaker_bringup' / 'config' / 'wavemakers.yaml')
sys.path.insert(0, str(SRC))  # transfer_function imports generate_waves and play_dat from there
_spec = importlib.util.spec_from_file_location('transfer_function', SRC / 'transfer_function.py')
tf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tf)
gw = tf.gw
play_dat = tf.play_dat

DEPTH_M = 1.0              # the yaml's, so no depth warning
DISTANCE_M = 3.0
SENSOR_M = 1.20            # draw wire above the floor
RATE_HZ = 200.0            # the record; the file is 50 Hz
STILL_BEFORE_S = 30.0      # the record starts this long before the file
DRIVE_GAIN, DRIVE_LAG_S, DRIVE_DELAY_S = 0.9, 0.077, 0.100
WATER_GAIN = 0.83
PROBE_NOISE_M = 0.0003
STROKE_ZERO_MM = 250.0     # the draw wire's arbitrary zero

HEADER = ['kind', 'period_s', 'height_m', 'steepness', 'n_periods', 'hs_m', 'tp_s', 'gamma',
          'duration_s', 'seed', 'n_ramp', 'pause_s']
MATRIX = [
    {'kind': 'regular', 'period_s': 1.3, 'steepness': '1/60', 'n_periods': 60, 'n_ramp': 3,
     'pause_s': 60},
    {'kind': 'irregular', 'hs_m': 0.03, 'tp_s': 1.2, 'gamma': 3.3, 'duration_s': 300,
     'seed': 7, 'n_ramp': 5, 'pause_s': 60},
]


# ---------------------------------------------------------------- the synthetic session

def drive_response(f):
    """Command -> flap at the waterline, as a complex gain (numpy's e^{-i w t} convention)."""
    w = 2 * np.pi * f
    return DRIVE_GAIN * np.exp(-1j * w * DRIVE_DELAY_S) / (1 + 1j * w * DRIVE_LAG_S)


def water_response(wm, f):
    """Flap at the waterline -> wave at the probe: WATER_GAIN x theory, and the travel."""
    out = np.zeros(len(f), complex)
    for i, fi in enumerate(f):
        if 0.05 < fi < 5.0:      # the command has no energy outside; sinh(kh) overflows above
            kh = gw.wavenumber_depth(1 / fi, DEPTH_M)
            out[i] = (WATER_GAIN * gw.stroke_ratio(kh, wm)
                      * np.exp(-1j * kh / DEPTH_M * DISTANCE_M))
    return out


def through(y, dt, response):
    """Filter y by a frequency response, zero-padded so nothing wraps around."""
    n = len(y)
    pad = 2 ** int(np.ceil(np.log2(2 * n)))
    f = np.fft.rfftfreq(pad, dt)
    return np.fft.irfft(np.fft.rfft(y, pad) * response(f), pad)[:n]


def waterline(wm, height_m):
    """Metres at the waterline per metre at this height above the floor (the flap)."""
    return (DEPTH_M - wm.hinge_height) / (height_m - wm.hinge_height)


def write_record(path, x_file, dt, wm, *, stroke_sign=1.0, reflection=None, seed=0,
                 cut_end_s=0.0):
    """Write the catman CSV for the whole file: Time [s], WP1 [mm], Stroke [mm]."""
    rng = np.random.default_rng(seed)
    x = np.concatenate([np.zeros(round(STILL_BEFORE_S / dt)), x_file,
                        np.zeros(round(30.0 / dt))])
    x_wl = x * waterline(wm, wm.attachment_height)
    s_wl = through(x_wl, dt, drive_response)
    eta = through(s_wl, dt, lambda f: water_response(wm, f))
    if reflection is not None:                          # (delay_s, scale)
        lag = round(reflection[0] / dt)
        eta[lag:] += reflection[1] * eta[:-lag]
    q = round(RATE_HZ * dt)                             # 4: 200 Hz against 0.02 s
    eta = signal.resample_poly(eta, q, 1) + rng.normal(0, PROBE_NOISE_M, len(x) * q)
    stroke = signal.resample_poly(s_wl / waterline(wm, SENSOR_M), q, 1)
    n = len(eta) - round(cut_end_s * RATE_HZ)
    t = np.arange(n) / RATE_HZ
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Time', 'WP1', 'Stroke'])
        w.writerows(zip(np.round(t, 6), np.round(eta[:n] * 1000, 4),
                        np.round(stroke_sign * stroke[:n] * 1000 + STROKE_ZERO_MM, 4)))
    return str(path)


@pytest.fixture(scope='module')
def session(tmp_path_factory):
    """Make the motion file, its run table and a record, once for all tests."""
    d = tmp_path_factory.mktemp('session')
    matrix = d / 'matrix.csv'
    with open(matrix, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=HEADER, restval='')
        writer.writeheader()
        writer.writerows(MATRIX)
    out = str(d / 'item')
    assert gw.main(['ladertanken', str(matrix), '-o', out, '--config', CONFIG,
                    '--no-plot']) == 0
    motion = play_dat.read_motion(out + '.dat')
    x_file = play_dat.select_channel(motion, 'position')
    dt = motion.sample_interval_s
    wm = gw.load_wavemaker('ladertanken', CONFIG)
    record = write_record(d / 'record.csv', x_file, dt, wm)
    return {'dir': d, 'dat': out + '.dat', 'record': record, 'x_file': x_file, 'dt': dt,
            'wm': wm}


def run_tool(session, *extra, run=1, record=None, stroke=True, name='tf'):
    """Run the tool; return (exit code, stdout, stderr, the output CSV's columns or None)."""
    out = str(session['dir'] / name)
    argv = [session['dat'], 'ladertanken', '--run', str(run),
            '--record', record or session['record'], '--probe', 'WP1', '--probe-scale',
            '0.001', '--probe-distance-m', str(DISTANCE_M), '--depth-m', str(DEPTH_M),
            '--config', CONFIG, '-o', out, *extra]
    if stroke:
        argv += ['--stroke', 'Stroke', '--stroke-scale', '0.001',
                 '--stroke-height-m', str(SENSOR_M)]
    return argv, out


def call(capsys, session, *extra, **kw):
    argv, out = run_tool(session, *extra, **kw)
    code = tf.main(argv)
    captured = capsys.readouterr()
    cols = None
    if code == 0:
        with open(out + '.csv', newline='') as f:
            rows = list(csv.reader(f))
        cols = {name: np.array([float(r[i]) for r in rows[1:]])
                for i, name in enumerate(rows[0])}
    return code, captured.out, captured.err, cols


def printed_offset(out):
    return float(re.search(r'offset\s+(-?[\d.]+) s', out).group(1))


# ---------------------------------------------------------------- the offset

def test_offset_from_the_stroke(capsys, session):
    code, out, err, _ = call(capsys, session)
    assert code == 0, err
    assert 'from the stroke' in out
    # the stroke lags the command by the drive's delay and lag; nothing else
    assert printed_offset(out) == pytest.approx(STILL_BEFORE_S, abs=0.3)


def test_offset_from_the_envelopes(capsys, session):
    code, out, err, _ = call(capsys, session, stroke=False, name='env')
    assert code == 0, err
    assert 'from the envelopes' in out
    assert printed_offset(out) == pytest.approx(STILL_BEFORE_S, abs=1.0)


def test_offset_given(capsys, session):
    code, out, err, _ = call(capsys, session, '--offset-s', str(STILL_BEFORE_S), name='given')
    assert code == 0, err
    assert '--offset-s' in out


# ---------------------------------------------------------------- the irregular run

def test_irregular_recovers_drive_water_and_gain(capsys, session):
    code, out, err, cols = call(capsys, session)
    assert code == 0, err
    f = cols['f_hz']
    assert len(f) > 20                         # most of the band is trusted
    drive = np.abs(drive_response(f))          # 0.9 and the lag's roll-off
    for name, expected in (('drive', drive), ('water', WATER_GAIN),
                           ('gain', WATER_GAIN * drive)):
        error = np.abs(cols[name] - expected)
        # within the reported error, with a margin for the smoothing over a sloping curve
        assert np.all(error <= 3 * cols[f'{name}_err'] + 0.01), (
            name, f[np.argmax(error)], cols[name][np.argmax(error)])


def test_water_phase_is_flat_and_cmd_phase_is_a_delay(capsys, session):
    code, out, err, cols = call(capsys, session)
    assert code == 0, err
    # G_water: stroke and probe are sampled together and the travel is linear theory
    assert np.abs(cols['phase_res_water_rad']).max() < 0.1
    delay = float(re.search(r'water delay\s+(-?[\d.]+) s', out).group(1))
    assert abs(delay) < 0.02
    # G_cmd: the drive's delay plus its lag's phase (curved), plus the offset's error
    assert float(re.search(r'\bdelay\s+(-?[\d.]+) s', out).group(1)) < 1.0


def test_chain_closes(capsys, session):
    code, out, err, _ = call(capsys, session)
    assert code == 0, err
    chain = float(re.search(r'largest difference (\d+)%', out).group(1))
    assert chain <= 5


# ---------------------------------------------------------------- the regular run

def test_regular_run_gives_one_row(capsys, session):
    code, out, err, cols = call(capsys, session, run=0, name='regular')
    assert code == 0, err
    assert cols['period_s'] == pytest.approx([1.3], rel=1e-3)
    expected = WATER_GAIN * abs(drive_response(np.array([1 / 1.3]))[0])
    assert cols['gain'][0] == pytest.approx(expected, rel=0.02)


# ---------------------------------------------------------------- reflection

def test_reflection_is_smoothed_away(capsys, session):
    """
    A copy of the wave 19 s later at 0.1: a ripple every 1/19 Hz in the raw gain.

    100 s segments, so the reflection stays coherent within a segment and shows as the ripple
    (in 40 s segments half of it is lost to the window and it is smaller). Compared in the middle
    of the band: at its edges the error is the little energy there, not the reflection.
    """
    record = write_record(session['dir'] / 'reflected.csv', session['x_file'], session['dt'],
                          session['wm'], reflection=(19.0, 0.1))
    code, out, err, cols = call(capsys, session, '--segment-s', '100', record=record,
                                name='reflected')
    assert code == 0, err
    mid = (cols['f_hz'] > 0.6) & (cols['f_hz'] < 2.0)
    expected = WATER_GAIN * np.abs(drive_response(cols['f_hz'][mid]))

    def rms(y):
        return np.sqrt(np.mean((y[mid] - expected) ** 2))

    assert rms(cols['gain_raw']) > 0.02        # the ripple is there
    assert rms(cols['gain']) < rms(cols['gain_raw']) / 3


# ---------------------------------------------------------------- refusals

def test_flipped_stroke_is_refused(capsys, session):
    record = write_record(session['dir'] / 'flipped.csv', session['x_file'], session['dt'],
                          session['wm'], stroke_sign=-1.0)
    code, _, err, _ = call(capsys, session, record=record, name='flipped')
    assert code == 1
    assert 'upside down' in err


def test_depth_is_required(session):
    argv, _ = run_tool(session)
    i = argv.index('--depth-m')
    with pytest.raises(SystemExit):
        tf.main(argv[:i] + argv[i + 2:])


def test_stroke_needs_its_height(session, capsys):
    argv, _ = run_tool(session)
    i = argv.index('--stroke-height-m')
    with pytest.raises(SystemExit):
        tf.main(argv[:i] + argv[i + 2:])
    assert '--stroke-height-m' in capsys.readouterr().err


def test_run_not_in_the_table(capsys, session):
    code, _, err, _ = call(capsys, session, run=7)
    assert code == 1
    assert 'run 7 is not in the run table' in err


def test_no_run_table(capsys, session, tmp_path):
    dat = tmp_path / 'bare.dat'
    dat.write_bytes(pathlib.Path(session['dat']).read_bytes())
    argv, _ = run_tool(session)
    argv[0] = str(dat)
    assert tf.main(argv) == 1
    assert 'no run table' in capsys.readouterr().err


def test_run_table_without_ramp_s(capsys, session, tmp_path):
    dat = tmp_path / 'old.dat'
    dat.write_bytes(pathlib.Path(session['dat']).read_bytes())
    with open(pathlib.Path(session['dat']).with_suffix('.csv'), newline='') as f:
        reader = csv.DictReader(f)
        fields = [c for c in reader.fieldnames if c != 'ramp_s']
        rows = [{k: r[k] for k in fields} for r in reader]
    with open(tmp_path / 'old.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    argv, _ = run_tool(session)
    argv[0] = str(dat)
    assert tf.main(argv) == 1
    assert 'no ramp_s' in capsys.readouterr().err


def test_column_not_in_the_record(capsys, session):
    argv, _ = run_tool(session)
    argv[argv.index('WP1')] = 'WP9'
    assert tf.main(argv) == 1
    err = capsys.readouterr().err
    assert "no column 'WP9'" in err and 'WP1' in err and 'Stroke' in err


def test_non_uniform_time(capsys, session, tmp_path):
    path = tmp_path / 'jumpy.csv'
    path.write_text('Time,WP1,Stroke\n0,0,0\n0.005,0,0\n0.02,0,0\n0.025,0,0\n')
    code, _, err, _ = call(capsys, session, record=str(path))
    assert code == 1
    assert 'not uniform' in err


def test_rate_not_a_whole_multiple(capsys, session, tmp_path):
    path = tmp_path / 'odd.csv'
    t = np.arange(100) / 130.0
    path.write_text('Time,WP1,Stroke\n' + ''.join(f'{v:.6f},0,0\n' for v in t))
    code, _, err, _ = call(capsys, session, record=str(path))
    assert code == 1
    assert 'not a whole multiple' in err


def test_record_shorter_than_the_file(capsys, session):
    record = write_record(session['dir'] / 'short.csv', session['x_file'], session['dt'],
                          session['wm'], cut_end_s=30.0 + 60.0 + 100.0)    # inside run 1
    code, _, err, _ = call(capsys, session, record=record, name='short')
    assert code == 1
    assert 'shorter than the file' in err and '--offset-s' in err


def test_record_that_stops_after_the_last_run(capsys, session):
    """As item38.bin: the record stops 14 s after the last run, before the file's 60 s rest."""
    record = write_record(session['dir'] / 'early.csv', session['x_file'], session['dt'],
                          session['wm'], cut_end_s=30.0 + 60.0 - 14.0)
    for stroke in (True, False):
        code, out, err, _ = call(capsys, session, record=record, stroke=stroke,
                                 name=f'early_{stroke}')
        assert code == 0, err
        assert printed_offset(out) == pytest.approx(STILL_BEFORE_S, abs=1.0)
