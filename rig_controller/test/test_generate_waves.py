"""
Tests for src/generate_waves.py, without a controller.

Covers the wavemaker from the yaml, the wave physics against the controller's reference values,
the test matrix, regular and irregular signals, the checks, and the round trip through play_dat.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_generate_waves.py -v
"""

import csv
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

SRC = pathlib.Path(__file__).resolve().parents[1] / 'src'
CONFIG = str(pathlib.Path(__file__).resolve().parents[2]
             / 'wavemaker_bringup' / 'config' / 'wavemakers.yaml')
sys.path.insert(0, str(SRC))  # generate_waves imports play_dat from next to it
_spec = importlib.util.spec_from_file_location('generate_waves', SRC / 'generate_waves.py')
gw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gw)

DT_S = 0.02
HEADER = ['kind', 'period_s', 'height_m', 'steepness', 'n_periods', 'hs_m', 'tp_s', 'gamma',
          'duration_s', 'seed', 'n_ramp', 'pause_s']


# ---------------------------------------------------------------- helpers

@pytest.fixture
def ladertanken():
    return gw.load_wavemaker('ladertanken', CONFIG)


def write_yaml(path, name, params):
    lines = [f'/wavemakers/{name}/controller:', '  ros__parameters:']
    lines += [f'    {key}: {value!r}' if isinstance(value, str) else f'    {key}: {value}'
              for key, value in params.items()]
    path.write_text('\n'.join(lines) + '\n')
    return str(path)


PISTON = {'drive_name': 'indradrive', 'wavemaker_type': 'piston', 'water_depth': 1.0,
          'drive_motion': 'linear',
          'travel_minimum_m': -0.2, 'travel_maximum_m': 0.2}


def write_matrix(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=HEADER, restval='')
        writer.writeheader()
        writer.writerows(rows)
    return str(path)


def regular(**overrides):
    spec = gw.RunSpec(kind='regular', n_ramp=3, pause_s=5.0, period_s=1.0, height_m=0.05,
                      n_periods=20)
    for key, value in overrides.items():
        setattr(spec, key, value)
    return spec


def irregular(**overrides):
    spec = gw.RunSpec(kind='irregular', n_ramp=5, pause_s=5.0, hs_m=0.03, tp_s=1.2, gamma=3.3,
                      duration_s=120.0, seed=1)
    for key, value in overrides.items():
        setattr(spec, key, value)
    return spec


def newton_kh(period_s, depth):
    """Exact solution of omega^2 = g k tanh(kh), the reference for the approximation."""
    omega = 2 * math.pi / period_s
    k = omega ** 2 / gw.G
    for _ in range(50):
        f = gw.G * k * math.tanh(k * depth) - omega ** 2
        k -= f / (gw.G * (math.tanh(k * depth) + k * depth / math.cosh(k * depth) ** 2))
    return k * depth


# ---------------------------------------------------------------- wavemaker from the yaml

def test_ladertanken_limits(ladertanken):
    assert ladertanken.min_m == pytest.approx(-0.14575)
    assert ladertanken.max_m == pytest.approx(0.14575)
    assert ladertanken.max_velocity_mps == pytest.approx(0.7632)
    assert ladertanken.max_acceleration_mps2 is None


def test_all_configured_wavemakers_load():
    assert gw.load_wavemaker('lilletanken', CONFIG).type == 'piston'
    assert gw.load_wavemaker('mclab', CONFIG).type == 'flap'


def test_unknown_wavemaker_lists_names():
    with pytest.raises(ValueError, match='ladertanken'):
        gw.load_wavemaker('nosuchtank', CONFIG)


@pytest.mark.parametrize('key', ['drive_name', 'wavemaker_type', 'water_depth',
                                 'travel_minimum_m', 'travel_maximum_m'])
def test_missing_required_key_is_named(tmp_path, key):
    params = {k: v for k, v in PISTON.items() if k != key}
    with pytest.raises(ValueError, match=key):
        gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', params))


def test_flap_needs_attachment_height(tmp_path):
    params = dict(PISTON, wavemaker_type='flap')
    with pytest.raises(ValueError, match='flap_attachment_height'):
        gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', params))


def test_linear_drive_has_no_speed_limit(tmp_path):
    wm = gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', PISTON))
    assert wm.max_velocity_mps is None
    assert (wm.min_m, wm.max_m) == (-0.2, 0.2)


def test_acceleration_limit_read_when_set(tmp_path):
    params = dict(PISTON, max_goal_acceleration_mps2=5.0)
    wm = gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', params))
    assert wm.max_acceleration_mps2 == 5.0


# ---------------------------------------------------------------- gain and physics

@pytest.mark.parametrize('period_s, gain', [(1.05, 0.755), (0.5, 0.64), (2.5, 0.76), (1.0, 0.80)])
def test_gain_table(ladertanken, period_s, gain):
    assert gw.transfer_gain(ladertanken, period_s) == pytest.approx(gain)


def test_constant_gain_without_table(tmp_path):
    params = dict(PISTON, wavemaker_transfer_gain=0.9)
    wm = gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', params))
    assert gw.transfer_gain(wm, 1.3) == 0.9


@pytest.mark.parametrize('period_s, kh', [(1.0, 4.0262), (1.5, 1.8753)])
def test_dispersion_reference_values(period_s, kh):
    assert gw.wavenumber_depth(period_s, 1.0) == pytest.approx(kh, abs=1e-4)


@pytest.mark.parametrize('period_s', [0.6, 1.0, 1.5, 2.0, 3.0])
def test_dispersion_close_to_exact(period_s):
    assert gw.wavenumber_depth(period_s, 1.0) == pytest.approx(newton_kh(period_s, 1.0), rel=1e-3)


def test_stroke_ratio_reference_values(tmp_path):
    piston = gw.load_wavemaker('test', write_yaml(tmp_path / 'w.yaml', 'test', PISTON))
    flap = gw.Wavemaker(**{**piston.__dict__, 'type': 'flap', 'hinge_height': 0.0})
    kh = gw.wavenumber_depth(1.0, 1.0)
    assert gw.stroke_ratio(kh, piston) == pytest.approx(1.9885, abs=1e-4)
    assert gw.stroke_ratio(kh, flap) == pytest.approx(1.5120, abs=1e-4)


@pytest.mark.parametrize('period_s, height_m, amplitude_m',
                         [(1.0, 0.05, 0.028342), (1.5, 0.10, 0.095470), (1.3, 0.171, 0.131552)])
def test_paddle_amplitude_matches_controller(ladertanken, period_s, height_m, amplitude_m):
    assert gw.paddle_amplitude(ladertanken, period_s, height_m) == pytest.approx(
        amplitude_m, rel=1e-3)


def test_yaml_comment_check_248_deg_200_rpm(ladertanken):
    """wavemakers.yaml, 2026-10-20: 0.171 m at 1.3 s needs +-248 deg and 200 rpm."""
    amplitude = gw.paddle_amplitude(ladertanken, 1.3, 0.171)
    lead = 0.00053
    assert amplitude / lead == pytest.approx(248, abs=1)
    assert amplitude * 2 * math.pi / 1.3 / lead / 6 == pytest.approx(200, abs=1)


# ---------------------------------------------------------------- test matrix

def test_parse_steepness_accepts_1_over_x():
    assert gw.parse_steepness('1/60', 0) == 60.0
    assert gw.parse_steepness(' 1/ 20 ', 0) == 20.0


@pytest.mark.parametrize('text', ['60', '0.0167', '1/0.5', '2/60', '1/', 'x/60', '1/abc'])
def test_parse_steepness_refuses_other_forms(text):
    with pytest.raises(ValueError, match='1/x'):
        gw.parse_steepness(text, 3)


def test_read_matrix_both_kinds_in_file_order(tmp_path):
    path = write_matrix(tmp_path / 'm.csv', [
        {'kind': 'irregular', 'hs_m': 0.05, 'tp_s': 1.2, 'gamma': 2.0, 'duration_s': 600,
         'seed': 7, 'n_ramp': 5, 'pause_s': 120},
        {'kind': 'regular', 'period_s': 1.0, 'steepness': '1/60', 'n_periods': 30, 'n_ramp': 3,
         'pause_s': 60},
        {'period_s': 1.3, 'height_m': 0.1, 'n_periods': 30, 'n_ramp': 3, 'pause_s': 60},
    ])
    specs = gw.read_test_matrix(path)
    assert [s.kind for s in specs] == ['irregular', 'regular', 'regular']
    assert (specs[0].hs_m, specs[0].gamma, specs[0].seed) == (0.05, 2.0, 7)
    assert (specs[1].steepness, specs[1].height_m) == (60.0, None)
    assert (specs[2].height_m, specs[2].steepness) == (0.1, None)


def test_gamma_defaults_to_3_3(tmp_path):
    path = write_matrix(tmp_path / 'm.csv', [
        {'kind': 'irregular', 'hs_m': 0.05, 'tp_s': 1.2, 'duration_s': 600, 'seed': 1,
         'n_ramp': 5, 'pause_s': 60}])
    assert gw.read_test_matrix(path)[0].gamma == 3.3


@pytest.mark.parametrize('row, message', [
    ({'period_s': 1.0, 'height_m': 0.05, 'steepness': '1/60', 'n_periods': 30}, 'not both'),
    ({'period_s': 1.0, 'n_periods': 30}, 'neither'),
    ({'height_m': 0.05, 'n_periods': 30}, 'period_s is required'),
    ({'period_s': 'one', 'height_m': 0.05, 'n_periods': 30}, "period_s 'one' is not a number"),
    ({'kind': 'chop', 'period_s': 1.0}, "kind 'chop'"),
    ({'kind': 'irregular', 'tp_s': 1.2, 'duration_s': 600, 'seed': 1}, 'hs_m is required'),
])
def test_read_matrix_errors_name_row_and_column(tmp_path, row, message):
    path = write_matrix(tmp_path / 'm.csv', [dict(row, n_ramp=3, pause_s=60)])
    with pytest.raises(ValueError, match=message):
        gw.read_test_matrix(path)


def test_target_height_from_steepness(ladertanken):
    spec = regular(height_m=None, steepness=60.0)
    lam = gw.wavelength(ladertanken, 1.0)
    assert lam == pytest.approx(2 * math.pi / gw.wavenumber_depth(1.0, 1.0))
    assert gw.target_height(ladertanken, spec) == pytest.approx(lam / 60)


# ---------------------------------------------------------------- ramps and regular signals

def test_apply_ramps():
    x = np.ones(100)
    ramped = gw.apply_ramps(x, 10)
    assert ramped[0] == 0.0 and ramped[-1] == 0.0
    assert np.all(ramped[10:90] == 1.0)
    assert np.all(x == 1.0)  # input untouched


@pytest.mark.parametrize('n_ramp', [1, 51])
def test_apply_ramps_refuses_bad_length(n_ramp):
    with pytest.raises(ValueError, match='ramp samples'):
        gw.apply_ramps(np.ones(100), n_ramp)


def test_regular_signal(ladertanken):
    x, row = gw.regular_signal(ladertanken, regular(), DT_S)
    assert len(x) == round(20 * 1.0 / DT_S)
    assert x[-1] == 0.0
    # Sampled at dt, the crest is missed by up to half a step: peak >= a cos(pi dt / T).
    assert row['amplitude_m'] * math.cos(math.pi * DT_S / 1.0) <= np.abs(x).max()
    assert np.abs(x).max() <= row['amplitude_m'] * (1 + 1e-4)
    assert row['kind'] == 'regular' and row['steepness'].startswith('1/')


def test_regular_row_records_steepness_for_height_runs(ladertanken):
    _, row = gw.regular_signal(ladertanken, regular(period_s=1.3, height_m=0.171), DT_S)
    assert row['steepness'] == '1/15.2'


# ---------------------------------------------------------------- irregular signals

def test_frequency_grid():
    n, f, df, n_samples = gw.frequency_grid(100.0, DT_S, 0.4, 2.5)
    assert n_samples == 5000
    assert df == pytest.approx(0.01)
    assert f[0] >= 0.4 and f[-1] <= 2.5
    assert np.allclose(f, n * df)


def test_frequency_grid_stops_below_nyquist():
    _, f, _, _ = gw.frequency_grid(10.0, 0.1, 0.1, 100.0)
    assert f[-1] < 0.5 / 0.1


def test_frequency_grid_refuses_partial_samples():
    with pytest.raises(ValueError, match='whole number'):
        gw.frequency_grid(10.005, DT_S, 0.4, 2.5)


def test_frequency_grid_refuses_empty_band():
    with pytest.raises(ValueError, match='no frequencies'):
        gw.frequency_grid(10.0, DT_S, 30.0, 40.0)


@pytest.mark.parametrize('gamma', [1.0, 3.3, 7.0])
def test_generated_hs_is_exact(gamma):
    _, f, df, _ = gw.frequency_grid(600.0, DT_S, 0.4, 2.5)
    a = gw.wave_amplitudes(gw.jonswap(f, df, 0.05, 1.2, gamma), df)
    assert 4 * math.sqrt(np.sum(a ** 2) / 2) == pytest.approx(0.05, rel=1e-12)


def test_jonswap_peak_at_fp():
    _, f, df, _ = gw.frequency_grid(600.0, DT_S, 0.4, 2.5)
    spectrum = gw.jonswap(f, df, 0.05, 1.2, 3.3)
    assert f[spectrum.argmax()] == pytest.approx(1 / 1.2, abs=df)


def test_paddle_amplitudes_are_regular_wave_amplitudes(ladertanken):
    f = np.array([0.6, 0.8, 1.0])
    a = np.array([0.01, 0.02, 0.005])
    p = gw.paddle_amplitudes(ladertanken, f, a)
    for fn, an, pn in zip(f, a, p):
        assert pn == gw.paddle_amplitude(ladertanken, 1 / fn, 2 * an)


def test_synthesize_matches_direct_sum():
    n, f, _, n_samples = gw.frequency_grid(20.0, DT_S, 0.4, 2.5)
    p = np.linspace(0.01, 0.02, len(f))
    phases = gw.random_phases(3, len(f))
    assert np.allclose(gw.synthesize(n, p, phases, n_samples),
                       gw.synthesize_direct(f, p, phases, n_samples, DT_S), atol=1e-12)


def test_synthesized_variance_matches_amplitudes():
    n, f, _, n_samples = gw.frequency_grid(200.0, DT_S, 0.4, 2.5)
    p = np.full(len(f), 0.001)
    x = gw.synthesize(n, p, gw.random_phases(1, len(f)), n_samples)
    assert x.std() == pytest.approx(math.sqrt(np.sum(p ** 2) / 2), rel=1e-9)


def test_same_seed_same_run_other_seed_other_run(ladertanken):
    x1, _ = gw.irregular_signal(ladertanken, irregular(seed=1), DT_S)
    x2, _ = gw.irregular_signal(ladertanken, irregular(seed=1), DT_S)
    x3, _ = gw.irregular_signal(ladertanken, irregular(seed=2), DT_S)
    assert np.array_equal(x1, x2)
    assert not np.allclose(x1, x3)


def test_irregular_signal(ladertanken):
    x, row = gw.irregular_signal(ladertanken, irregular(), DT_S)
    assert len(x) == round(120.0 / DT_S)
    assert x[0] == 0.0 and x[-1] == 0.0
    assert row['kind'] == 'irregular' and row['peak_period_s'] == 1.2
    assert row['f_min_hz'] >= 0.5 / 1.2 and row['f_max_hz'] <= 3.0 / 1.2
    assert row['paddle_peak_m'] == pytest.approx(np.abs(x).max(), abs=1e-5)


# ---------------------------------------------------------------- checks

def test_check_run_accepts_a_small_run(ladertanken):
    x, _ = gw.regular_signal(ladertanken, regular(), DT_S)
    assert gw.check_run(ladertanken, x, DT_S, ladertanken.max_velocity_mps) == []


def test_check_run_reports_travel_and_speed(ladertanken):
    x, _ = gw.regular_signal(ladertanken, regular(period_s=2.0, height_m=0.3), DT_S)
    issues = gw.check_run(ladertanken, x, DT_S, ladertanken.max_velocity_mps)
    assert any(i.startswith('travel') for i in issues)
    assert any(i.startswith('peak speed') for i in issues)


def test_check_run_reports_acceleration_only_when_limited(ladertanken):
    x, _ = gw.regular_signal(ladertanken, regular(), DT_S)
    ladertanken.max_acceleration_mps2 = 0.1
    assert any(i.startswith('peak acceleration')
               for i in gw.check_run(ladertanken, x, DT_S, ladertanken.max_velocity_mps))


def test_check_run_reports_start_away_from_upright(ladertanken):
    x = np.full(100, 0.01)
    x[-1] = 0.0
    issues = gw.check_run(ladertanken, x, DT_S, ladertanken.max_velocity_mps)
    assert any(i.startswith('starts') for i in issues)


# ---------------------------------------------------------------- assemble, write, round trip

def test_assemble_skips_without_a_gap(ladertanken):
    specs = [regular(), regular(period_s=2.0, height_m=0.3), regular(period_s=1.5, height_m=0.1)]
    frames, rows = gw.assemble(ladertanken, specs, DT_S, ladertanken.max_velocity_mps)
    assert [r['status'] for r in rows] == ['', 'skipped', '']
    assert rows[2]['start_s'] == pytest.approx(rows[0]['end_s'] + specs[0].pause_s)
    assert frames.shape[1] == 2
    assert frames[0, 0] == 0.0 and frames[-1, 0] == 0.0


def test_write_motion_header_and_layout(tmp_path):
    frames = np.arange(12, dtype=float).reshape(6, 2)
    path = tmp_path / 'm.dat'
    gw.write_motion(str(path), frames, DT_S)
    header = np.fromfile(path, dtype='<i4', count=2)
    body = np.fromfile(path, dtype='<f4', offset=8).reshape(6, 2)
    assert list(header) == [20, 2]
    assert np.array_equal(body, frames)


def test_write_motion_refuses_partial_milliseconds(tmp_path):
    with pytest.raises(ValueError, match='milliseconds'):
        gw.write_motion(str(tmp_path / 'm.dat'), np.zeros((4, 2)), 0.0125)


def test_round_trip_through_play_dat(ladertanken, tmp_path):
    specs = [regular(), regular(height_m=None, steepness=30.0), irregular(),
             regular(period_s=2.0, height_m=0.3)]
    frames, rows = gw.assemble(ladertanken, specs, DT_S, ladertanken.max_velocity_mps)
    dat = str(tmp_path / 'run.dat')
    gw.write_motion(dat, frames, DT_S)
    gw.write_run_table(str(tmp_path / 'run.csv'), rows)
    assert gw.verify_with_play_dat(dat, ladertanken, ladertanken.max_velocity_mps) == 3


def test_main_end_to_end(tmp_path, capsys):
    matrix = write_matrix(tmp_path / 'm.csv', [
        {'kind': 'regular', 'period_s': 1.0, 'steepness': '1/60', 'n_periods': 20, 'n_ramp': 3,
         'pause_s': 10},
        {'kind': 'irregular', 'hs_m': 0.03, 'tp_s': 1.2, 'duration_s': 120, 'seed': 1,
         'n_ramp': 5, 'pause_s': 10},
    ])
    out = str(tmp_path / 'session')
    assert gw.main(['ladertanken', matrix, '-o', out, '--config', CONFIG]) == 0
    printed = capsys.readouterr().out
    assert '2 runs, 0 skipped' in printed
    # the short play_dat form (item 40): limits from the yaml by name
    assert f'play_dat {out}.dat ladertanken --dry-run' in printed
    with open(out + '.csv', newline='') as f:
        table = list(csv.DictReader(f))
    assert [r['kind'] for r in table] == ['regular', 'irregular']
    assert table[1]['peak_period_s'] == '1.2' and table[0]['period_s'] == '1.0'
