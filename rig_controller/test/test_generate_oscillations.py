"""
Tests for src/generate_oscillations.py, without a controller.

Covers the test matrix and its checks, regular and irregular signals with cosine ramps, the jerk
column and warning, the run table and record, and the round trip through play_dat for both
oscillators in oscillator.yaml.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_generate_oscillations.py -v
"""

import csv
import importlib.util
import math
import pathlib
import re
import sys

import numpy as np
import pytest

SRC = pathlib.Path(__file__).resolve().parents[1] / 'src'
CONFIG = str(pathlib.Path(__file__).resolve().parents[2]
             / 'oscillator_bringup' / 'config' / 'oscillator.yaml')
sys.path.insert(0, str(SRC))  # generate_oscillations imports motion_files etc. from next to it
_spec = importlib.util.spec_from_file_location('generate_oscillations',
                                               SRC / 'generate_oscillations.py')
go = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(go)

DT_S = 0.02
HEADER = ['kind', 'period_s', 'stroke_m', 'n_periods', 'sig_stroke_m', 'tp_s', 'gamma',
          'duration_s', 'seed', 'n_ramp', 'pause_s']


# ---------------------------------------------------------------- helpers

@pytest.fixture
def vertical():
    return go.rig_limits.load_limits('oscillator', 'vertical', CONFIG)


def write_matrix(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=HEADER, restval='')
        writer.writeheader()
        writer.writerows(rows)
    return str(path)


def regular(**overrides):
    spec = go.RegularSpec(n_ramp=2, pause_s=5.0, period_s=2.0, stroke_m=0.02, n_periods=10)
    for key, value in overrides.items():
        setattr(spec, key, value)
    return spec


def irregular(**overrides):
    spec = go.IrregularSpec(n_ramp=3, pause_s=10.0, sig_stroke_m=0.01, tp_s=1.5, gamma=3.3,
                            duration_s=120.0, seed=1)
    for key, value in overrides.items():
        setattr(spec, key, value)
    return spec


REGULAR_ROW = {'kind': 'regular', 'period_s': 2.0, 'stroke_m': 0.02, 'n_periods': 10,
               'n_ramp': 2, 'pause_s': 5}
IRREGULAR_ROW = {'kind': 'irregular', 'sig_stroke_m': 0.01, 'tp_s': 1.5, 'gamma': 3.3,
                 'duration_s': 120, 'seed': 1, 'n_ramp': 3, 'pause_s': 10}


def generate(tmp_path, oscillator='vertical', rows=(REGULAR_ROW,), *options):
    matrix = write_matrix(tmp_path / 'm.csv', list(rows))
    out = str(tmp_path / 'session')
    code = go.main([oscillator, matrix, '-o', out, '--config', CONFIG, '--no-plot', *options])
    return code, out


# ---------------------------------------------------------------- the oscillators

def test_vertical_limits(vertical):
    """T8: the speed limit from max_velocity_rpm in the drive's units (600 x 10 mm/min)."""
    assert (vertical.min_m, vertical.max_m, vertical.upright_m) == (-0.2, 0.2, 0.0)
    assert vertical.max_velocity_mps == pytest.approx(0.1)
    assert vertical.namespace == '/oscillators/vertical'


def test_default_config_is_oscillator_yaml(monkeypatch):
    monkeypatch.setattr(go.rig_limits, 'default_config_path', lambda rig: f'{rig}.yaml')
    assert go.default_config_path() == 'oscillator.yaml'


# ---------------------------------------------------------------- test matrix

def test_read_matrix_both_kinds_in_file_order(tmp_path):
    specs = go.read_test_matrix(write_matrix(tmp_path / 'm.csv',
                                             [REGULAR_ROW, IRREGULAR_ROW, REGULAR_ROW]))
    assert [type(s).__name__ for s in specs] == ['RegularSpec', 'IrregularSpec', 'RegularSpec']
    assert specs[0] == regular()
    assert specs[1] == irregular()


def test_kind_defaults_to_regular_and_gamma_to_3_3(tmp_path):
    specs = go.read_test_matrix(write_matrix(tmp_path / 'm.csv', [
        {**REGULAR_ROW, 'kind': ''}, {**IRREGULAR_ROW, 'gamma': ''}]))
    assert isinstance(specs[0], go.RegularSpec)
    assert specs[1].gamma == 3.3


@pytest.mark.parametrize('row, message', [
    ({**REGULAR_ROW, 'kind': 'sine'}, "kind 'sine' must be regular or irregular"),
    ({**REGULAR_ROW, 'stroke_m': ''}, 'stroke_m is required'),
    ({**REGULAR_ROW, 'stroke_m': '2,0'}, "stroke_m '2,0' is not a number"),
    ({**IRREGULAR_ROW, 'sig_stroke_m': ''}, 'sig_stroke_m is required'),
    ({**REGULAR_ROW, 'n_ramp': 0}, 'n_ramp must be at least 1'),
    ({**REGULAR_ROW, 'pause_s': 0.5}, 'pause_s 0.5 is below 1.0 s'),
    ({**REGULAR_ROW, 'stroke_m': -0.02}, 'stroke_m must be > 0, not -0.02'),
    ({**REGULAR_ROW, 'period_s': 0}, 'period_s must be > 0'),
    ({**REGULAR_ROW, 'n_periods': 3}, 'n_periods 3 is shorter than the two ramps (4 periods)'),
    ({**IRREGULAR_ROW, 'sig_stroke_m': 0}, 'sig_stroke_m must be > 0'),
    ({**IRREGULAR_ROW, 'tp_s': -1}, 'tp_s must be > 0'),
    ({**IRREGULAR_ROW, 'gamma': 0}, 'gamma must be > 0'),
    ({**IRREGULAR_ROW, 'duration_s': 5}, 'duration_s 5.0 is shorter than the two ramps (9.0 s)'),
])
def test_matrix_errors_name_row_and_column(tmp_path, row, message):
    """The error names the row (0 = first after the header, as the run number)."""
    path = write_matrix(tmp_path / 'm.csv', [REGULAR_ROW, row])
    with pytest.raises(ValueError, match=r'^row 1: ' + re.escape(message)):
        go.read_test_matrix(path)


def test_matrix_error_is_a_usage_error_without_files(tmp_path, capsys):
    with pytest.raises(SystemExit) as stop:
        generate(tmp_path, 'vertical', [{**REGULAR_ROW, 'n_ramp': 0}])
    assert stop.value.code == 2
    assert 'row 0: n_ramp must be at least 1' in capsys.readouterr().err
    assert not (tmp_path / 'session.dat').exists()


# ---------------------------------------------------------------- signals

def test_regular_signal_amplitude_is_half_the_stroke():
    x, row = go.regular_signal(regular(), DT_S)
    assert len(x) == round(10 * 2.0 / DT_S)
    assert np.abs(x).max() == pytest.approx(0.01, rel=1e-3)
    assert row == {'kind': 'regular', 'period_s': 2.0, 'stroke_m': 0.02, 'amplitude_m': 0.01}


@pytest.mark.parametrize('make, period', [(regular, 2.0), (irregular, 1.5)])
def test_runs_start_and_end_at_rest(make, period):
    """The cosine ramp: first and last sample exactly 0, so play_dat finds the rests."""
    x, _, _ = go.run_signal(make(), DT_S)
    assert x[0] == 0.0 and x[-1] == 0.0


def test_ramps_are_cosine():
    """Inside the first ramp the envelope is (1 - cos)/2, not linear: slow start."""
    spec = regular(n_ramp=2)
    x, _ = go.regular_signal(spec, DT_S)
    n_ramp = round(2 * 2.0 / DT_S)
    t = np.arange(1, len(x) + 1) * DT_S
    sine = 0.01 * np.sin(2 * math.pi / 2.0 * t)
    quarter = n_ramp // 4 + 12          # a sample where the sine is far from 0
    r = (1 - np.cos(np.pi * np.linspace(0, 1, n_ramp))) / 2
    assert x[quarter] == pytest.approx(sine[quarter] * r[quarter])
    assert r[quarter] < quarter / (n_ramp - 1)     # below the linear ramp there


def test_irregular_significant_stroke_is_4_sigma():
    """sig_stroke_m goes into jonswap as Hs: 4 sigma of the position, within the ramps' loss."""
    x, row = go.irregular_signal(irregular(duration_s=600.0), DT_S)
    assert 4 * x.std() == pytest.approx(0.01, rel=0.03)
    assert row['std_m'] == round(float(x.std()), 5)
    assert row['peak_m'] == round(float(np.abs(x).max()), 5)
    assert (row['f_min_hz'], row['f_max_hz']) == pytest.approx((1 / 3, 2.0), abs=0.01)


def test_same_seed_same_run_other_seed_other_run():
    a, _ = go.irregular_signal(irregular(seed=1), DT_S)
    b, _ = go.irregular_signal(irregular(seed=1), DT_S)
    c, _ = go.irregular_signal(irregular(seed=2), DT_S)
    assert np.array_equal(a, b)
    assert not np.allclose(a, c)


# ---------------------------------------------------------------- jerk

def test_peak_jerk_of_a_sine():
    """A sine of amplitude a and period T has peak jerk a (2 pi / T)^3."""
    t = np.arange(0, 10, 0.001)
    x = 0.01 * np.sin(2 * math.pi / 2.0 * t)
    assert go.peak_jerk(x, 0.001) == pytest.approx(0.01 * (math.pi) ** 3, rel=1e-3)


def test_jerk_column_in_every_row():
    for make in (regular, irregular):
        x, row, _ = go.run_signal(make(), DT_S)
        assert row['peak_jerk_mps3'] == round(go.peak_jerk(x, DT_S), 3)


def test_jerk_warning_above_the_limit_only(tmp_path, capsys):
    rows = [REGULAR_ROW, {**REGULAR_ROW, 'period_s': 1.0, 'stroke_m': 0.01}]
    code, _ = generate(tmp_path, 'vertical', rows, '--max-jerk-mps3', '1.0')
    assert code == 0
    printed = capsys.readouterr().out
    assert 'run 0: peak jerk' not in printed               # 0.34 m/s^3
    assert 'run 1: peak jerk 1.333 m/s^3 above 1.0' in printed
    assert '2 runs, 0 skipped' in printed                  # a warning, not a skip


def test_no_jerk_warning_for_a_skipped_run(tmp_path, capsys):
    """A run outside the limits is skipped with its reason; no jerk warning on top."""
    rows = [REGULAR_ROW, {**REGULAR_ROW, 'period_s': 0.5, 'stroke_m': 0.3}]
    code, _ = generate(tmp_path, 'vertical', rows, '--max-jerk-mps3', '1.0')
    assert code == 0
    printed = capsys.readouterr().out
    assert 'run 1 skipped: peak speed' in printed
    assert 'run 1: peak jerk' not in printed


# ---------------------------------------------------------------- assemble and files

def test_assemble_skips_a_run_too_fast(vertical):
    fast = regular(period_s=1.0, stroke_m=0.1)              # peak speed 0.31 m/s > 0.1
    frames, rows = go.assemble(vertical, [regular(), fast], DT_S, vertical.max_velocity_mps)
    assert [r['status'] for r in rows] == ['', 'skipped']
    assert rows[1]['start_s'] == ''
    assert frames[:, 1].sum() == round(10 * 2.0 / DT_S)      # trigger only during run 0


def test_round_trip_through_play_dat(vertical, tmp_path):
    frames, rows = go.assemble(vertical, [regular(), irregular()], DT_S,
                               vertical.max_velocity_mps)
    dat = str(tmp_path / 'run.dat')
    go.motion_files.write_motion(dat, frames, DT_S)
    go.write_run_table(str(tmp_path / 'run.csv'), rows)
    assert go.motion_files.verify_with_play_dat(dat, vertical, vertical.max_velocity_mps) == 2


@pytest.mark.parametrize('oscillator', ['vertical', 'sloshing'])
def test_main_end_to_end(tmp_path, capsys, oscillator):
    code, out = generate(tmp_path, oscillator, [REGULAR_ROW, IRREGULAR_ROW])
    assert code == 0
    printed = capsys.readouterr().out
    assert '2 runs, 0 skipped' in printed
    assert (f'play_dat {out}.dat {oscillator} --rig oscillator --dry-run --config {CONFIG}'
            in printed)
    with open(out + '.csv', newline='') as f:
        reader = csv.DictReader(f)
        table = list(reader)
    assert reader.fieldnames == go.OSCILLATION_COLUMNS
    assert [r['kind'] for r in table] == ['regular', 'irregular']
    assert table[0]['stroke_m'] == '0.02' and table[1]['sig_stroke_m'] == '0.01'
    assert all(r['peak_jerk_mps3'] for r in table)


def test_max_velocity_override_is_recorded_and_printed(tmp_path, capsys):
    code, out = generate(tmp_path, 'vertical', [REGULAR_ROW], '--max-velocity-mps', '0.05')
    assert code == 0
    assert '--max-velocity-mps 0.0500' in capsys.readouterr().out
    assert 'max speed    0.0500 m/s (--max-velocity-mps)' in pathlib.Path(out + '.txt').read_text()


def test_record_next_to_the_file(tmp_path):
    code, out = generate(tmp_path)
    assert code == 0
    record = pathlib.Path(out + '.txt').read_text()
    assert f'command      generate_oscillations vertical {tmp_path}/m.csv -o {out}' in record
    assert 'oscillator   vertical\n' in record
    assert 'max jerk     3.0 m/s^3 (warning only)' in record
    assert 'max speed    0.1000 m/s\n' in record
    assert 'water depth' not in record
    assert record.endswith(f'test matrix {tmp_path}/m.csv:\n' + ','.join(HEADER) + '\n'
                           'regular,2.0,0.02,10,,,,,,2,5\n')


def test_plot_written(tmp_path):
    matrix = write_matrix(tmp_path / 'm.csv', [REGULAR_ROW])
    out = str(tmp_path / 'session')
    assert go.main(['vertical', matrix, '-o', out, '--config', CONFIG]) == 0
    assert pathlib.Path(out + '.png').stat().st_size > 0
