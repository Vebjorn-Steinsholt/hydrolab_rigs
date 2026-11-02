"""
Tests for src/play_dat.py, without a controller.

Covers file reading, run splitting, the run table, measuring, the checks and the command line
(dry run). play() is not tested here; use fake_controller.

    cd ~/ros_ws/src/ros2_wavemakers
    python3 -m pytest rig_controller/test/test_play_dat.py -v

PLAY_DAT=<path> tests another copy of play_dat.py.
"""

import csv
import importlib.util
import os
import pathlib

import numpy as np
import pytest

PLAY_DAT = os.environ.get(
    'PLAY_DAT', str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'play_dat.py'))
_spec = importlib.util.spec_from_file_location('play_dat', PLAY_DAT)
pd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pd)

DT_S = 0.01


# ---------------------------------------------------------------- helpers

def write_motion(path, columns, dt_s=DT_S):
    """Write a motion file: header (step in ms, channel count), interleaved float32."""
    data = np.column_stack(columns).astype('<f4')
    with open(path, 'wb') as f:
        np.array([round(dt_s * 1000), data.shape[1]], dtype='<i4').tofile(f)
        data.tofile(f)
    return path


def regular_run(period_s, amplitude_m, periods=10, ramp_periods=2, dt_s=DT_S):
    """Return a sin^2-ramped sine of whole periods that starts and ends at exactly 0."""
    n = int(round(periods * period_s / dt_s))
    t_s = np.arange(n + 1) * dt_s
    ramp_s = ramp_periods * period_s
    envelope = np.ones_like(t_s)
    up = t_s < ramp_s
    down = t_s > t_s[-1] - ramp_s
    envelope[up] = np.sin(np.pi / 2 * t_s[up] / ramp_s) ** 2
    envelope[down] = np.sin(np.pi / 2 * (t_s[-1] - t_s[down]) / ramp_s) ** 2
    x = amplitude_m * envelope * np.sin(2 * np.pi * t_s / period_s)
    x[0] = x[-1] = 0.0
    return x


def irregular_run(duration_s=60.0, amplitude_m=0.01, seed=1, dt_s=DT_S):
    """Sum of sines with random periods and phases, ramped to 0 at both ends."""
    rng = np.random.default_rng(seed)
    t_s = np.arange(int(round(duration_s / dt_s)) + 1) * dt_s
    x = np.zeros_like(t_s)
    for period_s, phase in zip(rng.uniform(0.6, 2.0, 30), rng.uniform(0, 2 * np.pi, 30)):
        x += np.sin(2 * np.pi * t_s / period_s + phase)
    x *= amplitude_m / np.abs(x).max()
    ramp = np.minimum(1.0, np.minimum(t_s, t_s[-1] - t_s) / 5.0)
    x *= np.sin(np.pi / 2 * ramp) ** 2
    x[0] = x[-1] = 0.0
    return x


def join(parts):
    """parts: list of rest lengths in s (float) and runs (arrays) -> one signal."""
    out = []
    for part in parts:
        out.append(np.zeros(int(round(part / DT_S))) if np.isscalar(part) else part)
    return np.concatenate(out)


def three_runs():
    """5 s lead-in, three regular runs with gaps of 2 s and 4 s, 1 s at the end."""
    runs = [regular_run(1.0, 0.05), regular_run(0.8, 0.03), regular_run(1.2, 0.04)]
    return join([5.0, runs[0], 2.0, runs[1], 4.0, runs[2], 1.0]), runs


def write_table(path, rows):
    fields = ['run', 'start_s', 'end_s', 'kind', 'period_s', 'height_m', 'status']
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def table_rows():
    """Run table for three_runs(), with boundaries as the generator writes them."""
    _, runs = three_runs()
    rows, t_s = [], 5.0
    for index, (run, period_s, gap_s) in enumerate(zip(runs, (1.0, 0.8, 1.2), (2.0, 4.0, 1.0))):
        end_s = t_s + len(run) * DT_S
        rows.append({'run': index, 'start_s': f'{t_s:.2f}', 'end_s': f'{end_s:.2f}',
                     'kind': 'regular', 'period_s': period_s, 'height_m': 0.1, 'status': 'ok'})
        t_s = end_s + gap_s
    return rows


# ---------------------------------------------------------------- read_motion

def test_read_motion_round_trip(tmp_path):
    position = np.linspace(-0.1, 0.1, 50)
    trigger = (position > 0).astype(float)
    motion = pd.read_motion(write_motion(tmp_path / 'a.dat', [position, trigger]))
    assert motion.sample_interval_s == pytest.approx(DT_S)
    assert motion.channels == 2
    assert motion.data.shape == (50, 2)
    np.testing.assert_allclose(motion.data[:, 0], position, atol=1e-7)


def test_read_motion_too_small(tmp_path):
    path = tmp_path / 'a.dat'
    path.write_bytes(b'\x01\x00\x00')
    with pytest.raises(ValueError, match='too small'):
        pd.read_motion(path)


@pytest.mark.parametrize('header', [(0, 2), (-10, 2), (10, 0), (10, -1)])
def test_read_motion_bad_header(tmp_path, header):
    path = tmp_path / 'a.dat'
    with open(path, 'wb') as f:
        np.array(header, dtype='<i4').tofile(f)
        np.zeros(4, dtype='<f4').tofile(f)
    with pytest.raises(ValueError, match='must be positive'):
        pd.read_motion(path)


def test_read_motion_size_does_not_fit_channels(tmp_path):
    path = tmp_path / 'a.dat'
    with open(path, 'wb') as f:
        np.array([10, 2], dtype='<i4').tofile(f)
        np.zeros(5, dtype='<f4').tofile(f)  # 5 values, 2 channels
    with pytest.raises(ValueError, match='do not fit'):
        pd.read_motion(path)


def test_read_motion_header_only(tmp_path):
    path = tmp_path / 'a.dat'
    np.array([10, 2], dtype='<i4').tofile(path)
    with pytest.raises(ValueError, match='do not fit'):
        pd.read_motion(path)


def test_read_motion_non_finite(tmp_path):
    position = np.zeros(10)
    position[7] = np.nan
    path = write_motion(tmp_path / 'a.dat', [position, np.zeros(10)])
    with pytest.raises(ValueError, match='step 7, channel 0'):
        pd.read_motion(path)


# ---------------------------------------------------------------- select_channel

def motion_file(*columns):
    return pd.MotionFile(DT_S, len(columns), np.column_stack(columns).astype('<f4'))


def test_select_channel_by_name_and_index():
    motion = motion_file(np.full(4, 0.1), np.full(4, 1.0))
    np.testing.assert_allclose(pd.select_channel(motion, 'position'), 0.1, atol=1e-7)
    np.testing.assert_allclose(pd.select_channel(motion, 'trigger'), 1.0)
    np.testing.assert_allclose(pd.select_channel(motion, '1'), 1.0)
    assert pd.select_channel(motion, '0').dtype == np.float64


def test_select_channel_errors():
    motion = motion_file(np.full(4, 0.1), np.zeros(4))
    with pytest.raises(ValueError, match='unknown channel'):
        pd.select_channel(motion, 'X')
    with pytest.raises(ValueError, match='does not exist'):
        pd.select_channel(motion, '2')
    with pytest.raises(ValueError, match='all zeros'):
        pd.select_channel(motion, 'trigger')


# ---------------------------------------------------------------- rest_mask, stretches

def test_stretches():
    mask = np.array([0, 0, 1, 1, 1, 0, 1], dtype=bool)
    assert [(int(a), int(b)) for a, b in pd.stretches(mask)] == [(2, 5), (6, 7)]
    assert pd.stretches(np.zeros(5, dtype=bool)) == []
    assert [(int(a), int(b)) for a, b in pd.stretches(np.ones(3, dtype=bool))] == [(0, 3)]


def test_rest_mask_zeros_are_rest_motion_is_not():
    x = join([1.0, regular_run(1.0, 0.05), 1.0])
    rest = pd.rest_mask(x, DT_S, 0.0)
    assert rest[:90].all()
    assert rest[-90:].all()
    moving = rest[100:-100]
    assert not moving.any(), 'a sample exactly at a zero crossing must not count as rest'


# ---------------------------------------------------------------- split_runs

def test_split_runs_finds_runs_and_gaps():
    x, parts = three_runs()
    runs = pd.split_runs(x, DT_S, upright_m=0.0)
    assert [r.index for r in runs] == [0, 1, 2]
    for run, part in zip(runs, parts):
        assert abs(len(run.values_m) - len(part)) <= 3
    gaps = [r.gap_before_s for r in runs]
    np.testing.assert_allclose(gaps, [5.0, 2.0, 4.0], atol=3 * DT_S)


def test_split_runs_every_run_starts_and_ends_at_rest():
    x, _ = three_runs()
    for run in pd.split_runs(x, DT_S, upright_m=0.0):
        assert run.values_m[0] == 0.0
        assert run.values_m[-1] == 0.0


def test_split_runs_positions_are_upright_plus_values():
    x, _ = three_runs()
    for run in pd.split_runs(x, DT_S, upright_m=0.25):
        np.testing.assert_allclose(run.positions_m, 0.25 + run.values_m)


def test_split_runs_runs_cover_all_motion():
    x, _ = three_runs()
    covered = np.zeros(len(x), dtype=bool)
    for run in pd.split_runs(x, DT_S, upright_m=0.0):
        covered[run.start_sample:run.start_sample + len(run.values_m)] = True
    assert not (x[~covered] != 0).any()


def test_split_runs_short_pause_stays_in_run():
    run = regular_run(1.0, 0.05)
    x = join([2.0, run, 0.3, run, 2.0])
    runs = pd.split_runs(x, DT_S, upright_m=0.0)
    assert len(runs) == 1
    assert len(runs[0].short_pauses) == 1
    start_s, length_s = runs[0].short_pauses[0]
    assert length_s == pytest.approx(0.3, abs=3 * DT_S)
    assert start_s == pytest.approx(len(run) * DT_S, abs=3 * DT_S)


def test_split_runs_min_gap():
    run = regular_run(1.0, 0.05)
    x = join([2.0, run, 0.5, run, 2.0])
    assert len(pd.split_runs(x, DT_S, 0.0, min_gap_s=1.0)) == 1
    assert len(pd.split_runs(x, DT_S, 0.0, min_gap_s=0.4)) == 2


def test_split_runs_motion_at_file_start_and_end():
    x = regular_run(1.0, 0.05)[:-50]  # starts moving at sample 1, still moving at the end
    runs = pd.split_runs(x, DT_S, upright_m=0.0)
    assert len(runs) == 1
    assert runs[0].start_sample == 0
    assert runs[0].gap_before_s == 0.0
    assert len(runs[0].values_m) == len(x)


def test_split_runs_short_rest_at_file_ends_is_not_played():
    run = regular_run(1.0, 0.05)
    x = join([0.3, run, 0.5])  # both shorter than min_gap_s
    runs = pd.split_runs(x, DT_S, upright_m=0.0)
    assert len(runs) == 1
    assert runs[0].short_pauses == []
    assert len(runs[0].values_m) <= len(run) + 3
    assert runs[0].gap_before_s == pytest.approx(0.3, abs=3 * DT_S)


def test_split_runs_irregular_is_not_cut():
    x = join([2.0, irregular_run(), 2.0])
    assert len(pd.split_runs(x, DT_S, upright_m=0.0)) == 1


def test_split_runs_gap_threshold():
    run = regular_run(1.0, 0.05)
    noise = np.full(200, 1e-5)  # a near-zero offset, not exact zeros
    x = np.concatenate([np.zeros(100), run, noise, run, np.zeros(100)])
    assert len(pd.split_runs(x, DT_S, 0.0, gap_threshold_m=0.0)) == 1
    assert len(pd.split_runs(x, DT_S, 0.0, gap_threshold_m=1e-4)) == 2


def test_whole_file_run():
    x, _ = three_runs()
    runs = pd.whole_file_run(x, upright_m=0.1)
    assert len(runs) == 1
    assert runs[0].start_sample == 0
    np.testing.assert_allclose(runs[0].positions_m, 0.1 + x)


# ---------------------------------------------------------------- run table

def test_read_run_table_missing_file(tmp_path):
    assert pd.read_run_table(str(tmp_path / 'a.dat')) is None


def test_read_run_table_drops_skipped_rows(tmp_path):
    rows = table_rows()
    rows[1]['status'] = 'skipped'
    write_table(tmp_path / 'a.csv', rows)
    table = pd.read_run_table(str(tmp_path / 'a.dat'))
    assert [row['run'] for row in table] == ['0', '2']


def test_read_run_table_missing_column(tmp_path):
    (tmp_path / 'a.csv').write_text('run,start_s\n0,5.0\n')
    with pytest.raises(ValueError, match='end_s'):
        pd.read_run_table(str(tmp_path / 'a.dat'))


def test_apply_run_table_names_runs(tmp_path):
    x, _ = three_runs()
    write_table(tmp_path / 'a.csv', table_rows())
    table = pd.read_run_table(str(tmp_path / 'a.dat'))
    runs = pd.apply_run_table(pd.split_runs(x, DT_S, 0.0), table, DT_S)
    assert [r.kind for r in runs] == ['regular'] * 3
    assert [r.info['period_s'] for r in runs] == ['1.0', '0.8', '1.2']


def test_apply_run_table_count_mismatch():
    x, _ = three_runs()
    with pytest.raises(ValueError, match='lists 2 runs'):
        pd.apply_run_table(pd.split_runs(x, DT_S, 0.0), table_rows()[:2], DT_S)


@pytest.mark.parametrize('column', ['start_s', 'end_s'])
def test_apply_run_table_boundary_mismatch(column):
    x, _ = three_runs()
    rows = table_rows()
    rows[1][column] = f'{float(rows[1][column]) + 0.5:.2f}'
    rows = [{k: str(v) for k, v in row.items()} for row in rows]
    with pytest.raises(ValueError, match='run 1'):
        pd.apply_run_table(pd.split_runs(x, DT_S, 0.0), rows, DT_S)


# ---------------------------------------------------------------- measuring

def make_run(x, kind='unknown', upright_m=0.0, info=None):
    return pd.Run(0, 0, x, upright_m + x, 0.0, kind=kind, info=info or {})


def test_upward_zero_crossings_interpolated():
    t_s = np.arange(0, 3, DT_S)
    x = np.sin(2 * np.pi * (t_s - 0.123))  # crossings at 0.123 + n
    np.testing.assert_allclose(pd.upward_zero_crossings_s(x, DT_S), [0.123, 1.123, 2.123],
                               atol=1e-3)


def test_classify_regular_and_irregular():
    assert pd.classify_run(make_run(regular_run(1.0, 0.05, periods=20)), DT_S) == 'regular'
    assert pd.classify_run(make_run(irregular_run()), DT_S) == 'irregular'
    assert pd.classify_run(make_run(np.zeros(500)), DT_S) == 'unknown'


def test_describe_regular_run():
    d = pd.describe_run(make_run(regular_run(1.25, 0.05, periods=20), kind='regular'), DT_S)
    assert d['period_s'] == pytest.approx(1.25, rel=1e-3)
    assert d['amplitude_m'] == pytest.approx(0.05, rel=0.01)
    assert d['max_abs_m'] == pytest.approx(0.05, rel=0.01)
    assert d['peak_speed_mps'] == pytest.approx(2 * np.pi * 0.05 / 1.25, rel=0.02)
    assert d['duration_s'] == pytest.approx(len(regular_run(1.25, 0.05, periods=20)) * DT_S)
    assert d['source'] == 'measured'


def test_describe_irregular_run_has_peak_period_and_excursion():
    d = pd.describe_run(make_run(irregular_run(), kind='irregular'), DT_S)
    assert 0.6 <= d['period_s'] <= 2.0
    assert d['excursion_m'] > 0


def test_describe_table_period_wins():
    run = make_run(regular_run(1.0, 0.05), kind='regular', info={'period_s': '1.5'})
    d = pd.describe_run(run, DT_S)
    assert d['period_s'] == 1.5
    assert d['source'] == 'table'


# ---------------------------------------------------------------- check_run

LIMITS = {'upright_m': 0.0, 'minimum_m': -0.1, 'maximum_m': 0.1, 'max_velocity_mps': 0.5}


def test_check_run_ok():
    assert pd.check_run(make_run(regular_run(1.0, 0.05)), DT_S, **LIMITS) == []


def test_check_run_outside_travel():
    issues = pd.check_run(make_run(regular_run(2.0, 0.15)), DT_S, **LIMITS)
    assert any('travel' in issue for issue in issues)


def test_check_run_old_mm_file_fails_travel():
    issues = pd.check_run(make_run(regular_run(1.0, 0.05) * 1000), DT_S, **LIMITS)
    assert any('travel' in issue for issue in issues)


def test_check_run_too_fast():
    issues = pd.check_run(make_run(regular_run(0.5, 0.08)), DT_S, **LIMITS)  # ~1.0 m/s
    assert any('speed' in issue for issue in issues)


def test_check_run_not_at_upright():
    x = regular_run(1.0, 0.05)
    x[0] = 0.01
    issues = pd.check_run(make_run(x), DT_S, **LIMITS)
    assert any(issue.startswith('starts') for issue in issues)


def test_check_run_upright_offset():
    run = make_run(regular_run(1.0, 0.05), upright_m=0.3)
    assert pd.check_run(run, DT_S, upright_m=0.3, minimum_m=0.2, maximum_m=0.4,
                        max_velocity_mps=0.5) == []


# ---------------------------------------------------------------- parse_runs

def test_parse_runs():
    assert pd.parse_runs('', 4) == [0, 1, 2, 3]
    assert pd.parse_runs('2', 4) == [2]
    assert pd.parse_runs('1-3', 4) == [1, 2, 3]
    assert pd.parse_runs('0,3', 4) == [0, 3]


@pytest.mark.parametrize('text', ['4', '3-4', '0,9'])
def test_parse_runs_out_of_range(text):
    with pytest.raises(ValueError):
        pd.parse_runs(text, 4)


# ---------------------------------------------------------------- main (dry run only)

def dry_run(tmp_path, *extra, table=True, limits=('--min-m', '-0.1', '--max-m', '0.1',
                                                  '--max-velocity-mps', '0.54')):
    x, _ = three_runs()
    path = write_motion(tmp_path / 'a.dat', [x, (x != 0).astype(float)])
    if table:
        write_table(tmp_path / 'a.csv', table_rows())
    return pd.main(['play_dat', str(path), *limits, '--dry-run', *extra])


def test_main_dry_run_ok(tmp_path, capsys):
    assert dry_run(tmp_path) == 0
    out = capsys.readouterr().out
    assert out.count(' OK') == 3
    assert 'regular' in out


def test_main_dry_run_without_table_measures(tmp_path, capsys):
    assert dry_run(tmp_path, table=False) == 0
    out = capsys.readouterr().out
    assert 'no run table' in out
    assert out.count('regular') == 3


def test_main_selected_runs(tmp_path, capsys):
    assert dry_run(tmp_path, '--runs', '1') == 0
    assert capsys.readouterr().out.count(' OK') == 1


def test_main_problems_stop_before_ros(tmp_path, capsys):
    tight = ('--min-m', '-0.01', '--max-m', '0.01', '--max-velocity-mps', '0.54')
    x, _ = three_runs()
    path = write_motion(tmp_path / 'a.dat', [x])
    assert pd.main(['play_dat', str(path), *tight]) == 1  # no --dry-run, but nothing is sent
    assert 'travel' in capsys.readouterr().out


def test_main_bad_file(tmp_path, capsys):
    path = tmp_path / 'a.dat'
    path.write_bytes(b'abc')
    assert pd.main(['play_dat', str(path), '--min-m', '-0.1', '--max-m', '0.1',
                    '--max-velocity-mps', '0.5']) == 2
    assert 'error' in capsys.readouterr().err


def test_main_table_for_another_file(tmp_path, capsys):
    rows = table_rows()[:2]
    x, _ = three_runs()
    path = write_motion(tmp_path / 'a.dat', [x])
    write_table(tmp_path / 'a.csv', rows)
    assert pd.main(['play_dat', str(path), '--min-m', '-0.1', '--max-m', '0.1',
                    '--max-velocity-mps', '0.54', '--dry-run']) == 2
    assert 'lists 2 runs' in capsys.readouterr().err


def test_main_bad_runs_argument(tmp_path):
    assert dry_run(tmp_path, '--runs', '7') == 2
