#!/usr/bin/env python3
"""
Play wavemaker motion files (.dat) as pregenerated move_wavemaker goals.

File format (the binary layout of write2mcl.m; written by the wave_analysis generator):
  int32    time step in ms
  int32    number of channels
  float32  frames: channel 0..N-1 of step 0, channel 0..N-1 of step 1, ...
Little-endian. Channels are 0-based and named in CHANNELS: 'position' is the flap position in
metres at the attachment point, relative to upright, with the gain already applied; 'trigger'
is for external equipment and is not played yet. The values are played as they are, without
wavemaker_transfer_gain. Old MC-lab files in mm are not supported; generate them again.

Regular and irregular files are handled alike: a goal is only positions at a fixed interval.
The file is split into runs where the paddle is at rest for at least --min-gap-s seconds; gaps may
differ in length, and shorter pauses stay inside the run. Runs are found from the file: the
generator leaves out rejected runs, so run N in the file is not necessarily row N of the test
matrix. If a run table <file>.csv exists next to the .dat, it names the runs and gives their
boundaries.

Each run is one goal; the gaps are played as pauses. Ctrl-C cancels the current goal and stops
the programme. A rejected or aborted run stops it too. A goal can be long (an irregular run is
about 26 min) and has no heartbeat: Ctrl-C, the soft stop, Stop on the web page and the e-stop
stop it.

The goal carries positions, so the controller needs no special mode: run it next to the normal
lab launch (the wavemaker service, lab.launch.py), with the web page open if you like. The page
then plots the paddle and its idle auto-stop counts the runs as motion. The controller rejects
goals while the soft stop is engaged, and with wavemakers.launch.py alone no soft_stop node runs,
so use lab.launch.py. Without hardware:
    ros2 run rig_controller fake_controller --ros-args -r __ns:=/wavemakers/ladertanken

    ros2 run rig_controller play_dat 2002.dat ladertanken --dry-run
    ros2 run rig_controller play_dat 2002.dat ladertanken --runs 3-5

The rig name gives the limits, the upright position and the namespace from the yaml (rig_limits).
Without a name, give --min-m, --max-m and --max-velocity-mps (and --upright-m if not 0.0); with a
name, each of them overrides the yaml's value.
"""

import argparse
import csv
from dataclasses import dataclass, field
import os
import sys
import time

import numpy as np

import rig_limits

HEADER_BYTES = 8           # int32 time step [ms], int32 channel count
MIN_GAP_S = 1.0            # rest at least this long separates two runs
REST_SPEED_MPS = 1e-6      # slower than this (and near zero) counts as rest
SHORT_PAUSE_REPORT_S = 0.1  # rest shorter than MIN_GAP_S but longer than this is reported
UPRIGHT_TOLERANCE_M = 0.002  # a run should start and end this close to upright
CLASSIFY_SPREAD = 0.02     # zero-crossing spacing std / mean below this -> regular
CHANNELS = {'position': 0, 'trigger': 1}  # same as motion_file.py in wave_analysis
REQUIRED_COLUMNS = ('run', 'start_s', 'end_s')
RUN_TABLE_TOLERANCE_S = 0.05  # table boundary vs detected boundary
SERVER_TIMEOUT_S = 5.0
# Fraction of a run measured for period and size (skips the ramps).
MEASURE_FROM, MEASURE_TO = 0.15, 0.85


# ---------------------------------------------------------------- file handling (no ROS)

@dataclass
class MotionFile:
    sample_interval_s: float  # from the header
    channels: int
    data: np.ndarray         # frames x channels, float32, as in the file


@dataclass
class Run:
    index: int               # 0-based, as in the run table
    start_sample: int         # first frame in the file
    values_m: np.ndarray      # relative to upright (for the table and checks)
    positions_m: np.ndarray   # absolute, at the flap attachment point (sent in the goal)
    gap_before_s: float      # measured rest before this run, played as a pause
    kind: str = 'unknown'    # 'regular' | 'irregular', from the run table or classify_run
    # run table row (period_s, height_m, steepness, gain, amplitude_m)
    info: dict = field(default_factory=dict)
    short_pauses: list = field(default_factory=list)  # (start_s, length_s) kept inside the run


def read_motion(path) -> MotionFile:
    """Read a wavemaker motion file(write2mcl layout); raise ValueError if it is not one."""
    file_size = os.path.getsize(path)
    if (file_size < HEADER_BYTES):
        raise ValueError(f'{path}: {file_size} bytes. File is too small to contain a valid header')
    interval_ms, channels = (int(v) for v in np.fromfile(path, dtype='<i4', count=2))
    if interval_ms <= 0:
        raise ValueError(f'{path}: interval_ms must be positive, got {interval_ms}')
    if channels <= 0:
        raise ValueError(f'{path}: channels must be positive, got {channels}')

    frames, leftover = divmod(os.path.getsize(path) - HEADER_BYTES, channels * 4)
    if (leftover or frames == 0):
        raise ValueError(f'{path}: {file_size} bytes do not fit {channels} channels of float32')
    data = np.fromfile(path, dtype='<f4', offset=HEADER_BYTES).reshape((frames, channels))
    if not np.isfinite(data).all():
        step, channel = np.argwhere(~np.isfinite(data))[0]
        raise ValueError(f'{path}: non-finite value at step {step}, channel {channel}')
    return MotionFile(sample_interval_s=interval_ms / 1000.0, channels=channels, data=data)


def select_channel(motion: MotionFile, channel: str) -> np.ndarray:
    """One channel as float64, by name in CHANNELS or by 0-based index ('0', '1', ...)."""
    if channel in CHANNELS:
        index = CHANNELS[channel]
    elif channel.isdigit():
        index = int(channel)
    else:
        raise ValueError(f"unknown channel '{channel}'; use {', '.join(CHANNELS)} "
                         'or a 0-based index')
    if index >= motion.channels:
        raise ValueError(f'channel {index} does not exist; the file has {motion.channels}')
    # float64 avoids float32 rounding errors in the checks
    values = motion.data[:, index].astype(np.float64)
    if not values.any():
        raise ValueError(f'channel {index} is all zeros; this might be a wrong channel.')
    return values


def rest_mask(values_m: np.ndarray, sample_interval_s: float,
              gap_threshold_m: float) -> np.ndarray:
    """Mark the samples where the paddle is at rest: near zero and not moving."""
    speed_mps = np.abs(np.gradient(values_m)) / sample_interval_s
    return (np.abs(values_m) <= gap_threshold_m) & (speed_mps <= REST_SPEED_MPS)


def stretches(mask: np.ndarray) -> list[tuple[int, int]]:
    """(start, end) of each True stretch, end exclusive."""
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def split_runs(values_m: np.ndarray, sample_interval_s: float, upright_m: float,
               gap_threshold_m: float = 0.0, min_gap_s: float = MIN_GAP_S) -> list[Run]:
    """
    Split only where the paddle is at rest for at least min_gap_s; gaps may differ in length.

    At rest: |value| <= gap_threshold_m and |velocity| <= REST_SPEED_MPS, so a slow pass
    through zero inside an irregular run is never taken for a gap. When in doubt, keep it in the
    run: playing a pause inside a goal is harmless, cutting a goal while moving is not.
    """
    rest = rest_mask(values_m, sample_interval_s, gap_threshold_m)
    min_gap_samples = int(round(min_gap_s / sample_interval_s))
    moving, previous = [], 0
    for start, end in stretches(rest):
        # rest at the file's start or end has no run on its other side: never part of a run
        if end - start >= min_gap_samples or start == 0 or end == len(values_m):
            if start > previous:
                moving.append((previous, start))
            previous = end
    if previous < len(values_m):
        moving.append((previous, len(values_m)))
    runs, previous_end = [], 0
    for index, (start, end) in enumerate(moving):
        start, end = max(start - 1, 0), min(end + 1, len(values_m))  # one rest sample each end
        run_values_m = values_m[start:end]
        pauses = [(a * sample_interval_s, (b - a) * sample_interval_s)
                  for a, b in stretches(rest[start:end])
                  if (b - a) * sample_interval_s >= SHORT_PAUSE_REPORT_S]
        runs.append(Run(index, start, run_values_m, upright_m + run_values_m,
                        gap_before_s=(start - previous_end) * sample_interval_s,
                        short_pauses=pauses))
        previous_end = end
    return runs


def whole_file_run(values_m: np.ndarray, upright_m: float) -> list[Run]:
    """--no-split: the whole file as one run (fallback for files whose gaps can't be detected)."""
    return [Run(0, 0, values_m, upright_m + values_m, gap_before_s=0.0)]


def read_run_table(dat_path) -> list[dict] | None:
    """Rows of <file>.csv: required run, start_s, end_s; optional kind and any test parameters."""
    path = os.path.splitext(dat_path)[0] + '.csv'
    if not os.path.exists(path):
        return None
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f'{path}: missing columns {", ".join(missing)}')
        return [row for row in reader if row.get('status', '') != 'skipped']


def apply_run_table(runs: list[Run], table: list[dict], sample_interval_s: float) -> list[Run]:
    """Name the detected runs from the table; refuse if the boundaries do not match the file."""
    if len(table) != len(runs):
        raise ValueError(f'run table lists {len(table)} runs, the file has {len(runs)}; '
                         'is the table for another file?')
    for run, row in zip(runs, table):
        start_s = run.start_sample * sample_interval_s
        end_s = start_s + len(run.values_m) * sample_interval_s
        if (abs(float(row['start_s']) - start_s) > RUN_TABLE_TOLERANCE_S
                or abs(float(row['end_s']) - end_s) > RUN_TABLE_TOLERANCE_S):
            raise ValueError(f"run {row['run']}: table {row['start_s']}-{row['end_s']} s, "
                             f'file {start_s:.2f}-{end_s:.2f} s')
        run.kind = row.get('kind') or run.kind
        run.info = row
    return runs


def middle_part(values_m: np.ndarray) -> np.ndarray:
    """Return the run without its ramps: MEASURE_FROM to MEASURE_TO of its length."""
    n = len(values_m)
    return values_m[int(n * MEASURE_FROM):int(n * MEASURE_TO)]


def upward_zero_crossings_s(values_m: np.ndarray, sample_interval_s: float) -> np.ndarray:
    """Return the times where values cross zero going up, interpolated between samples."""
    i = np.flatnonzero((values_m[:-1] < 0) & (values_m[1:] >= 0))
    fraction = -values_m[i] / (values_m[i + 1] - values_m[i])
    return (i + fraction) * sample_interval_s


def classify_run(run: Run, sample_interval_s: float) -> str:
    """'regular' if the zero-crossing spacing is constant, else 'irregular'."""
    spacing_s = np.diff(upward_zero_crossings_s(middle_part(run.values_m), sample_interval_s))
    if len(spacing_s) < 2:
        return 'unknown'
    return 'regular' if spacing_s.std() / spacing_s.mean() < CLASSIFY_SPREAD else 'irregular'


def describe_run(run: Run, sample_interval_s: float) -> dict:
    """Numbers for the table; the period from the run table wins over the measured one."""
    middle_m = middle_part(run.values_m)
    d = {'start_s': run.start_sample * sample_interval_s,
         'duration_s': len(run.values_m) * sample_interval_s,
         'gap_before_s': run.gap_before_s,
         'max_abs_m': float(np.abs(run.values_m).max()),
         'peak_speed_mps': float(np.abs(np.diff(run.positions_m)).max() / sample_interval_s),
         'period_s': float('nan'),
         'source': 'measured'}
    if run.kind == 'regular':
        spacing_s = np.diff(upward_zero_crossings_s(middle_m, sample_interval_s))
        if len(spacing_s):
            d['period_s'] = float(spacing_s.mean())
        d['amplitude_m'] = float(middle_m.max() - middle_m.min()) / 2
    elif run.kind == 'irregular':
        spectrum = np.abs(np.fft.rfft(middle_m - middle_m.mean()))
        frequencies_hz = np.fft.rfftfreq(len(middle_m), sample_interval_s)
        d['period_s'] = float(1 / frequencies_hz[1:][spectrum[1:].argmax()])  # peak period
        d['excursion_m'] = 4 * float(middle_m.std())  # flap, not the wave's significant height
    period_s = run.info.get('period_s') or run.info.get('peak_period_s')
    if period_s:
        d['period_s'], d['source'] = float(period_s), 'table'
    return d


def check_run(run: Run, sample_interval_s: float, upright_m: float, minimum_m: float,
              maximum_m: float, max_velocity_mps: float) -> list[str]:
    """Reasons the controller would reject this run, or it should not be played; empty if fine."""
    issues = []
    low_m, high_m = run.positions_m.min(), run.positions_m.max()
    if low_m < minimum_m or high_m > maximum_m:
        issues.append(f'travel {low_m:+.4f}..{high_m:+.4f} m outside '
                      f'{minimum_m:+.4f}..{maximum_m:+.4f} m')
    speed_mps = np.abs(np.diff(run.positions_m)).max() / sample_interval_s
    if speed_mps > max_velocity_mps:
        issues.append(f'peak speed {speed_mps:.3f} > {max_velocity_mps:.3f} m/s')
    for where, position_m in (('starts', run.positions_m[0]), ('ends', run.positions_m[-1])):
        if abs(position_m - upright_m) > UPRIGHT_TOLERANCE_M:
            issues.append(f'{where} {position_m - upright_m:+.4f} m from upright')
    return issues


def print_table(runs: list[Run], sample_interval_s: float, issues: dict[int, list[str]]):
    """One line per run, with OK or the issues; short pauses listed under their run."""
    print(f"{'run':>3}  {'kind':9} {'start':>7} {'length':>7} {'gap':>6} {'period':>6}"
          f" {'max|x|':>7} {'speed':>6}  status")
    for run in runs:
        d = describe_run(run, sample_interval_s)
        status = '; '.join(issues.get(run.index, [])) or 'OK'
        print(f"{run.index:>3}  {run.kind:9} {d['start_s']:7.1f} {d['duration_s']:7.1f}"
              f" {d['gap_before_s']:6.1f} {d['period_s']:6.3f} {d['max_abs_m']:7.4f}"
              f" {d['peak_speed_mps']:6.3f}  {status}")
        for start_s, length_s in run.short_pauses:
            print(f'      pause at {start_s:.2f} s for {length_s:.2f} s (kept in the run)')
    print('times in s, max|x| in m relative to upright, speed in m/s')


# ---------------------------------------------------------------- ROS (goals)

def play(runs: list[Run], sample_interval_s: float, rig: str, ros_args: list[str],
         limits=None) -> int:
    """Send each run as a move_<rig> goal, in order; return an exit code."""
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from rig_interfaces.action import MoveRig

    rclpy.init(args=ros_args)
    node = rclpy.create_node('play_dat')
    client = ActionClient(node, MoveRig, f'move_{rig}')
    goal_handle = None
    try:
        if not client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
            node.get_logger().error(f'no move_{rig} server; is the controller up and active?')
            return 3
        if limits is not None:
            cross_check_limits(node, limits)
        for run in runs:
            pause_end_s = time.monotonic() + run.gap_before_s
            while time.monotonic() < pause_end_s:
                rclpy.spin_once(node, timeout_sec=0.1)
            d = describe_run(run, sample_interval_s)
            node.get_logger().info(
                f"run {run.index} ({run.kind}): {d['duration_s']:.1f} s, "
                f"max {d['max_abs_m']:.4f} m, {d['peak_speed_mps']:.3f} m/s")
            goal = MoveRig.Goal()
            goal.positions = run.positions_m.tolist()
            goal.sample_interval = sample_interval_s
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future)
            goal_handle = future.result()  # None if spinning stopped early (shutdown)
            if goal_handle is None or not goal_handle.accepted:
                node.get_logger().error(f'run {run.index}: goal rejected, see the controller log')
                return 4
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            response = result_future.result()
            status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
            if status != GoalStatus.STATUS_SUCCEEDED:
                node.get_logger().error(f'run {run.index}: ended with status {status}; stopping')
                return 5
            goal_handle = None
        return 0
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# ---------------------------------------------------------------- command line

def parse_runs(text: str, count: int) -> list[int]:
    """'3', '3-5', '3,7,9' -> run indices (0-based, as in the table); '' -> all runs."""
    if not text:
        return list(range(count))
    indices = []
    for part in text.split(','):
        first, _, last = part.partition('-')
        indices.extend(range(int(first), int(last or first) + 1))
    bad = [i for i in indices if not 0 <= i < count]
    if bad:
        raise ValueError(f'no run {bad[0]}; the file has runs 0-{count - 1}')
    return indices


LIMIT_FIELDS = ('upright_m', 'min_m', 'max_m', 'max_velocity_mps')


def resolve_limits(args, parser) -> tuple['rig_limits.RigLimits | None', dict]:
    """
    Resolve the limits to check against.

    The rig's yaml when a name is given, each option on the command line overriding one value.
    Prints where they came from. Returns the yaml's limits (for the live cross-check; None
    without a name) and the values to use.
    """
    limits = rig_limits.load_limits(args.rig, args.name, args.config) if args.name else None
    values, overridden = {}, []
    for key in LIMIT_FIELDS:  # not 'field': play_dat imports dataclasses.field
        given = getattr(args, key)
        if given is not None:
            values[key] = given
            if limits is not None:
                overridden.append('--' + key.replace('_', '-'))
        else:
            values[key] = getattr(limits, key) if limits is not None else None
    if values['upright_m'] is None:
        values['upright_m'] = 0.0  # the old default, for the form without a name
    if limits is None and None in (values['min_m'], values['max_m'], values['max_velocity_mps']):
        parser.error('give the rig name, or --min-m, --max-m and --max-velocity-mps')
    source = f'{limits.source} ({limits.name})' if limits is not None else 'the command line'
    if overridden:
        source += f", {', '.join(overridden)} overridden"
    print(f"limits from {source}: {values['min_m']:+.5f}..{values['max_m']:+.5f} m, "
          f"{values['max_velocity_mps']:.4f} m/s, upright {values['upright_m']:+.4f} m")
    return limits, values


def with_namespace(argv: list[str], limits) -> list[str]:
    """Add the rig's namespace as a ROS remap, unless the command line already has one."""
    if limits is None or any(a.startswith('__ns:=') for a in argv):
        return argv
    return [*argv, '--ros-args', '-r', f'__ns:={limits.namespace}']


# Every parameter limits_from_parameters reads; asked from the running controller.
LIMIT_PARAMETERS = [
    'drive_name', 'drive_motion', 'upright_position_m', 'actuator_lead_m_per_degree',
    'actuator_upright_angle_deg', 'min_position_deg', 'max_position_deg', 'max_velocity_rpm',
    'drive_m_per_unit', 'drive_velocity_per_unit_per_s',
    'travel_minimum_m', 'travel_maximum_m', 'max_velocity_mps', 'max_acceleration_mps2',
    'max_goal_acceleration_mps2',
]


def cross_check_limits(node, limits) -> None:
    """
    Warn if the running controller's limits differ from the yaml's.

    The runs were checked against the yaml; they can differ on a Pi not pulled or rebuilt, or
    with a yaml edited after configure. Never stops the play: the controller enforces its own
    limits regardless.
    """
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient

    log = node.get_logger()
    client = AsyncParameterClient(node, f'{limits.namespace}/controller')
    if not client.wait_for_services(timeout_sec=2.0):
        log.warn('controller parameters not available; limits not cross-checked')
        return
    # get_parameters answers nothing at all if one name is not declared (rclcpp and rclpy), and
    # each drive declares only its own (max_velocity_rpm on the IndraDrive, max_velocity_mps on
    # the LinMot): ask for the declared ones only.
    listed = client.list_parameters()
    rclpy.spin_until_future_complete(node, listed, timeout_sec=2.0)
    response = listed.result() if listed.done() else None
    if response is None:
        log.warn('controller parameters not available; limits not cross-checked')
        return
    declared = set(response.result.names)
    keys = [key for key in LIMIT_PARAMETERS if key in declared]
    future = client.get_parameters(keys)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    response = future.result() if future.done() else None
    if response is None:
        log.warn('controller parameters not available; limits not cross-checked')
        return
    params = {key: parameter_value_to_python(value)
              for key, value in zip(keys, response.values)}
    try:
        live = rig_limits.limits_from_parameters(limits.rig, limits.name, params, 'controller')
    except ValueError as e:
        log.warn(f'controller parameters incomplete ({e}); limits not cross-checked')
        return
    differences = rig_limits.compare(limits, live)
    for line in differences:
        log.warn(f'yaml and controller differ: {line}')
    if not differences:
        log.info('limits match the controller')


def main(argv=None):
    from rclpy.utilities import remove_ros_args
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description='Play motion files as pregenerated waves.')
    parser.add_argument('file')
    parser.add_argument('name', nargs='?',
                        help='rig name in the yaml (ladertanken, sloshing): gives the limits, the '
                             'upright position and the namespace')
    parser.add_argument('--rig', default='wavemaker', choices=('wavemaker', 'oscillator'),
                        help='action server move_<rig> and the yaml it is in; default wavemaker')
    parser.add_argument('--config', help='another yaml; default the installed one')
    parser.add_argument('--channel', default='position',
                        help=f'name ({", ".join(CHANNELS)}) or 0-based index; default position')
    parser.add_argument('--upright-m', type=float, help='override; default the yaml, else 0.0')
    parser.add_argument('--runs', default='', help="e.g. '3', '3-5', '3,7'; default all")
    parser.add_argument('--gap-threshold-m', type=float, default=0.0,
                        help='|position| at or below this counts as rest; default exact zeros')
    parser.add_argument('--min-gap-s', type=float, default=MIN_GAP_S,
                        help='shortest rest that separates two runs')
    parser.add_argument('--no-split', action='store_true',
                        help='send the whole file as one goal')
    parser.add_argument('--min-m', type=float, help='lowest position; overrides the yaml')
    parser.add_argument('--max-m', type=float, help='highest position; overrides the yaml')
    parser.add_argument('--max-velocity-mps', type=float, help='overrides the yaml')
    parser.add_argument('--dry-run', action='store_true', help='print the run table only')
    args = parser.parse_intermixed_args(remove_ros_args(argv)[1:])

    try:
        limits, lim = resolve_limits(args, parser)
    except (OSError, ValueError) as e:  # yaml missing, rig not in it, a limit not set
        print(f'error: {e}', file=sys.stderr)
        return 2
    if limits is not None:
        print(f'namespace {limits.namespace}')

    try:
        motion = read_motion(args.file)
        values_m = select_channel(motion, args.channel)
        sample_interval_s = motion.sample_interval_s
        runs = (whole_file_run(values_m, lim['upright_m']) if args.no_split else
                split_runs(values_m, sample_interval_s, lim['upright_m'],
                           args.gap_threshold_m, args.min_gap_s))
        table = None if args.no_split else read_run_table(args.file)
        if table:
            runs = apply_run_table(runs, table, sample_interval_s)
        else:
            print('no run table; kinds and periods are measured')
        for run in runs:
            if run.kind == 'unknown':
                run.kind = classify_run(run, sample_interval_s)
        selected = [runs[i] for i in parse_runs(args.runs, len(runs))]
    except (OSError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    issues = {r.index: check_run(r, sample_interval_s, lim['upright_m'], lim['min_m'],
                                 lim['max_m'], lim['max_velocity_mps'])
              for r in selected}
    print_table(selected, sample_interval_s, issues)
    if args.dry_run:
        return 0
    if any(issues.values()):
        print('not playing: fix the problems above (nothing moved)', file=sys.stderr)
        return 1
    return play(selected, sample_interval_s, args.rig, with_namespace(argv, limits), limits)


if __name__ == '__main__':
    sys.exit(main())
