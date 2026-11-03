# Sketch: `play_dat` takes its limits from the yaml (TODO item 40)

## Problem

`play_dat` checks every run before playing it, against limits typed by hand:

```bash
ros2 run rig_controller play_dat waves.dat --min-m -0.14575 --max-m 0.14575 \
  --max-velocity-mps 0.7632 --upright-m 0.0 --ros-args -r __ns:=/wavemakers/ladertanken
```

All five values follow from the rig's name and its yaml block. A typo gives a misleading dry-run
result (the controller still enforces its own limits, so this is about a trustworthy dry run, not
safety). `--upright-m` defaults to 0.0, which is wrong for the oscillator (0.1 m).

## Goal

```bash
ros2 run rig_controller play_dat waves.dat ladertanken --dry-run
ros2 run rig_controller play_dat waves.dat ladertanken
ros2 run rig_controller play_dat waves.dat sloshing --rig oscillator
```

## Decisions (2026-11-03)

1. **The yaml for the checks; the running controller as a cross-check when playing.** The dry run
   must work without the controller. When playing, one parameter request catches a Pi whose yaml
   or build differs; a difference is a warning, not a stop (the controller enforces its own limits).
2. **The rig name is an optional positional argument after the file**, as in
   `generate_waves ladertanken matrix.csv`.
3. **The old form stays:** no name and all three limits given. Every run prints where its limits
   came from.

## Files

| File | Change | Section |
|---|---|---|
| `rig_controller/src/rig_limits.py` | New: limits from a yaml or a parameter dict | 1 |
| `rig_controller/src/play_dat.py` | Rig name, overrides, namespace, live cross-check | 2 |
| `rig_controller/src/generate_waves.py` | Limits from `rig_limits`; `required()` moves there | 3 |
| `rig_controller/CMakeLists.txt` | Install `rig_limits.py` | 4 |
| `rig_controller/test/test_rig_limits.py` | New | 5 |
| `rig_controller/test/test_play_dat.py`, `test_generate_waves.py` | Import path, new cases | 5 |
| README, `play_dat.py` docstring | Short form in the examples | 6 |

---

## 1. `rig_limits.py` (new, complete)

The rules are the controller's, from three places:

| Case | Recognised by | `min_m`, `max_m` | `max_velocity_mps` | `max_acceleration_mps2` |
|---|---|---|---|---|
| IndraDrive, angular (ladertanken) | `drive_motion: angular` | `upright_m + (min/max_position_deg - actuator_upright_angle_deg) * actuator_lead_m_per_degree` (`controller_node.cpp` ~212-223, `drive_scale.hpp`) | `max_velocity_rpm * 6 * lead` (`indradrive_actuator.cpp` ~249) | `max_goal_acceleration_mps2` if > 0 |
| IndraDrive, linear (lilletanken, mclab) | `drive_motion: linear`, `drive_name: indradrive` | `travel_minimum_m`, `travel_maximum_m` (~224-244) | none: the controller's `rpm * 6` is not m/s (TODO item 19) | as above |
| LinMot (oscillator) | `drive_name: linmot` | `travel_minimum_m`, `travel_maximum_m` | `max_velocity_mps`, default 2.0 (`linmot_actuator.cpp`) | the lower of `max_goal_acceleration_mps2` (if > 0) and `max_acceleration_mps2` (default 20.0) |

`drive_name` is required: the controller declares it as `''` and refuses to configure without a
known drive. Every key that sets a limit is required; only offsets and the LinMot defaults fall
back on the controller's defaults.

```python
#!/usr/bin/env python3
"""
Goal limits of a rig, from its yaml block or its live parameters, derived as the controller does.

Shared by play_dat and generate_waves so they cannot disagree. The rules are the controller's:
position limits in controller_node.cpp on_configure (~212-244, through DriveScale), speed and
acceleration from the actuator (indradrive_actuator.cpp, linmot_actuator.cpp) and
max_goal_acceleration_mps2.
"""

from dataclasses import dataclass
import os

import yaml

# rig -> (namespace prefix, bringup package, yaml file)
RIGS = {
    'wavemaker': ('wavemakers', 'wavemaker_bringup', 'wavemakers.yaml'),
    'oscillator': ('oscillators', 'oscillator_bringup', 'oscillator.yaml'),
}
LINMOT_MAX_VELOCITY_MPS = 2.0       # linmot_actuator.cpp defaults
LINMOT_MAX_ACCELERATION_MPS2 = 20.0


@dataclass
class RigLimits:
    rig: str
    name: str
    namespace: str
    upright_m: float
    min_m: float                          # absolute goal limits
    max_m: float
    max_velocity_mps: float | None        # None: not known (linear IndraDrive, TODO item 19)
    max_acceleration_mps2: float | None   # None: no limit
    source: str                           # 'wavemakers.yaml', 'controller', ...


def required(params: dict, key: str, name: str, source: str):
    """A key with no safe default: the controller's generic default would give the wrong limits."""
    if key not in params:
        raise ValueError(f'{name}: {key} not set in {source}')
    return params[key]


def default_config_path(rig: str) -> str:
    """The installed yaml when the workspace is sourced, else the repository's copy."""
    _, package, filename = RIGS[rig]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
    except Exception:  # not sourced
        share = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', package)
    return os.path.join(share, 'config', filename)


def read_block(rig: str, name: str, config_path: str) -> dict:
    """ros__parameters of /<prefix>/<name>/controller; lists the names in the file if missing."""
    prefix = RIGS[rig][0]
    with open(config_path) as f:
        config = yaml.safe_load(f) or {}
    key = f'/{prefix}/{name}/controller'
    if key not in config:
        names = [k.split('/')[2] for k in config if k.startswith(f'/{prefix}/')]
        raise ValueError(f"no {rig} '{name}' in {config_path}; have {', '.join(names) or 'none'}")
    return config[key]['ros__parameters']


def limits_from_parameters(rig: str, name: str, params: dict, source: str) -> RigLimits:
    """The controller's rules, applied to a yaml block or to parameters read from the controller."""
    drive = required(params, 'drive_name', name, source)
    upright_m = params.get('upright_position_m', 0.0)

    if params.get('drive_motion', 'linear') == 'angular':
        lead = required(params, 'actuator_lead_m_per_degree', name, source)
        if lead <= 0.0:
            raise ValueError(f'{name}: actuator_lead_m_per_degree must be > 0 in {source}')
        upright_deg = params.get('actuator_upright_angle_deg', 0.0)
        min_m = upright_m + (required(params, 'min_position_deg', name, source) - upright_deg) * lead
        max_m = upright_m + (required(params, 'max_position_deg', name, source) - upright_deg) * lead
        max_velocity = required(params, 'max_velocity_rpm', name, source) * 6.0 * lead
    else:
        min_m = required(params, 'travel_minimum_m', name, source)
        max_m = required(params, 'travel_maximum_m', name, source)
        max_velocity = (params.get('max_velocity_mps', LINMOT_MAX_VELOCITY_MPS)
                        if drive == 'linmot' else None)

    goal_acceleration = params.get('max_goal_acceleration_mps2', 0.0)
    acceleration = goal_acceleration if goal_acceleration > 0.0 else None
    if drive == 'linmot':
        drive_acceleration = params.get('max_acceleration_mps2', LINMOT_MAX_ACCELERATION_MPS2)
        acceleration = (drive_acceleration if acceleration is None
                        else min(acceleration, drive_acceleration))

    # The controller refuses to configure with these, so a file checked against them means nothing.
    if not min_m < max_m:
        raise ValueError(f'{name}: lower limit {min_m} not below upper limit {max_m} in {source}')
    if not min_m <= upright_m <= max_m:
        raise ValueError(f'{name}: upright {upright_m} outside {min_m}..{max_m} in {source}')

    return RigLimits(rig, name, f'/{RIGS[rig][0]}/{name}', upright_m, min_m, max_m,
                     max_velocity, acceleration, source)


def load_limits(rig: str, name: str, config_path: str | None = None) -> RigLimits:
    config_path = config_path or default_config_path(rig)
    params = read_block(rig, name, config_path)
    return limits_from_parameters(rig, name, params, os.path.basename(config_path))


def compare(expected: RigLimits, actual: RigLimits) -> list[str]:
    """Differences worth a warning: positions beyond 0.1 mm, speed and acceleration beyond 1e-3."""
    lines = []
    for field in ('upright_m', 'min_m', 'max_m'):
        e, a = getattr(expected, field), getattr(actual, field)
        if abs(e - a) > 1e-4:
            lines.append(f'{field}: {expected.source} {e:+.5f} m, {actual.source} {a:+.5f} m')
    e, a = expected.max_velocity_mps, actual.max_velocity_mps
    if e is not None and a is not None and abs(e - a) > 1e-3:
        lines.append(f'max_velocity_mps: {expected.source} {e:.4f}, {actual.source} {a:.4f}')
    e, a = expected.max_acceleration_mps2, actual.max_acceleration_mps2
    if (e is None) != (a is None) or (e is not None and abs(e - a) > 1e-3):
        lines.append(f'max_acceleration_mps2: {expected.source} {e}, {actual.source} {a}')
    return lines
```

Checked against today's yamls:

| Rig | `min_m` | `max_m` | `max_velocity_mps` | `max_acceleration_mps2` | `upright_m` | namespace |
|---|---|---|---|---|---|---|
| ladertanken | -0.14575 | +0.14575 | 0.7632 | None | 0.0 | `/wavemakers/ladertanken` |
| lilletanken, mclab | 0.0 | 0.5 | None | None | 0.0 | `/wavemakers/<name>` |
| sloshing | 0.015 | 0.185 | 2.0 | 9.81 | 0.1 | `/oscillators/sloshing` |

---

## 2. `play_dat.py`

### 2.1 Import

At the top, with the other imports. Installed next to `play_dat` (section 4), so it imports the
same way `generate_waves` imports `play_dat`:

```python
import rig_limits
```

### 2.2 New functions

Put them above `main()`.

```python
LIMIT_FIELDS = ('upright_m', 'min_m', 'max_m', 'max_velocity_mps')


def resolve_limits(args, parser) -> tuple['rig_limits.RigLimits | None', dict]:
    """
    The limits to check against: the rig's yaml when a name is given, each option on the command
    line overriding one value. Prints where they came from. Returns the yaml's limits (for the
    live cross-check; None without a name) and the values to use.
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
    if values['max_velocity_mps'] is None:
        parser.error(f'{args.name} has no speed limit in the yaml (TODO item 19): '
                     'give --max-velocity-mps')
    source = f'{limits.source} ({limits.name})' if limits is not None else 'the command line'
    if overridden:
        source += f", {', '.join(overridden)} overridden"
    print(f"limits from {source}: {values['min_m']:+.5f}..{values['max_m']:+.5f} m, "
          f"{values['max_velocity_mps']:.4f} m/s, upright {values['upright_m']:+.4f} m")
    return limits, values


def with_namespace(argv: list[str], limits) -> list[str]:
    """The rig's namespace as a ROS remap, unless the command line already has one."""
    if limits is None or any(a.startswith('__ns:=') for a in argv):
        return argv
    return [*argv, '--ros-args', '-r', f'__ns:={limits.namespace}']


# Every parameter limits_from_parameters reads; asked from the running controller.
LIMIT_PARAMETERS = [
    'drive_name', 'drive_motion', 'upright_position_m', 'actuator_lead_m_per_degree',
    'actuator_upright_angle_deg', 'min_position_deg', 'max_position_deg', 'max_velocity_rpm',
    'travel_minimum_m', 'travel_maximum_m', 'max_velocity_mps', 'max_acceleration_mps2',
    'max_goal_acceleration_mps2',
]


def cross_check_limits(node, limits) -> None:
    """
    Warn if the running controller's limits differ from the yaml's the runs were checked against
    (a Pi not pulled or rebuilt, a yaml edited after configure). Never stops the play: the
    controller enforces its own limits regardless.
    """
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient

    log = node.get_logger()
    client = AsyncParameterClient(node, f'{limits.namespace}/controller')
    if not client.wait_for_services(timeout_sec=2.0):
        log.warn('controller parameters not available; limits not cross-checked')
        return
    future = client.get_parameters(LIMIT_PARAMETERS)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.result() is None:
        log.warn('controller parameters not available; limits not cross-checked')
        return
    # A parameter the controller does not declare (e.g. max_velocity_rpm on the LinMot) comes
    # back as NOT_SET; leave it out, as if it were missing from the yaml.
    params = {key: parameter_value_to_python(value)
              for key, value in zip(LIMIT_PARAMETERS, future.result().values)
              if value.type != ParameterType.PARAMETER_NOT_SET}
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
```

### 2.3 `main()`: arguments

Replace the `--rig`, `--upright-m`, `--min-m`, `--max-m` and `--max-velocity-mps` lines
(today ~358-373); the others stay:

```python
    parser.add_argument('file')
    parser.add_argument('name', nargs='?',
                        help='rig name in the yaml (ladertanken, sloshing): gives the limits, the '
                             'upright position and the namespace')
    parser.add_argument('--rig', default='wavemaker', choices=('wavemaker', 'oscillator'),
                        help='action server move_<rig> and the yaml it is in; default wavemaker')
    parser.add_argument('--config', help='another yaml; default the installed one')
    parser.add_argument('--upright-m', type=float, help='override; default the yaml, else 0.0')
    parser.add_argument('--min-m', type=float, help='lowest position; overrides the yaml')
    parser.add_argument('--max-m', type=float, help='highest position; overrides the yaml')
    parser.add_argument('--max-velocity-mps', type=float, help='overrides the yaml')
```

### 2.4 `main()`: body

Parse with `parse_intermixed_args` instead of `parse_args`:

```python
    args = parser.parse_intermixed_args(remove_ros_args(argv)[1:])
```

With `parse_args`, an optional positional is only taken straight after the file:
`play_dat a.dat ladertanken --dry-run` works, but `play_dat a.dat --dry-run ladertanken` fails
with "unrecognized arguments: ladertanken". `parse_intermixed_args` takes the name anywhere
(`test_name_after_the_options`).

Then, before reading the file:

```python
    try:
        limits, lim = resolve_limits(args, parser)
    except (OSError, ValueError) as e:  # yaml missing, rig not in it, a limit not set
        print(f'error: {e}', file=sys.stderr)
        return 2
    if limits is not None:
        print(f'namespace {limits.namespace}')
```

Then replace each use of the old arguments:

| Today | After |
|---|---|
| `whole_file_run(values_m, args.upright_m)` | `whole_file_run(values_m, lim['upright_m'])` |
| `split_runs(values_m, sample_interval_s, args.upright_m, ...)` | `split_runs(values_m, sample_interval_s, lim['upright_m'], ...)` |
| `check_run(r, sample_interval_s, args.upright_m, args.min_m, args.max_m, args.max_velocity_mps)` | `check_run(r, sample_interval_s, lim['upright_m'], lim['min_m'], lim['max_m'], lim['max_velocity_mps'])` |
| `return play(selected, sample_interval_s, args.rig, argv)` | `return play(selected, sample_interval_s, args.rig, with_namespace(argv, limits), limits)` |

### 2.5 `play()`

Add the parameter and one call after the action server is found:

```python
def play(runs: list[Run], sample_interval_s: float, rig: str, ros_args: list[str],
         limits=None) -> int:
    ...
        if not client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
            node.get_logger().error(f'no move_{rig} server; is the controller up and active?')
            return 3
        if limits is not None:
            cross_check_limits(node, limits)
    ...
```

### What it prints

```
limits from wavemakers.yaml (ladertanken): -0.14575..+0.14575 m, 0.7632 m/s, upright +0.0000 m
namespace /wavemakers/ladertanken
limits from wavemakers.yaml (ladertanken), --max-m overridden: -0.14575..+0.10000 m, 0.7632 m/s, upright +0.0000 m
limits from the command line: -0.10000..+0.10000 m, 0.5000 m/s, upright +0.0000 m
limits from oscillator.yaml (sloshing): +0.01500..+0.18500 m, 2.0000 m/s, upright +0.1000 m
play_dat: error: lilletanken has no speed limit in the yaml (TODO item 19): give --max-velocity-mps
play_dat: error: give the rig name, or --min-m, --max-m and --max-velocity-mps
```

An explicit `--ros-args -r __ns:=/x` on the command line is left alone and wins.

---

## 3. `generate_waves.py`

### 3.1 Import and config path

```python
import play_dat
import rig_limits
```

Remove `import yaml`: with `load_wavemaker` reading through `rig_limits.read_block`, nothing in
`generate_waves.py` uses it any more (pyflakes reports it).

Replace the body of `default_config_path()`:

```python
def default_config_path() -> str:
    return rig_limits.default_config_path('wavemaker')
```

Delete `required()`; its uses become `rig_limits.required(params, key, name, source)`.

### 3.2 `load_wavemaker()`

The limits come from `rig_limits`; the geometry and gain stay here:

```python
def load_wavemaker(name: str, config_path: str) -> Wavemaker:
    """
    The wavemaker's geometry and gain from its yaml block, and its limits from rig_limits (the
    same derivation play_dat uses). Keys that set the geometry are required, as the limits are.
    """
    params = rig_limits.read_block('wavemaker', name, config_path)
    source = os.path.basename(config_path)
    limits = rig_limits.limits_from_parameters('wavemaker', name, params, source)

    wavemaker_type = rig_limits.required(params, 'wavemaker_type', name, source)
    if wavemaker_type == 'flap':
        attachment_height = rig_limits.required(params, 'flap_attachment_height', name, source)
    else:
        attachment_height = 0.0  # unused for a piston

    return Wavemaker(
        name=name,
        type=wavemaker_type,
        water_depth=rig_limits.required(params, 'water_depth', name, source),
        hinge_height=params.get('hinge_height', 0.0),
        attachment_height=attachment_height,
        gain=params.get('wavemaker_transfer_gain', 1.0),
        gain_periods=list(params.get('wavemaker_transfer_gain_periods', [])),
        gain_values=list(params.get('wavemaker_transfer_gain_values', [])),
        min_m=limits.min_m, max_m=limits.max_m, upright_m=limits.upright_m,
        max_velocity_mps=limits.max_velocity_mps,
        max_acceleration_mps2=limits.max_acceleration_mps2)
```

Keep the docstring's limit formulas and test values in `rig_limits.py` instead.

### 3.3 `main()`: the printed command

Replace the last `print`:

```python
    command = f'ros2 run rig_controller play_dat {dat_path} {wm.name} --dry-run'
    if wm.max_velocity_mps is None:  # linear drive: the speed came from the command line
        command += f' --max-velocity-mps {max_velocity:.4f}'
    if os.path.realpath(args.config) != os.path.realpath(default_config_path()):
        command += f' --config {args.config}'
    print(command)
```

`realpath` because the repository fallback path contains `..`.

---

## 4. `CMakeLists.txt`

Extend the `install(FILES ...)` added for `generate_waves`:

```cmake
# Imported by play_dat and generate_waves; the programs are installed without .py.
install(FILES src/play_dat.py src/rig_limits.py
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## 5. Tests

Written before the code (2026-11-03), so they fail until it exists. Against today's code the 124
existing tests pass and only the new ones fail; against this sketch applied to a copy of the
repository all 162 pass. A test still failing once you are done points at a real difference from
the sketch.

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test -v
# until rig_limits.py exists, test_rig_limits.py cannot import and stops the run; meanwhile:
python3 -m pytest rig_controller/test -q --continue-on-collection-errors
```

| File | What was added | Waiting for |
|---|---|---|
| `test_rig_limits.py` (new, 25 tests) | The table of section 1 for all four rigs; LinMot defaults; the goal acceleration limit on an IndraDrive; every missing key named (linear and angular); the three configurations the controller refuses; an unknown name; `default_config_path`; `compare`: yaml and parameters agree, a changed position, speed or acceleration is reported, the linear-drive speed is ignored, below 0.1 mm is tolerated | Section 1 |
| `test_play_dat.py` (`src` on `sys.path`, 12 new tests) | Limits from the yaml by name; the name after the options; one override; an override checked against; no name and no limits; one limit missing; the old form's printed line; a linear drive without and with `--max-velocity-mps`; an unknown name; the oscillator's 0.1 m upright and namespace; `with_namespace` | Sections 1 and 2 |
| `test_generate_waves.py` | `PISTON` gains `drive_name`; `drive_name` in the required keys; the end-to-end test checks the printed `play_dat … ladertanken --dry-run` | Section 3 |

`cross_check_limits` needs a node and is not covered: check it by hand. Against
`fake_controller` it prints "controller parameters not available; limits not cross-checked" (the
fake declares no parameters); against the real controller "limits match the controller".

## 6. Documentation

- `play_dat.py` docstring, the examples:
  ```
  ros2 run rig_controller play_dat 2002.dat ladertanken --dry-run
  ros2 run rig_controller play_dat 2002.dat ladertanken --runs 3-5
  ```
  plus: "Without a name, give --min-m, --max-m and --max-velocity-mps (and --upright-m if not
  0.0); with a name, each of them overrides the yaml's value."
- README *Generating motion files*, *Check and play*: the short form.
- `rpi/README.md`, rig test *Goal type per goal and `play_dat`*: the old form still works; move to
  the short form when it is next run.

## Out of scope

- The water depth: `play_dat` sends positions only. Whether a file was made for the current depth
  is item 39's record.
- The trigger channel (planned feature *Trigger output*).
- `fake_controller` declaring the limit parameters: only needed for an automated end-to-end test of
  the cross-check.
