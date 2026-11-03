# Sketch: `play_dat` takes its limits from the yaml (TODO item 40)

## Problem

`play_dat` checks every run before playing it, against limits typed by hand:

```bash
ros2 run rig_controller play_dat waves.dat --min-m -0.14575 --max-m 0.14575 \
  --max-velocity-mps 0.7632 --upright-m 0.0 --ros-args -r __ns:=/wavemakers/ladertanken
```

All five values follow from the rig's name and its yaml block. `generate_waves` already works the
limits out from `wavemakers.yaml`; `play_dat` does not. A typo gives a misleading dry-run result.
The controller still enforces its own limits, so this is about a trustworthy dry run, not about
safety. `--upright-m` defaults to 0.0, which is wrong for the oscillator (0.1 m).

## Goal

```bash
ros2 run rig_controller play_dat waves.dat ladertanken --dry-run
ros2 run rig_controller play_dat waves.dat ladertanken
ros2 run rig_controller play_dat waves.dat sloshing --rig oscillator
```

The name gives the limits, the upright position and the namespace. The old form (no name, all
limits typed) keeps working.

## Decisions (2026-11-03)

1. **The yaml for the checks, the running controller as a cross-check when playing.** The dry run
   must work without the controller; when playing, the controller is up anyway, so one parameter
   request catches a Pi whose yaml or build differs from the one the dry run used. A difference
   is a warning, not a stop: the controller rejects goals outside its own limits regardless.
2. **The rig name is an optional positional argument after the file**, as in
   `generate_waves ladertanken matrix.csv`.
3. **The old form stays:** no name and all three limits given. Every run prints where its limits
   came from.

## Files

| File | Change |
|---|---|
| `rig_controller/src/rig_limits.py` | New. Reads limits from a yaml or from a parameter dict, the same rules for both |
| `rig_controller/src/play_dat.py` | Rig name, overrides, namespace, live cross-check |
| `rig_controller/src/generate_waves.py` | Takes its limits from `rig_limits`; `required()` moves there |
| `rig_controller/CMakeLists.txt` | Install `rig_limits.py` next to the programs |
| `rig_controller/test/test_rig_limits.py` | New |
| `rig_controller/test/test_play_dat.py`, `test_generate_waves.py` | New and changed cases |
| `README.md`, `rpi/README.md`, `play_dat.py` docstring | Short form in the examples |

---

## 1. `rig_limits.py`

### The rules, from the controller

The controller derives a goal's position limits in `on_configure` (`controller_node.cpp`
~212-244) through `DriveScale` (`drive_scale.hpp`), and its speed and acceleration limits from
the actuator (`indradrive_actuator.cpp` ~249-256, `linmot_actuator.cpp` ~151-158) and
`max_goal_acceleration_mps2` (`controller_node.cpp` ~931-948).

| Case | How to recognise it | `min_m`, `max_m` | `max_velocity_mps` | `max_acceleration_mps2` |
|---|---|---|---|---|
| IndraDrive, angular (ladertanken) | `drive_motion == 'angular'` | `upright_m + (min/max_position_deg - actuator_upright_angle_deg) * actuator_lead_m_per_degree` | `max_velocity_rpm * 6 * lead` | `max_goal_acceleration_mps2` if > 0, else none |
| IndraDrive, linear (lilletanken, mclab) | `drive_name == 'indradrive'`, `drive_motion == 'linear'` | `travel_minimum_m`, `travel_maximum_m` | none (TODO item 19: the controller's value, `rpm * 6`, is not a speed in m/s) | as above |
| LinMot (oscillator) | `drive_name == 'linmot'` | `travel_minimum_m`, `travel_maximum_m` | `max_velocity_mps` | the lower of `max_goal_acceleration_mps2` (if > 0) and `max_acceleration_mps2` |

`upright_m` is `upright_position_m` in every case.

Defaults when a key is missing from the yaml (the controller's `declare_parameter` defaults):
`drive_motion`
`'linear'`, `upright_position_m` 0.0, `actuator_upright_angle_deg` 0.0,
`max_goal_acceleration_mps2` 0.0, LinMot `max_velocity_mps` 2.0, LinMot `max_acceleration_mps2`
20.0. `drive_name` has no usable default (the controller declares it as `''` and refuses to
configure without a known drive), so it is required too. Every other key in the table sets a
limit and is required: no fallback on the controller's generic defaults (as `generate_waves`
already does).

Expected values from today's yamls:

| Rig | `min_m` | `max_m` | `max_velocity_mps` | `max_acceleration_mps2` | `upright_m` | namespace |
|---|---|---|---|---|---|---|
| ladertanken | -0.14575 | +0.14575 | 0.7632 | none | 0.0 | `/wavemakers/ladertanken` |
| lilletanken | 0.0 | 0.5 | none | none | 0.0 | `/wavemakers/lilletanken` |
| sloshing | 0.015 | 0.185 | 2.0 | 9.81 | 0.1 | `/oscillators/sloshing` |

### Contents

```python
#!/usr/bin/env python3
"""Goal limits of a rig, derived from its yaml block or its live parameters as the controller does."""

from dataclasses import dataclass
import os

import yaml

RIGS = {  # rig -> (namespace prefix, bringup package, yaml file)
    'wavemaker': ('wavemakers', 'wavemaker_bringup', 'wavemakers.yaml'),
    'oscillator': ('oscillators', 'oscillator_bringup', 'oscillator.yaml'),
}


@dataclass
class RigLimits:
    rig: str
    name: str
    namespace: str
    upright_m: float
    min_m: float
    max_m: float
    max_velocity_mps: float | None
    max_acceleration_mps2: float | None
    source: str        # for the printed line, e.g. 'wavemakers.yaml' or 'controller'


def required(params: dict, key: str, name: str, source: str): ...
def default_config_path(rig: str) -> str: ...
def read_block(rig: str, name: str, config_path: str) -> dict: ...
def limits_from_parameters(rig: str, name: str, params: dict, source: str) -> RigLimits: ...
def load_limits(rig: str, name: str, config_path: str | None = None) -> RigLimits: ...
def compare(expected: RigLimits, actual: RigLimits) -> list[str]: ...
```

### `required(params, key, name, source)`

Moves here from `generate_waves.py`, with the source added to the message so a missing key in a
yaml and a missing parameter on the controller read differently:

- Return `params[key]` if present.
- Otherwise raise `ValueError(f"{name}: {key} not set in {source}")`.

### `default_config_path(rig)`

As `generate_waves.default_config_path` today, for the rig's package and file from `RIGS`:

1. `get_package_share_directory(package)` + `config/<file>`, when the workspace is sourced.
2. Otherwise the repository's copy, two levels up from this file:
   `<repo>/<package>/config/<file>`.

### `read_block(rig, name, config_path)`

1. `yaml.safe_load` the file.
2. Key `f'/{prefix}/{name}/controller'`, with `prefix` from `RIGS`.
3. Missing: raise `ValueError` listing the names in the file
   (`k.split('/')[2]` for keys starting with `f'/{prefix}/'`), as `generate_waves` does now.
4. Return `config[key]['ros__parameters']`.

### `limits_from_parameters(rig, name, params, source)`

The one place with the rules from the table above. Both the yaml and the live controller go
through it, so they cannot be derived differently.

1. `drive = required(params, 'drive_name', name, source)`; `upright_m = params.get('upright_position_m', 0.0)`.
2. If `params.get('drive_motion', 'linear') == 'angular'`:
   - `lead = required(... 'actuator_lead_m_per_degree' ...)`; refuse `lead <= 0` with a
     `ValueError` (the controller refuses it too).
   - `upright_deg = params.get('actuator_upright_angle_deg', 0.0)`.
   - `min_m`, `max_m` from `min_position_deg`, `max_position_deg` (both required).
   - `max_velocity_mps = required(... 'max_velocity_rpm' ...) * 6.0 * lead`.
3. Otherwise:
   - `min_m`, `max_m` = `travel_minimum_m`, `travel_maximum_m` (both required).
   - If `drive == 'linmot'`: `max_velocity_mps = params.get('max_velocity_mps', 2.0)`.
   - Else: `max_velocity_mps = None`.
4. Acceleration:
   - `goal = params.get('max_goal_acceleration_mps2', 0.0)`; `None` if `goal <= 0`.
   - If `drive == 'linmot'`: `drive_limit = params.get('max_acceleration_mps2', 20.0)` and the
     limit is `drive_limit` if `goal` is `None`, else `min(goal, drive_limit)`.
   - Else the limit is `goal`.
5. Refuse `min_m >= max_m` and an upright outside `min_m..max_m` with a `ValueError`, as the
   controller refuses to configure with them.
6. Return `RigLimits(rig, name, f'/{prefix}/{name}', upright_m, min_m, max_m, max_velocity_mps,
   max_acceleration_mps2, source)`.

### `load_limits(rig, name, config_path=None)`

1. `config_path = config_path or default_config_path(rig)`.
2. `params = read_block(rig, name, config_path)`.
3. Return `limits_from_parameters(rig, name, params, os.path.basename(config_path))`.

### `compare(expected, actual)`

Returns one line per difference, empty when they agree:

- Positions (`min_m`, `max_m`, `upright_m`): differ when `abs(a - b) > 1e-4` (0.1 mm).
  Line: `f'max_m: {expected.source} {e:+.5f} m, {actual.source} {a:+.5f} m'`.
- `max_velocity_mps`: compare only if both are not `None`; differ when `abs(a - b) > 1e-3`.
- `max_acceleration_mps2`: differ when one is `None` and the other not, or both set and
  `abs(a - b) > 1e-3`.

---

## 2. `play_dat.py`

### Arguments

Changes to `main()` (today ~353-375):

```python
parser.add_argument('file')
parser.add_argument('name', nargs='?',
                    help='rig name in the yaml (ladertanken, sloshing); gives the limits, upright '
                         'and namespace')
parser.add_argument('--rig', default='wavemaker', choices=('wavemaker', 'oscillator'),
                    help='action server move_<rig> and the yaml; default wavemaker')
parser.add_argument('--config', help='another yaml; default the installed one')
parser.add_argument('--upright-m', type=float, help='override; default from the yaml, else 0.0')
parser.add_argument('--min-m', type=float, help='override; required without a name')
parser.add_argument('--max-m', type=float, help='override; required without a name')
parser.add_argument('--max-velocity-mps', type=float, help='override; required without a name')
```

`--upright-m`, `--min-m`, `--max-m` and `--max-velocity-mps` lose `required=True` and their
defaults (`None` means "not given").

### Resolving the limits

A new function, called in `main()` straight after parsing and before reading the file:

```python
def resolve_limits(args) -> tuple[RigLimits | None, dict]:
```

1. If `args.name`: `limits = rig_limits.load_limits(args.rig, args.name, args.config)`; a
   `ValueError` goes to the same `error: ...` / return 2 path as a bad file.
2. Else `limits = None`.
3. For each of `upright_m`, `min_m`, `max_m`, `max_velocity_mps`: the command-line value if
   given, else the yaml's (`limits.<field>`), else, for `upright_m` only, 0.0 (the old default).
4. If `min_m`, `max_m` or `max_velocity_mps` is still `None`:
   - without a name: `parser.error('give the rig name, or --min-m, --max-m and --max-velocity-mps')`;
   - with a name, only `max_velocity_mps` can be missing (a linear IndraDrive):
     `parser.error(f'{name} has no speed limit in the yaml (TODO item 19): give --max-velocity-mps')`.
5. Print one line, before the run table:
   ```
   limits from wavemakers.yaml (ladertanken): -0.14575..+0.14575 m, 0.7632 m/s, upright +0.0000 m
   limits from wavemakers.yaml (ladertanken), --max-m overridden: -0.14575..+0.10000 m, ...
   limits from the command line: -0.10600..+0.10600 m, 0.5400 m/s, upright +0.0000 m
   ```
6. Return `limits` (for the cross-check) and the resolved values (for `check_run`).

The calls to `split_runs`, `whole_file_run` and `check_run` then use the resolved `upright_m`,
`min_m`, `max_m` and `max_velocity_mps` in place of `args.*`.

### Namespace

`play()` receives `argv` and passes it to `rclpy.init(args=ros_args)` (~293). Before that, in
`main()`:

- If a name was given and `argv` has no `__ns:=` anywhere, append
  `['--ros-args', '-r', f'__ns:={limits.namespace}']` to the list handed to `play()`.
  (Appending a second `--ros-args` block is allowed; rclpy reads all of them.)
- An explicit `__ns` remap is left alone, so it still wins.
- Print `namespace /wavemakers/ladertanken` with the limits line, so a wrong rig shows before any
  goal.

### Live cross-check

In `play()`, after `node = rclpy.create_node('play_dat')` and the action server is found, before
the first goal. Only when `limits` is not `None` (a name was given).

The parameters to ask for: every key `limits_from_parameters` reads.

```python
LIMIT_PARAMETERS = [
    'drive_name', 'drive_motion', 'upright_position_m', 'actuator_lead_m_per_degree',
    'actuator_upright_angle_deg', 'min_position_deg', 'max_position_deg', 'max_velocity_rpm',
    'travel_minimum_m', 'travel_maximum_m', 'max_velocity_mps', 'max_acceleration_mps2',
    'max_goal_acceleration_mps2',
]
```

Getting them with rclpy (Jazzy: `rclpy.parameter_client.AsyncParameterClient`):

```python
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

client = AsyncParameterClient(node, f'{limits.namespace}/controller')
if not client.wait_for_services(timeout_sec=2.0):
    ...  # warn: not cross-checked
future = client.get_parameters(LIMIT_PARAMETERS)
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
values = future.result().values if future.done() and future.result() else None
params = {key: parameter_value_to_python(value)
          for key, value in zip(LIMIT_PARAMETERS, values)
          if value.type != 0}   # 0 = PARAMETER_NOT_SET: not declared on this controller
```

Then:

1. `live = rig_limits.limits_from_parameters(rig, name, params, 'controller')`. A `ValueError`
   here (a parameter missing on the controller) is a warning, not a stop.
2. For each line of `rig_limits.compare(limits, live)`: `node.get_logger().warn(f'yaml and controller differ: {line}')`.
3. No answer within 2 s (services missing, controller unconfigured):
   `warn('controller parameters not available; limits not cross-checked')` and carry on.
4. Agreement: `info('limits match the controller')`.

The controller declares its parameters at construction, so the values are there even before
`configure`. They are the values it would configure with, which is what matters.

---

## 3. `generate_waves.py`

- Delete `required()`; `import rig_limits` and use `rig_limits.required` for the geometry keys
  (`wavemaker_type`, `water_depth`, `flap_attachment_height`). The message gains the source:
  `ladertanken: water_depth not set in wavemakers.yaml`.
- `load_wavemaker` keeps reading the geometry and gain from the block, and takes the limits from
  `rig_limits.limits_from_parameters('wavemaker', name, params, ...)`; `Wavemaker` keeps its
  `min_m`, `max_m`, `upright_m`, `max_velocity_mps`, `max_acceleration_mps2` fields, filled from
  the `RigLimits`. Nothing else in the file changes.
- `default_config_path()` becomes `rig_limits.default_config_path('wavemaker')`.
- `main()` prints the short `play_dat` command:
  `ros2 run rig_controller play_dat session_waves.dat ladertanken --dry-run`; for a linear drive
  add `--max-velocity-mps <the value used>`.

## 4. `CMakeLists.txt`

The `install(FILES ...)` added for `generate_waves` gains the new module:

```cmake
# Imported by play_dat and generate_waves; the programs are installed without .py.
install(FILES src/play_dat.py src/rig_limits.py
  DESTINATION lib/${PROJECT_NAME}
)
```

## 5. Tests

### `test_rig_limits.py` (new)

Loaded like `test_generate_waves.py` (src on `sys.path`). Config paths point at the repository's
yamls; synthetic yamls are written to `tmp_path`.

| Test | Expect |
|---|---|
| ladertanken from `wavemakers.yaml` | the values in the table in section 1, `source == 'wavemakers.yaml'` |
| lilletanken | 0.0..0.5 m, `max_velocity_mps is None` |
| sloshing from `oscillator.yaml` | 0.015..0.185 m, 2.0 m/s, 9.81 m/s^2, upright 0.1, `/oscillators/sloshing` |
| LinMot without `max_goal_acceleration_mps2` | acceleration 20.0 (the drive's) |
| missing `min_position_deg` (angular) | `ValueError` naming the key and the file |
| `lead <= 0`, `min_m >= max_m`, upright outside the travel | `ValueError` each |
| unknown name | `ValueError` listing `ladertanken` |
| `limits_from_parameters` with the yaml's block as a dict | equal to `load_limits` |
| `compare`, equal limits | `[]` |
| `compare`, `max_position_deg` 275 vs 250 | one line naming `max_m` |
| `compare`, speed `None` vs 600 (linear IndraDrive live) | `[]` |
| `compare`, acceleration `None` vs 9.81 | one line |

### `test_play_dat.py` (new cases)

`main([...])` with `--dry-run` on a file from `make_test_dat` style helpers already in the file.

| Test | Expect |
|---|---|
| file + `ladertanken` | exit 0; printed line `limits from wavemakers.yaml (ladertanken)` with ±0.14575 |
| file + `ladertanken --max-m 0.1` | line says `--max-m overridden`; a run beyond 0.1 m is flagged |
| file, no name, no limits | exit 2 (argparse error) with "give the rig name" |
| file, no name, all three limits | exit 0; `limits from the command line`; old behaviour |
| file + `lilletanken` without `--max-velocity-mps` | argparse error naming item 19 |
| file + `sloshing --rig oscillator` | upright 0.1 used: runs relative to 0.1 |

The live cross-check is not covered by these: it needs a node. Test its parts instead:
`compare` (above) and the parameter dict to `RigLimits` path (`limits_from_parameters` with a
dict shaped like the client's result). For an end-to-end check, `fake_controller.py` would have
to declare the parameters in `LIMIT_PARAMETERS`; today it declares none, so `play_dat` against it
prints "not cross-checked", which is the right behaviour to check by hand.

### `test_generate_waves.py`

- `test_missing_required_key_is_named`: the message now includes the file name; the match on the
  key still passes.
- Add: the printed `play_dat` command in `test_main_end_to_end` contains `ladertanken --dry-run`.

## 6. Documentation

- `play_dat.py` docstring: the examples in the short form, plus one line on the old form and the
  overrides.
- README *Generating motion files*, *Check and play*: the short form.
- `rpi/README.md`, rig test *Goal type per goal and `play_dat`*: can stay as it is (the old form
  works); move to the short form when it is next run.

## Out of scope

- The water depth: `play_dat` does not need it (positions only). Whether a file was made for the
  current depth is item 39's record.
- The trigger channel (planned feature *Trigger output*).
- Making `fake_controller` declare the limit parameters: only needed for an automated end-to-end
  test of the cross-check.
