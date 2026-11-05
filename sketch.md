# Sketch: `generate_waves` records how a file was made, and two follow-ups of item 40 (TODO items 39, 41, 42)

The previous sketch (`play_dat` takes its limits from the yaml, item 40) is done and in the git
history (`git show b298c13:sketch.md`).

## Problem

- **39.** A motion file is only right for the depth, yaml and code it was made with, and a
  pregenerated goal carries positions only, so neither `play_dat` nor the controller can tell.
  Item 38's `item38.dat` (2026-11-05) is already an example: made at 0.98 m with a gain table for
  1.0 m, and nothing next to it says so. The probe test (item 43) needs this record.
- **41.** `--water-depth` equal to the yaml's depth still warns: "the gain table is for 1.0 m, not
  1.0 m".
- **42.** Without the bringup package installed, `rig_limits.default_config_path` falls back to a
  path inside `install/rig_controller/` that does not exist, and the error does not say why.
  `generate_waves` looks the default up while building its parser, so it fails even with
  `--config`.

## Decisions (2026-11-05)

1. **The record is `<output>.txt`**, next to the `.dat`, `.csv` and `.png`: plain `key value`
   lines, then the test matrix as it was. The matrix is copied because `matrix.csv` is often edited
   or overwritten afterwards; with it, the file can be made again.
2. **The commit is the repository's at generation time** (`git describe --always --dirty`).
   An installed copy has no repository next to it, so `CMakeLists.txt` installs `source_dir.txt`
   with the source path. This shows what was checked out, not proof that it was built: after a
   `git pull` without `colcon build`, the code that ran is older. Without git, `unknown`.
3. **Item 42 differs from the sketch in TODO.md.** That sketch falls back to the repository only
   on `ImportError`, but `ament_index_python` imports whenever `/opt/ros/jazzy` is sourced, which
   it is in every shell on the PC and the Pi. Running the tests with only ROS sourced, or with the
   workspace sourced but built without the bringup packages, would then fail. Instead: fall back
   whenever the package is not found, and raise only when the fallback does not exist either (an
   installed copy).
4. **`generate_waves` prints `--config` whenever it was given**, instead of comparing it with the
   default. The comparison called `default_config_path()`, which can now raise. Giving the default
   path explicitly then prints a `--config` that is not needed but does no harm.

## Files

| File | Change | Section |
|---|---|---|
| `rig_controller/src/rig_limits.py` | `default_config_path`: narrower except, error when nothing is found | 1 |
| `rig_controller/src/generate_waves.py` | `--config` default `None`; the warning; `source_commit`, `write_record` | 2 |
| `rig_controller/CMakeLists.txt` | Install `source_dir.txt` | 3 |
| `rig_controller/test/test_rig_limits.py`, `test_generate_waves.py` | 8 new tests (already in the repository) | 4 |
| README, `rpi/README.md`, TODO.md | The `.txt`, the short `play_dat` form, close 39, 41, 42 | 5 |

Checked in a copy of the repository (2026-11-05): with sections 1-3 applied, all 171 tests pass.

---

## 1. `rig_limits.py`: `default_config_path` (item 42)

Replace the function (today line 43):

```python
def default_config_path(rig: str) -> str:
    """The installed yaml, else the repository's copy when running from the source tree."""
    _, package, filename = RIGS[rig]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
    except (ImportError, LookupError):  # ROS not sourced, or the package not installed
        share = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', package)
        if not os.path.isdir(share):  # an installed copy: no repository next to it
            raise FileNotFoundError(
                f'{package} is not installed, so there is no {filename}: build it '
                f'(colcon build --packages-up-to {package}) or give --config') from None
    return os.path.join(share, 'config', filename)
```

`PackageNotFoundError` is a `LookupError`. `except Exception` would also have hidden a real error
in `ament_index_python`. `play_dat` catches the `FileNotFoundError` as an `OSError` (exit 2), and
`fake_controller` does too.

---

## 2. `generate_waves.py`

### 2.1 Docstring and imports

The first lines of the module docstring:

```python
"""
Generate wave motion files (.dat, run table .csv, record .txt) for one wavemaker, using its
geometry and limits from wavemakers.yaml. The goal is compatibility: every file plays with
play_dat.py and every run is accepted by the controller on that wavemaker.
```

Add, in order with the others:

```python
from datetime import datetime
import shlex
import subprocess
```

### 2.2 New functions (item 39)

Put them after `write_run_table` (today ~467-477):

```python
def source_commit() -> str:
    """The repository's commit, with '-dirty' for local changes; 'unknown' without git."""
    here = os.path.dirname(os.path.realpath(__file__))
    try:  # an installed copy: CMakeLists.txt writes where the repository is
        with open(os.path.join(here, 'source_dir.txt')) as f:
            here = f.read().strip()
    except OSError:
        pass  # running from the source tree
    try:
        result = subprocess.run(['git', '-C', here, 'describe', '--always', '--dirty',
                                 '--exclude=*'],
                                capture_output=True, text=True, timeout=5, check=True)
    except (OSError, subprocess.SubprocessError):
        return 'unknown'
    return result.stdout.strip()


def write_record(path: str, argv: list[str], args, wm: Wavemaker, yaml_depth: float,
                 config: str, max_velocity_mps: float, summary: str):
    """<output>.txt: what the file was made from, to read probe data and to make it again."""
    if args.water_depth is None:
        depth = f'{wm.water_depth} m (the yaml)'
    else:
        depth = f'{wm.water_depth} m (--water-depth; the yaml has {yaml_depth} m)'
    speed = f'{max_velocity_mps:.4f} m/s'
    if args.max_velocity_mps:
        speed += ' (--max-velocity-mps)'
    with open(args.test_matrix) as f:
        matrix = f.read().rstrip('\n')
    lines = [
        f'generated    {datetime.now().astimezone().isoformat(timespec="seconds")}',
        f'command      generate_waves {shlex.join(argv)}',
        f'folder       {os.getcwd()}',
        f'commit       {source_commit()}',
        f'config       {os.path.realpath(config)}',
        f'wavemaker    {wm.name} ({wm.type})',
        f'water depth  {depth}',
        f'dt           {args.dt} s',
        f'max speed    {speed}',
        f'result       {summary}',
        '',
        f'test matrix {args.test_matrix}:',
        matrix,
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
```

- `--exclude=*` keeps `describe` to the bare commit, even if the repository gets tags.
- `realpath` resolves the installed yaml to a full path.
- `folder` makes a relative `-o` or test matrix in `command` readable later.

### 2.3 `main()`

The `-o` and `--config` arguments (today ~543-544):

```python
    parser.add_argument('-o', '--output', required=True,
                        help='base name for .dat, .csv, .txt and .png')
    parser.add_argument('--config', help='another wavemakers.yaml; default the installed one')
```

After the arguments, the command line for the record, then the config path (item 42) and the
warning (item 41). Today ~550-558 becomes:

```python
    argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(argv)
    if not os.path.isfile(args.test_matrix):
        parser.error(f'test matrix {args.test_matrix} not found (relative to {os.getcwd()})')
    try:
        config = args.config or default_config_path()
    except FileNotFoundError as e:
        parser.error(str(e))

    wm = load_wavemaker(args.wavemaker, config)
    yaml_depth = wm.water_depth
    if args.water_depth is not None:
        if wm.gain_periods and args.water_depth != wm.water_depth:
            print(f'warning: the gain table is for {wm.water_depth} m, not {args.water_depth} m')
        wm.water_depth = args.water_depth
```

The `!=` compares two floats parsed from text (`1.0` in the yaml, `1.0` on the command line), so
exact equality is what is meant.

After `verify_with_play_dat` (today ~567-571): the summary once, the record before the plot. If the
plot fails, as it did on the Pi without matplotlib, the record is still written:

```python
    played = verify_with_play_dat(dat_path, wm, max_velocity)
    summary = (f'{played} runs, {len(rows) - played} skipped, '
               f'{len(frames) * args.dt / 60:.1f} min')
    write_record(args.output + '.txt', argv, args, wm, yaml_depth, config, max_velocity, summary)
    if not args.no_plot:
        plot_motion(args.output + '.png', wm, frames, rows, args.dt, max_velocity, args.show)
    print(f'{dat_path}: {summary}')
```

The `--config` in the printed command (today ~575):

```python
    if args.config is not None:
        command += f' --config {args.config}'
```

### What it writes

From the copy, `generate_waves ladertanken m.csv -o demo --water-depth 0.98 --no-plot --config …`:

```
generated    2026-11-05T10:05:51+01:00
command      generate_waves ladertanken m.csv -o demo --water-depth 0.98 --no-plot --config repo/wavemaker_bringup/config/wavemakers.yaml
folder       /tmp/…/scratchpad
commit       5e85a0a-dirty
config       /tmp/…/scratchpad/repo/wavemaker_bringup/config/wavemakers.yaml
wavemaker    ladertanken (flap)
water depth  0.98 m (--water-depth; the yaml has 1.0 m)
dt           0.02 s
max speed    0.7632 m/s
result       1 runs, 0 skipped, 0.3 min

test matrix m.csv:
kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,1.0,,1/60,10,,,,,,2,5
```

---

## 3. `CMakeLists.txt`

Before the `install(FILES src/play_dat.py src/rig_limits.py ...)` block (today line 71):

```cmake
# generate_waves records the repository's commit (source_commit); an installed copy finds the
# repository through this file.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt "${CMAKE_CURRENT_SOURCE_DIR}\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt
  DESTINATION lib/${PROJECT_NAME}
)
```

`file(WRITE)` runs at configure time, which `colcon build` does not repeat. That is fine: the path
does not change, and the commit is read when `generate_waves` runs. Checked with a minimal project:
the installed file holds the source path.

---

## 4. Tests

Already in the repository, written before the code, so 6 fail until sections 1-2 are in. Against
today's code: 165 pass, 6 fail. With the sketch applied to a copy: 171 pass.

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test -q
```

| Test | Item | Fails today |
|---|---|---|
| `test_rig_limits.py::test_default_config_path_falls_back_to_the_repository` | 42 | no (keeps the fallback) |
| `test_rig_limits.py::test_default_config_path_installed_without_the_package` | 42 | yes |
| `test_generate_waves.py::test_gain_table_warning_only_at_another_depth[1.0-False]` | 41 | yes |
| `test_generate_waves.py::test_gain_table_warning_only_at_another_depth[0.98-True]` | 41 | no (keeps the warning) |
| `test_generate_waves.py::test_config_given_works_without_the_package` | 42 | yes |
| `test_generate_waves.py::test_record_next_to_the_file` | 39 | yes |
| `test_generate_waves.py::test_record_depth_from_the_yaml` | 39 | yes |
| `test_generate_waves.py::test_source_commit_of_an_installed_copy` | 39 | yes |

The two `default_config_path` tests patch `get_package_share_directory` and skip without
`ament_index_python`. `test_source_commit_of_an_installed_copy` places `source_dir.txt` in a
temporary folder and points it at this repository.

On the Pi, after `colcon build`, check by hand that the installed copy finds the repository:
`ls ~/ros_ws/install/rig_controller/lib/rig_controller/source_dir.txt`, then one
`generate_waves` run with `commit` showing a hash in its `.txt`, not `unknown`.

---

## 5. Documentation

- **README *Generating motion files*:**
  - Under *It writes*, add `session_waves.txt`: how the file was made (date, command, commit, yaml,
    water depth, dt, speed limit) and a copy of the test matrix. Keep it with the `.dat`, as it is
    what the probe data is read against.
  - The `--config` row: "default the installed one, else the repository's when running from the
    source tree".
  - The `--water-depth` row: "warns when the gain table was measured at another depth" is now true.
- **`rpi/README.md`, rig test *Goal type per goal and `play_dat`*, steps 3-4:** the short form
  (`play_dat /tmp/test_runs.dat ladertanken --dry-run`, and `--runs 1` without `--ros-args`), as
  the previous sketch planned for when that test is next run. The expected output then also has
  the `limits from wavemakers.yaml (ladertanken)` and `namespace` lines, and the 0.54 m/s becomes
  the yaml's 0.7632 m/s. The six runs (peak 0.157 m/s) pass either way.
- **TODO.md:**
  - Remove items 39, 41 and 42.
  - In item 43, note that `item38.dat` has no `.txt`. Generating it again for the probe test makes
    one, and it goes with the probe data.

## Out of scope

- A check in `play_dat` that the file matches today's depth: `play_dat` cannot know the depth the
  file was made for unless it reads the `.txt`. That is possible later, now that the `.txt` exists.
- The 60 s wait before `--runs N`, the slow service stop and the soft stop gaps seen on 2026-11-05.
  These are separate TODO items.
