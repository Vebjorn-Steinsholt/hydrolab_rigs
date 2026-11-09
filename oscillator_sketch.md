# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

Line numbers are as of commit `6b994c7` plus the uncommitted work on `bugfix-items-20-25`, except
*Your tasks* and *Task details*: commit `cecf951` on `feat-vertical-oscillator` (2026-11-09).

## Scope (2026-11-06)

- **Phase 1, now: replugging, for developing and testing the code only.** It lasts until the
  daisy-chain cable is delivered. The oscillator drive keeps PROFIBUS address **2**, as the
  wavemaker drive does, so its existing setup is untouched. Only one of the two drives is on
  the gateway's PROFIBUS cable at a time.
- **Phase 2, when the cable arrives: daisy chain**
  ([Phase 2: daisy chain](#phase-2-daisy-chain-when-the-cable-arrives)). Both drives stay on the
  line, so one of them needs a different address. One rig still runs at a time. Running both at
  once is deferred beyond that.
- **Phase 1 work carries over.** Everything that differs between the phases is in the yaml
  (blocks, PROFIBUS address), so the change is a config edit plus the phase 2 tasks, not a
  rewrite.
- **The oscillator is developed by playing motion files** (`play_dat --rig oscillator`).
- **You write the code.** Each task says what to build, where, and how to check it. Claude
  writes tests and docs when you hand them over.

## Goal

A second Rexroth IndraDrive C (rotary motor, linear drive scaling) runs a vertical oscillator.
Connect it to the ladertanken MGate (`192.168.1.70`) in place of the wavemaker drive and run it
from the ladertanken Pi, playing motion files, with the same safety behaviour as the wavemaker.

## The oscillator drive (read 2026-11-06)

| | Value | Meaning for the code |
|---|---|---|
| PROFIBUS address | 2 | Same as the wavemaker: the gateway's slave 2, blocks 0/0, unchanged |
| Scaling type | linear (the drive converts motor turns to mm) | `drive_motion: linear` |
| Position resolution | 0.0001 mm | Tag factor 0.0001 → the "deg" tag is **mm** |
| Velocity resolution | 0.001 mm/min | Tag factor 0.0001 → the "rpm" tag is **10 mm/min** per unit (1 mm/s = 6 units) |
| Acceleration resolution | 0.001 mm/s² | Not in the cyclic data; the drive's own positioning acceleration applies |
| Torque/force resolution | 0.1 % | Not in the cyclic data |
| Position limits | −211 … +211 mm | The drive's backstop; the controller's travel goes inside it |
| Current position | about 0 mm | Mid-travel |
| Direction (measured 2026-11-09) | **+ command = down** | The rig uses + = up, so T2 flips the sign: `drive_direction: -1` (decision 4) |
| Scaling check (2026-11-09) | `--move` +5: drive 4.925 mm, sensor 4.940 mm at 1.68 mm/s | `drive_m_per_unit: 0.001` and `drive_velocity_per_unit_per_s: 6.0` confirmed (see *PROFIBUS diagnosis log*, first move) |
| Power section | Needs mains switched on at the cabinet (button) | Without it the drive stays at A0013 (`bb`) and `enable()` times out waiting for `Ab` |

From the parameter file `Default.par` (IndraWorks export 2026-11-06 10:31), compared with the
wavemaker's `WMDefault.par`:

| | Oscillator | Wavemaker | Meaning |
|---|---|---|---|
| Hardware | FWA MPB-**05**V20, HCS02.1E-W0028, MSK050C-0300 (`-UG1`, brake), CSB01.1**N**-PB | MPB-04V24, HCS03.1E-W0070, MSK071E-0450 (`-UG0`), CSB01.1C-PB | **MPx-05 needs `RX060107.gsd`** (Fail_Safe): the repo's GSD files only cover up to MPB-04. See *PROFIBUS diagnosis log* |
| P-0-4084 profile | 0xFFFE | 0xFFFE | ✓ |
| S-0-0032 operating mode | 0x001B | 0x001B | ✓ |
| P-0-4080 inputs | P-0-4078, **S-0-0051**, S-0-0040, S-0-0390 (14 bytes) | P-0-4078, S-0-0386, S-0-0040, S-0-0390 | ✓ length. S-0-0051 is the motor encoder position, the same as S-0-0386 with one encoder |
| P-0-4081 outputs | P-0-4077, S-0-0258, S-0-0259, S-0-0108 (12 bytes) | the same | ✓ |
| P-0-4088 master communication config | 0x002A | not in the file (default 0) | Bits 15/14 = 00: commands from PROFIBUS. **Bits 2/1 = 01: a bus failure is only a warning (E4xxx) and the last command is kept** (Reference Book R911297317-08, p. 1335). See *PROFIBUS diagnosis log* |
| P-0-4076 fieldbus cycle time | 1000 µs | 2000 µs | — |
| PROFIBUS address (P-0-4025) | **2** (read on the drive) | not in the file | P-0-4031 = 0x00020002 in both files |
| S-0-0076/77/78 position scaling | linear, 1 × 10⁻⁷ m = 0.0001 mm | rotary, 0.0001° | `drive_m_per_unit: 0.001` |
| S-0-0044/45/46 velocity scaling | linear, 1 × 10⁻⁶ m/min = 0.001 mm/min | rotary, 0.0001 rpm | `drive_velocity_per_unit_per_s: 6.0` |
| S-0-0160/161/162 acceleration scaling | linear, 0.001 mm/s² | rotary, 0.001 rad/s² | — |
| S-0-0123 feed constant | 20 mm/rev | 10 °/rev | — |
| S-0-0049/50 limits | +211 / −211 mm | +500 / −550° | — |
| S-0-0057 position window | 0.1 mm | | `in_position_tol_deg: 0.1` |
| S-0-0055 position polarities | 0x1F (all inverted) | 0x10 | Result: + = down. Left unchanged; the sign is flipped in the controller (`drive_direction: -1`) |
| S-0-0091 velocity limit | 96 000 mm/min (1.6 m/s) | | — |
| S-0-0260 positioning acceleration | 20 000 mm/s² (20 m/s²) | 1000 rad/s² | — |
| S-0-0193 positioning jerk | **3000 mm/s³ (3 m/s³)** | 10 000 rad/s³ | Expected to limit the oscillation; below 3 m/s³ it did not (decision 5) |
| P-0-0525 / P-0-0540 holding brake | 0x0004 / **5 Nm** | 0x0000 / 0 Nm | Brake configured; about 1.5 kN at 20 mm/rev |
| P-0-0119 best possible deceleration | 0 | 0 | Check its meaning for a vertical axis |
| S-0-0273 max drive-off delay | 10 000 ms | | — |
| S-0-0108 feedrate override (stored) | 0 % | | `enable()` writes 100 % (`indradrive.cpp:197`) |

## How the code is built today (what decides the design)

- Each controller node owns its own gateway connection: `IndraDriveActuator` creates one
  `MGateDriver` and one `IndraDrive` (`rig_controller/src/indradrive_actuator.cpp:138-139`).
  The blocks, sizes and status area are parameters; ladertanken uses 0/7 in, 0/6 out, status 768/9.
- `IndraDrive` assumes a fixed cyclic layout (`indradrive.h:1-15`): P-0-4084 = 0xFFFE,
  drive-internal interpolation (S-0-0032), and P-0-4080/4081 = status, position, velocity, diag
  in; control, target, speed cap, override out. The tags carry the factor 0.0001 and call the
  units "deg" and "rpm" (`indradrive.cpp:27-33`).
- **Linear IndraDrive scaling (TODO item 19)** was unfinished: a 0.1 m goal became 0.1 mm and
  the speed cap was 36 × too low. Fixed by T2 (2026-11-09): `drive_m_per_unit`,
  `drive_velocity_per_unit_per_s`, `drive_direction`, limits from travel.
- **Only one rig runs on the Pi at a time.** `wavemaker@` and `oscillator@` refuse to start while
  the other runs (`rpi/refuse_if_busy.sh`, which also catches a launch by hand through the web
  port), and `windows/_rig.bat` stops the other before starting one. Since T5 (2026-11-09) the
  oscillator launch takes `oscillator:=sloshing|vertical`, set through `OSCILLATOR=` in
  `/etc/default/oscillator`.
- **`play_dat` already plays to an oscillator** (`--rig oscillator`, action `move_oscillator`).
  With a rig name it reads limits and namespace from `oscillator.yaml` (`rig_limits.py:20`).
  For a linear IndraDrive, `rig_limits` gives no speed limit (`rig_limits.py:87-92`), so
  `--max-velocity-mps` is required until T8. `generate_waves` is wavemakers only; the
  oscillator's generator is T7 (`generate_oscillations`).
- **`mgate_example` works on the oscillator as it is**, because blocks 0/0 and address 2 are
  the same (`drivers/mgate5101_driver/examples/example_main.cpp`). `--move` steps +5 and back
  at 10: on this drive **5 mm at 100 mm/min (1.7 mm/s)**, about 3 s each way. Its limits default
  to −550…500 (`IndraDriveConfig`), outside ±211 mm, so the drive's limits are the only
  backstop; that is fine for a 5 mm step from about 0. It is built but not installed
  (`~/ros_ws/build/mgate5101_driver/mgate_example`).

## Decisions

All implemented on 2026-11-09 except where noted.


1. **Phase 1: replug, don't daisy-chain, while the address stays 2.** This is temporary, until
   the daisy-chain cable is delivered. Two slaves with one address on one line would collide. The gateway, `wavemakers.yaml` and the oscillator drive stay as they are.
   **The risk:** nothing in the cyclic data tells the controller which drive is connected.
   - The wavemaker's yaml on the oscillator would treat mm as flap degrees (its travel ±275
     "deg" = ±275 mm, beyond the ±211 mm limits).
   - The oscillator's yaml on the wavemaker would treat flap degrees as mm.
   - So the order is: stop the service, replug, then check before enabling. Stop the running
     service (`systemctl stop`), replug, label the plug, and check the monitor output against
     the drive you expect before starting the matching service (Commissioning, step 0).
2. **The oscillator drive must speak the layout `IndraDrive` expects.** It sits on the gateway's
   slave 2, whose modules are 7 words in / 6 words out. Check its P-0-4084, P-0-4080/4081 and
   S-0-0032 against `indradrive.h:1-15`, **read only**. If the lengths differ, the drive does
   not enter data exchange (live bit 0), which is safe: nothing moves. If the lists differ, the
   tags read the wrong values. Either way, decide then whether to change the drive's lists
   (which touches its existing setup) or make the tag table configurable.
3. **Use the existing `rig: oscillator` path with `drive_name: indradrive`.** No new actuator
   class. Check that nothing assumes `rig: oscillator` means LinMot.
4. **Linear drive scaling (TODO item 19) is done now,** with two parameters and defaults that
   keep today's angular behaviour:
   - `drive_m_per_unit`: metres per position tag unit. **0.001** here (0.0001 mm resolution ×
     tag factor 10 000 = 1 mm). It goes into `DriveScale::m_per_unit` in the linear branch,
     which is fixed at 1 today.
   - `drive_velocity_per_unit_per_s`: velocity tag units per (position tag unit per second).
     **6.0** here (1 mm/s = 60 mm/min = 6 × 10 mm/min). Angular default **1/6** (deg/s → rpm).
     It replaces the `/ 6` in `write_setpoint()` and the `* 6.0` in `max_velocity_mps()`.
   - `drive_direction`: +1 or −1, the sign between rig coordinates (+ = up) and drive
     coordinates. **−1 here** (+ command moves down, measured 2026-11-09); default +1 keeps
     ladertanken unchanged. It applies to setpoints, position feedback and the drive limits
     (min and max swap on the drive side). The velocity cap and `max_velocity_mps()` use
     magnitudes only, so the cap stays positive. The drive's S-0-0055 stays as it is.
   - The drive limits come from `travel_minimum_m/maximum_m` (one set of limits, not two).
     `max_velocity_rpm` / `hold_velocity_rpm` stay in velocity tag units (here 10 mm/min each;
     0.1 m/s = 600). The ±211 mm in the drive stay as the backstop.
5. **Vertical axis: gravity, brake, acceleration and jerk.** Control word 0 (our fault path,
   and the gateway fault value) removes torque.
   - **Brake:** the motor has a 5 Nm holding brake, and the drive controls it (P-0-0525 =
     0x0004, P-0-0540; motor `-UG1`). At 20 mm/rev that holds about 1.5 kN. It held at every
     switch-off on 2026-11-09, including the kill test (unloaded). Still open: whether the stop
     reaction (P-0-0119 = 0) brakes the slide before the brake engages under load, and where the
     slide rests when disabled.
   - **The drive limits every move.** The controller sends no acceleration
     (`max_acceleration_mps2()` is infinite), so the drive's positioning acceleration (S-0-0260
     = 20 m/s²) and **jerk (S-0-0193 = 3 m/s³)** do.
   - **Jerk was expected to be the binding limit.** A sine of stroke `s` and period `T` needs acceleration
     `(s/2)(2π/T)²` and jerk `(s/2)(2π/T)³`. Examples:
     - 20 mm at 1 s: 0.39 m/s², 2.5 m/s³.
     - 20 mm at 0.8 s: 0.62 m/s², 4.8 m/s³.
     - 50 mm at 2 s: 0.25 m/s², 0.78 m/s³.

     Above 3 m/s³ the drive rounds the motion off. **Measured 2026-11-09 (unloaded): below 3 m/s³
     the stroke loss does not depend on the jerk** (20 mm and 10 mm at 1 s both 89.7 %); it
     depends on the period (*Commissioning*, waves 3-5; *Open questions*, tracking). Keep files
     under 3 m/s³ anyway: `generate_oscillations` warns above it. Raising S-0-0193 is for the drive
     tuning with Robert.
6. **One rig at a time stays enforced by the Pi services.** The vertical oscillator runs as
   `oscillator@ladertanken`, which cannot start while `wavemaker@ladertanken` runs. The
   oscillator launch gets an `oscillator` argument (`sloshing` or `vertical`), so the LinMot rig
   keeps working.
7. **Files are played with `play_dat` as they are,** made by `generate_oscillations` (T7, built
   like `generate_waves`). Pass `--max-velocity-mps` until T8. Single waves also run from the web
   page (height = stroke, period).

## Your tasks

| # | File | Where | What | Done when | Left (2026-11-09) |
|---|---|---|---|---|---|
| T5 | the ladertanken Pi | — | Decision 6: the last checks | Refused while the wavemaker runs; the sloshing rig still starts | Two checks at the next rig switch. **Decide: switching between sloshing and vertical** |
| T7 | new `rig_controller/src/generate_oscillations.py` and `motion_files.py` | the *Oscillator motion generator* | A test-matrix generator built like `generate_waves` (spec below) | `play_dat FILE vertical --rig oscillator --dry-run` lists every run as `OK` | All: step 1 `motion_files.py`, step 2 the script |
| T8 | `rig_controller/src/rig_limits.py`, `wavemaker_bringup/config/wavemakers.yaml` | `limits_from_parameters` linear branch (87-92); lilletanken and mclab blocks | Speed limit for a linear IndraDrive from T2's parameters: `max_velocity_rpm / drive_velocity_per_unit_per_s × drive_m_per_unit` | `play_dat` without `--max-velocity-mps` uses the drive's limit | All, plus `max_velocity_rpm: 100.0` in lilletanken's and mclab's blocks (decided) |
| T9 | `README.md`, `COMMISSIONING.md`, `TODO.md`, `rpi/README.md` | parameter table; new rig section; replug procedure; items 19 and 30 | Document the rig, close item 19 | — | All |
| T10 (optional) | `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt` | `main()` (87-116), `test_move()` (58-83); `install(TARGETS ...)` (49) | `--step`, `--speed`, `--min`/`--max` (tag units) and installing `mgate_example`. Not needed: `--move` did its job on 2026-11-09 | `ros2 run mgate5101_driver mgate_example --move --step 50` moves 50 mm and back | Not needed now |

Order: T5 → T8 → T7; T9 alongside.

Tests Claude can write once you hand them over (`rig_controller/test/test_indradrive_actuator.cpp`,
with the `FakeMGate` from item 21):
- T2: with `drive_m_per_unit: 0.001` and `drive_velocity_per_unit_per_s: 6`, a 0.1 m setpoint
  writes raw 1 000 000 to TargetPosition (100 mm), and a 0.05 m/s cap writes raw 3 000 000 to
  PosVelocity (300 units = 3000 mm/min = 50 mm/s). `max_velocity_mps()` with
  `max_velocity_rpm: 600` returns 0.1. The angular defaults leave ladertanken's numbers
  unchanged.
- T2 with `drive_direction: -1`: a +0.1 m setpoint writes raw −1 000 000 (−100 mm), a raw
  drive position of +50 mm reads back as −0.05 m, the speed cap is still +3 000 000, and
  travel −0.2…+0.2 m becomes drive limits −200…+200 mm with min and max swapped.
- `test_rig_limits.py` for T8: the same 0.1 m/s from the yaml. The existing
  `test_linear_indradrive_has_no_speed_limit` (`test_rig_limits.py:37-40`) expects `None` and
  fails after T8; it then expects 100 / (1/6) × 1 = 600 m/s from the yaml's `max_velocity_rpm: 100.0`
  (factor 1: their units are not set yet, TODO item 19).
- T3: configure → activate with the vertical block's parameters (including `drive_direction` as
  the int `-1`) on the fake gateway, in `test_controller_node.cpp`.
- T7 step 1: `test_source_commit_of_an_installed_copy` patches `motion_files.__file__` instead of
  `gw.__file__`; nothing else in `test_generate_waves.py` changes.
- T7 step 2: `test_generate_oscillations.py` beside `test_generate_waves.py`: the matrix reader
  (each kind, the error messages), stroke → amplitude, the cosine ramp's ends, the jerk column and
  warning, the vertical block's limits, and a file that `play_dat` reads back.

### T7 spec: `generate_oscillations`, the oscillator's `generate_waves` (decided 2026-11-09)

The oscillator gets a generator built like `generate_waves.py`: a test matrix in, the same four
files out (`.dat`, run table `.csv`, record `.txt`, plot `.png`), checked by reading the file back
with `play_dat`. It is the *Oscillator motion generator* in TODO.md (its decisions of 2026-11-03 hold: its own
script, the same file contract). Regular and irregular, as the wavemaker.

**What differs from `generate_waves`:**

| | `generate_waves` | `generate_oscillations` |
|---|---|---|
| Rig | `wavemakers.yaml`, a wavemaker name | `oscillator.yaml`, an oscillator name (`vertical`, `sloshing`), through `rig_limits('oscillator', ...)` |
| Regular input | wave height H or steepness 1/x, period | **stroke** (peak to peak, m), period. No steepness: there is no wave |
| Irregular input | Hs, Tp, γ (JONSWAP of the water surface) | **significant stroke** (4σ of the position, the analogue of Hs = 4σ), Tp, γ: JONSWAP **shape** for the slide's motion |
| Physics | wavenumber, stroke ratio, transfer gain (`paddle_amplitude`, `paddle_amplitudes`) | none: amplitude = stroke / 2; spectrum amplitudes are the slide's |
| `--water-depth` | yes | no |
| Ramp | linear (`apply_ramps`) | **cosine** (see below) |
| Checks | travel, speed, acceleration, start/end at upright | the same, **plus jerk**: printed per run, warning above `--max-jerk-mps3` (default 3.0, the IndraDrive's S-0-0193). A warning, not a skip: the drive rounds the motion off, it does not refuse it |
| Run table columns | `height_m`, `steepness`, `gain`, `paddle_std_m`, ... | `stroke_m`, `amplitude_m`, `sig_stroke_m`, `std_m`, `peak_m`, `peak_jerk_mps3` |
| `play_dat` command printed | `play_dat FILE ladertanken --dry-run` | `play_dat FILE vertical --rig oscillator --dry-run` |

**Test matrix** (same rules as the wavemaker's: one row per run, file order, each row fills its
kind's columns):

```csv
kind,period_s,stroke_m,n_periods,sig_stroke_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,2.0,0.020,10,,,,,,2,5
regular,1.0,0.010,10,,,,,,2,5
irregular,,,,0.010,1.5,3.3,120,1,3,10
```

| Column | Kind | Meaning |
|---|---|---|
| `kind` | both | `regular` (default) or `irregular` |
| `period_s` | regular | period [s] |
| `stroke_m` | regular | stroke, peak to peak [m] (amplitude = stroke / 2) |
| `n_periods` | regular | length in periods, ramps included |
| `sig_stroke_m` | irregular | significant stroke, 4σ of the position [m] |
| `tp_s` | irregular | peak period [s] |
| `gamma` | irregular | JONSWAP peak enhancement; empty = 3.3, 1.0 = Pierson–Moskowitz |
| `duration_s` | irregular | length [s], ramps included |
| `seed` | irregular | random phases |
| `n_ramp` | both | ramp at each end, in periods (regular) or peak periods (irregular) |
| `pause_s` | both | rest after the run, at least 1 s |

**Cosine ramp.** A linear ramp steps the acceleration at the ramp's ends; the third difference sees
a jerk spike of about step / dt (stroke 20 mm, T = 1 s, dt 20 ms, 2 ramp periods: 5.6 m/s³ against
2.5 for the sine; a cosine ramp gives 2.7). With a jerk check that spike would warn on files the
drive handles. Cosine: `r = (1 - np.cos(np.pi * np.linspace(0, 1, n_ramp))) / 2`, same ends as
`apply_ramps` (first 0, last exactly 0). The wavemaker keeps its linear ramp.

**Decided 2026-11-09: (a) the shared module `motion_files.py`, and the name `generate_oscillations`.**

#### T7 step 1: `motion_files.py` (move code, behaviour unchanged)

**Goal.** Everything in `generate_waves.py` that is about the motion file and not about waves
moves into a new module, `rig_controller/src/motion_files.py`. After this step `generate_waves`
behaves exactly as before: the same command line, the same four files, byte for byte. That is
what makes step 1 safe to do on its own, and why it is one commit.

**Why a pure move first.** If the move and the new script come together and a test fails, it is
unclear which one broke it. With 78 tests on `generate_waves` and a byte-identical `.dat` as the
yardstick, step 1 is either clearly right or clearly wrong.

**1.1 Make a reference file before touching anything.** On the PC, in the repo:

```bash
mkdir -p /tmp/t7ref && cd /tmp/t7ref
cat > m.csv <<'END'
kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,1.0,,1/60,30,,,,,,3,5
regular,1.3,0.171,,30,,,,,,3,5
irregular,,,,,0.04,1.2,3.3,120,1,5,10
END
python3 ~/ros_ws/src/hydrolab_rigs/rig_controller/src/generate_waves.py ladertanken m.csv -o before \
  --config ~/ros_ws/src/hydrolab_rigs/wavemaker_bringup/config/wavemakers.yaml --no-plot
```

Keep `before.dat` and `before.csv`. (The `.txt` differs anyway: date and commit.)

**1.2 Create `motion_files.py`.** Module docstring: move the two contract paragraphs from the top
of `generate_waves.py` (*Contract with play_dat.py*, *Contract with the controller*, lines 8-32)
here, because they describe the file, not the waves. `generate_waves`'s docstring keeps its first
paragraph and points to `motion_files` for the contract.

Imports it needs: `from collections.abc import Callable`, `csv`, `from datetime import datetime`,
`math`, `os`, `shlex`, `subprocess`, `from typing import TypeVar`, `numpy as np`, `play_dat`. Not
`rig_limits`: the module gets limits passed in, it never reads a yaml.

Then move these, in this order (the order of the file):

| From `generate_waves.py` | In `motion_files.py` | Change |
|---|---|---|
| `G = 9.81` (54) | `G = 9.81` | None. `jonswap` uses it, so it lives with `jonswap`; `generate_waves` imports it back for its wave physics. Keeping the formula unchanged is what keeps the `.dat` byte-identical |
| `T = TypeVar('T')`, `number` (234-245) | same | None |
| `apply_ramps` (301-312) | `apply_ramps(x, n_ramp_samples, shape='linear')` | New argument, see 1.3 |
| `SIGMA_1`, `SIGMA_2` (318-319) | same | None (they are `jonswap`'s defaults). `F_MIN_FACTOR` / `F_MAX_FACTOR` stay in `generate_waves`: they are each generator's band choice |
| `frequency_grid`, `jonswap`, `wave_amplitudes`, `random_phases`, `synthesize`, `synthesize_direct` (322-395, not `paddle_amplitudes`) | same | None. `wave_amplitudes` keeps its name (a test uses it); its docstring can say "amplitudes of a spectrum" |
| `check_run` (432-461) | `check_run(limits, x, dt, max_velocity_mps)` | Rename `wm` → `limits` in the signature and body; the docstring says what `limits` needs: `upright_m`, `min_m`, `max_m`, `max_acceleration_mps2` |
| `assemble` (464-490) | `assemble(runs, dt, check)` | Generalised, see 1.4 |
| `write_motion` (493-500) | same | None |
| `write_run_table` (503-513) | `write_run_table(path, rows, columns)` | The column list becomes an argument |
| `source_commit` (516-530) | same | None |
| `write_record` (533-561) | `write_record(path, program, argv, config, rig_lines, dt, speed, summary, matrix_path)` | Generalised, see 1.5 |
| `verify_with_play_dat` (564-578) | `verify_with_play_dat(dat_path, limits, max_velocity_mps)` | `wm` → `limits` |
| `plot_motion` (581-621) | `plot_motion(png_path, limits, frames, rows, dt, max_velocity_mps, show=False)` | `wm` → `limits`. It only uses `name`, `min_m`, `max_m`, `upright_m`, which `RigLimits` has too |

**1.3 `apply_ramps` with a shape.**

```python
def apply_ramps(x, n_ramp_samples: int, shape: str = 'linear'):
    """Ramp 0 -> 1 at the start and 1 -> 0 at the end; the run ends exactly at upright.

    'linear' (the wavemaker's) or 'cosine' (the oscillator's: its slope is 0 at both ends, so the
    acceleration does not step where the ramp meets the steady part).
    """
    ...the existing length check...
    ramp = np.linspace(0.0, 1.0, n_ramp_samples)
    if shape == 'cosine':
        ramp = (1.0 - np.cos(np.pi * ramp)) / 2.0
    elif shape != 'linear':
        raise ValueError(f"ramp shape '{shape}' must be linear or cosine")
    ...the rest unchanged...
```

The default keeps every existing call (`generate_waves`, the two `apply_ramps` tests) unchanged.

**1.4 `assemble` without the wavemaker.** Today it loops over specs, calls `regular_signal` or
`irregular_signal` by type, checks with `check_run(wm, ...)`, and builds frames and rows. The
loop's bookkeeping (lead-in, run numbers, start and end times, pauses, skipping) is shared; making
the signal and checking it are not. So:

```python
def assemble(runs, dt: float, check):
    """
    Frames (n x 2: position, trigger) and run-table rows, in file order.

    runs: (x, row, pause_s) per run, x relative to upright, row the generator's own fields.
    check(x): the reasons the run would be refused; a run with any is skipped.
    """
    ...the existing body, with
        for index, (x, row, pause_s) in enumerate(runs):
    instead of building x and row from the spec, check(x) instead of check_run(wm, ...),
    and pause_s instead of spec.pause_s...
```

`generate_waves` keeps a wrapper with the old signature, because the tests call
`gw.assemble(wm, specs, dt, max_velocity_mps)`:

```python
def run_signal(wm, spec, dt):
    """(x, row, pause_s) for one spec of the test matrix."""
    if isinstance(spec, RegularSpec):
        x, row = regular_signal(wm, spec, dt)
    elif isinstance(spec, IrregularSpec):
        x, row = irregular_signal(wm, spec, dt)
    else:
        raise TypeError(f'unknown spec {type(spec).__name__}')
    return x, row, spec.pause_s


def assemble(wm, specs, dt, max_velocity_mps):
    runs = [run_signal(wm, spec, dt) for spec in specs]
    return motion_files.assemble(
        runs, dt, lambda x: motion_files.check_run(wm, x, dt, max_velocity_mps))
```

One visible difference: the old loop raised `TypeError(f'run {index}: ...')` with the run number.
No test or user hits it (the matrix reader only makes the two types), so the message may lose
the number.

**1.5 `write_record` without the wavemaker.** The lines must come out in the same order, because
`test_main_end_to_end` and `test_record_depth_from_the_yaml` read them. The common lines are
`generated`, `command`, `folder`, `commit`, `config`, then the rig's lines, then `dt`,
`max speed`, `result`, an empty line, and the test matrix:

```python
def write_record(path, program, argv, config, rig_lines, dt, speed, summary, matrix_path):
    """<output>.txt: what the file was made from, to read the measurements and to make it again."""
    with open(matrix_path) as f:
        matrix = f.read().rstrip('\n')
    lines = [
        f'generated    {datetime.now().astimezone().isoformat(timespec="seconds")}',
        f'command      {program} {shlex.join(argv)}',
        f'folder       {os.getcwd()}',
        f'commit       {source_commit()}',
        f'config       {os.path.realpath(config)}',
        *rig_lines,
        f'dt           {dt} s',
        f'max speed    {speed}',
        f'result       {summary}',
        '',
        f'test matrix {matrix_path}:',
        matrix,
    ]
    ...write as before...
```

`generate_waves.write_record` keeps its old signature and builds `rig_lines`
(`wavemaker    ladertanken (flap)`, `water depth  ...`) and the `speed` text exactly as today,
then calls this one with `program='generate_waves'`.

**1.6 Change `generate_waves.py`.**

- Remove the moved definitions. Add `import motion_files`, and import the moved names that its
  own code or the tests use:

  ```python
  import motion_files
  from motion_files import (  # noqa: F401  re-exported: test_generate_waves uses gw.<name>
      G, SIGMA_1, SIGMA_2, apply_ramps, check_run, frequency_grid, jonswap, number,
      random_phases, source_commit, synthesize, synthesize_direct, verify_with_play_dat,
      wave_amplitudes, write_motion)
  ```

  The `noqa` is needed because flake8 (part of `colcon test`) flags names imported but not
  used in this file (`synthesize_direct`, `source_commit`, ...).
- Keep wrappers for the three functions whose signature changed: `assemble` (1.4),
  `write_run_table(path, rows)` calling `motion_files.write_run_table(path, rows, WAVE_COLUMNS)`
  with today's column list as the module constant `WAVE_COLUMNS`, and `write_record` (1.5).
- `plot_motion` is called by `main` only: call `motion_files.plot_motion` there, no wrapper.
- Drop imports `generate_waves` no longer uses (`csv`, `datetime`, `shlex`, `subprocess`, ...):
  flake8 flags them too.

**1.7 Install the module.** `rig_controller/CMakeLists.txt` line 79:
`install(FILES src/play_dat.py src/rig_limits.py src/motion_files.py ...)`. Without it the
installed `generate_waves` fails on `import motion_files`. (Claude can do this line.)

**1.8 The one test change** (Claude): `test_source_commit_of_an_installed_copy` patches
`gw.__file__`; `source_commit` now reads `motion_files.__file__`, so the test patches that.

**1.9 Check.**

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test/test_generate_waves.py rig_controller/test/test_play_dat.py -q   # all green
cd /tmp/t7ref && python3 ~/ros_ws/src/hydrolab_rigs/rig_controller/src/generate_waves.py ladertanken m.csv -o after \
  --config ~/ros_ws/src/hydrolab_rigs/wavemaker_bringup/config/wavemakers.yaml --no-plot
cmp before.dat after.dat && cmp before.csv after.csv && echo identical
cd ~/ros_ws && colcon build --packages-select rig_controller && colcon test --packages-select rig_controller
colcon test-result --verbose | grep -E "flake8|pep257|Summary"
ros2 run rig_controller generate_waves ladertanken /tmp/t7ref/m.csv -o /tmp/t7ref/installed --no-plot   # installed copy runs
```

**Done when:** the tests are green (only 1.8 changed), `identical` is printed, flake8 and pep257
are clean, and the installed `generate_waves` runs.

#### T7 step 2: `generate_oscillations.py`

**Goal.** A new program, built like `generate_waves.py` and using `motion_files` for everything
that is not oscillator-specific. Reading `generate_waves.py` side by side is the easiest way to
write it: the sections below follow its order.

**2.1 The module docstring.** What it does (motion files for one oscillator from a test matrix),
that the contract with `play_dat` and the controller is in `motion_files`, and the difference to
`generate_waves` in one sentence: the input is the slide's stroke, there is no wave physics.

**2.2 The oscillator.** No dataclass of its own is needed:

```python
limits = rig_limits.load_limits('oscillator', args.oscillator, config)
```

`RigLimits` has `name`, `namespace`, `upright_m`, `min_m`, `max_m`, `max_velocity_mps` (from T8 for
`vertical`; the LinMot default for `sloshing`) and `max_acceleration_mps2` (the lower of the goal
cap and the LinMot's limit; `None` for `vertical` today). That is everything `check_run`,
`verify_with_play_dat` and `plot_motion` need. The default `config` is
`rig_limits.default_config_path('oscillator')`, as `generate_waves.default_config_path` does for
the wavemakers.

**2.3 The test matrix.** Dataclasses as in `generate_waves` (197-219), with the oscillator's
fields:

```python
@dataclass
class RunSpec:
    n_ramp: int        # periods (regular) or peak periods (irregular) of ramp
    pause_s: float     # rest after the run, >= play_dat.MIN_GAP_S (1 s)

@dataclass
class RegularSpec(RunSpec):
    period_s: float
    stroke_m: float    # peak to peak
    n_periods: int     # including ramps

@dataclass
class IrregularSpec(RunSpec):
    sig_stroke_m: float  # 4 sigma of the position
    tp_s: float
    gamma: float         # 3.3 typical; 1.0 = Pierson-Moskowitz
    duration_s: float    # whole run, ramps included
    seed: int
```

`read_test_matrix(path)` as in `generate_waves` (248-285), with `motion_files.number` for each
required column. Simpler than the wavemaker's: no `height_m` / `steepness` either-or. Columns per
kind: all `kind` (default `regular`), `n_ramp`, `pause_s`; regular `period_s`, `stroke_m`,
`n_periods`; irregular `sig_stroke_m`, `tp_s`, `gamma` (default 3.3), `duration_s`, `seed`. An
unknown kind is an error with the row number, as in the wavemaker's.

Checks the wavemaker's reader does not make, but cheap here and with clear messages: `stroke_m`
and `sig_stroke_m` > 0; `period_s`, `tp_s` > 0; `pause_s` >= `play_dat.MIN_GAP_S`; `n_periods`
>= 2 × `n_ramp` (else `apply_ramps` refuses it later with a sample count, which is harder to read).

**2.4 Regular runs.**

```python
def regular_signal(spec: RegularSpec, dt: float):
    amplitude = spec.stroke_m / 2.0
    n = round(spec.n_periods * spec.period_s / dt)
    t = np.arange(1, n + 1) * dt
    x = amplitude * np.sin(2.0 * math.pi / spec.period_s * t)
    x = motion_files.apply_ramps(x, round(spec.n_ramp * spec.period_s / dt), shape='cosine')
    row = {'kind': 'regular', 'period_s': spec.period_s, 'stroke_m': spec.stroke_m,
           'amplitude_m': round(amplitude, 5)}
    return x, row
```

The same sample grid as the wavemaker's (`t` from `dt` to `n·dt`), so the last sample sits on a
whole number of periods.

**2.5 Irregular runs.** As `generate_waves.irregular_signal` (397-414) without the wave-to-paddle
step:

```python
F_MIN_FACTOR = 0.5   # band fp/2 .. 3 fp, as the wavemaker's
F_MAX_FACTOR = 3.0

def irregular_signal(spec: IrregularSpec, dt: float):
    fp = 1.0 / spec.tp_s
    n, f, df, n_samples = motion_files.frequency_grid(
        spec.duration_s, dt, F_MIN_FACTOR * fp, F_MAX_FACTOR * fp)
    a = motion_files.wave_amplitudes(
        motion_files.jonswap(f, df, spec.sig_stroke_m, spec.tp_s, spec.gamma), df)
    x = motion_files.synthesize(n, a, motion_files.random_phases(spec.seed, len(f)), n_samples)
    x = motion_files.apply_ramps(x, round(spec.n_ramp * spec.tp_s / dt), shape='cosine')
    row = {'kind': 'irregular', 'peak_period_s': spec.tp_s, 'sig_stroke_m': spec.sig_stroke_m,
           'gamma': spec.gamma, 'seed': spec.seed,
           'f_min_hz': round(float(f[0]), 4), 'f_max_hz': round(float(f[-1]), 4),
           'std_m': round(float(x.std()), 5), 'peak_m': round(float(np.abs(x).max()), 5)}
    return x, row
```

Why `sig_stroke_m` goes straight into `jonswap`'s `hs_m`: `jonswap` scales the spectrum so that
4·√m0 equals that argument. For the water that is Hs; for the slide it is 4σ of the position, which
is what *significant stroke* means here. Nothing else changes.

**2.6 Jerk.** The drive's limit S-0-0193 (3 m/s³) is not a refusal, so it is a column and a
warning, not a `check_run` issue:

```python
def peak_jerk(x, dt: float) -> float:
    """max |third difference| / dt^3, as play_dat computes speed and acceleration."""
    return float(np.abs(np.diff(x, 3)).max() / dt ** 3)
```

After each signal, `row['peak_jerk_mps3'] = round(peak_jerk(x, dt), 3)`, and if it is above
`args.max_jerk_mps3` print `run N: peak jerk J m/s^3 above M: the drive rounds the motion off`.
Expect irregular runs to show high jerk sooner than regular ones: jerk grows with f³, so the top
of the band (3·fp) dominates.

**2.7 Run table columns.**

```python
OSCILLATION_COLUMNS = ['run', 'start_s', 'end_s', 'kind',
                       'period_s', 'stroke_m', 'amplitude_m',                    # regular
                       'peak_period_s', 'sig_stroke_m', 'gamma', 'seed',         # irregular
                       'f_min_hz', 'f_max_hz', 'std_m', 'peak_m',
                       'peak_jerk_mps3', 'status']
```

`run`, `start_s`, `end_s` and `status` are what `play_dat` reads (`read_run_table`); the rest is
for people.

**2.8 `main`.** The same shape as `generate_waves.main` (624-678):

| Step | Content |
|---|---|
| Arguments | `oscillator` (name in `oscillator.yaml`), `test_matrix`, `-o/--output` (required), `--config`, `--dt` (0.02), `--max-velocity-mps`, `--max-jerk-mps3` (3.0), `--no-plot`, `--show`. No `--water-depth` |
| Limits | `rig_limits.load_limits('oscillator', ...)`; `max_velocity = args.max_velocity_mps or limits.max_velocity_mps`; if `None`, `parser.error` naming `--max-velocity-mps` (before T8, for `vertical`) |
| Signals | `runs = [run_signal(spec, dt) for spec in read_test_matrix(...)]`, with the jerk added to each row |
| Files | `motion_files.assemble(runs, dt, lambda x: motion_files.check_run(limits, x, dt, max_velocity))`, then `write_motion`, `write_run_table(..., OSCILLATION_COLUMNS)`, `verify_with_play_dat(dat, limits, max_velocity)`, `write_record(..., program='generate_oscillations', rig_lines=[f'oscillator   {limits.name}', f'max jerk     {args.max_jerk_mps3} m/s^3 (warning only)'], ...)`, `plot_motion` unless `--no-plot` |
| Output | `<output>.dat: N runs, M skipped, X min`, then the command: `ros2 run rig_controller play_dat <output>.dat <name> --rig oscillator --dry-run`, plus `--max-velocity-mps` if it came from the command line and `--config` if one was given |

The same warning as the wavemaker's README applies: `-o session` would overwrite `session.csv`.
Refusing `-o` equal to the matrix's base name is cheap here.

**2.9 Install.** Next to `generate_waves` in `rig_controller/CMakeLists.txt` (63-66):

```cmake
# Generates oscillator motion files (.dat + run table) for play_dat (see the script's docstring).
install(PROGRAMS src/generate_oscillations.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_oscillations
)
```

**2.10 Check.**

```bash
mkdir -p /tmp/t7osc && cd /tmp/t7osc
cat > session.csv <<'END'
kind,period_s,stroke_m,n_periods,sig_stroke_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,2.0,0.020,10,,,,,,2,5
regular,1.0,0.010,10,,,,,,2,5
irregular,,,,0.010,1.5,3.3,120,1,3,10
END
ros2 run rig_controller generate_oscillations vertical session.csv -o session_osc
ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator --dry-run
```

**Done when:** three runs, none skipped; `play_dat` lists every run `OK` with the limits and
namespace `/oscillators/vertical`; the `.png` shows the runs inside ±0.2 m; the `.txt` names the
oscillator and the yaml; the run table's `peak_jerk_mps3` is about 0.31 for run 0 and 1.24 for run 1
((s/2)(2π/T)³; the cosine ramp adds a little);
and `sloshing` works too (`generate_oscillations sloshing ...`, LinMot limits). Tests:
`test_generate_oscillations.py` (Claude).

**Needs first:** T8 (the vertical oscillator's speed limit from `rig_limits`) for step 2; without
it the script needs `--max-velocity-mps`, as linear wavemakers do today. Step 1 does not need it.

## Task details

What is left of each task, with the reasons. Done parts are left out.

### T5: the Pi

Left: two checks for when the rigs are switched, and the switching decision.

1. **Refusal, on the wavemaker side.** With the wavemaker drive plugged in and
   `wavemaker@ladertanken` running, `sudo systemctl start oscillator@ladertanken` fails with
   `wavemaker@ladertanken is running; stop it first` (`refuse_if_busy.sh`). Only with the
   wavemaker drive plugged in: the wavemaker's yaml on the oscillator drive would read mm as flap
   degrees (decision 1).
2. **The sloshing rig, when it is connected.** Remove `OSCILLATOR` from `/etc/default/oscillator`
   (or set `sloshing`) and start; the journal shows `LinMot: homed` and the nodes are in
   `/oscillators/sloshing`.
3. **Afterwards.** The Pi stays on `feat-vertical-oscillator`, so the wavemaker runs this branch's
   code too. T2 leaves ladertanken's numbers unchanged (the tests check it), but the first wavemaker
   run after the switch should be a small wave. Back to `master` once the branch is merged:
   `git checkout master && git pull`, `colcon build`.

**Decide: switching between sloshing and vertical.** Both run on the ladertanken Pi as the same
unit, `oscillator@ladertanken`, and the unit's instance (`%i`) is the Pi user, not the
oscillator. With `OSCILLATOR=vertical` in the file, `start_oscillator.bat` and `systemctl start
oscillator@ladertanken` start the vertical rig, and the sloshing rig only after editing it back.
Options:

- (a) **Edit the file to switch (recommended for phase 1):** one line, no new unit. Only one rig
  is connected and tested at a time anyway. Add the switch to the replug procedure
  (*Commissioning* step 0).
- (b) A second unit, `vertical@.service`, with `Environment=OSCILLATOR=vertical`. Then
  `refuse_if_busy.sh` takes a list of units (wavemaker and the other oscillator), the wavemaker's
  unit and `windows/_rig.bat` learn the third rig, and the sudoers line gets two more commands.

### T7: generate_oscillations

Spec and plan: *T7 spec* above. In order: T8 first (the speed limit `generate_oscillations` reads
through `rig_limits`), then step 1 (`motion_files.py`, a pure move: 78 tests green and a
byte-identical `.dat`), then step 2 (the script). Claude reviews each step and runs the tests;
the test changes are Claude's once handed over.

### T8: rig_limits

`rig_controller/src/rig_limits.py`, linear branch (87-92):

```python
    else:
        min_m = required(params, 'travel_minimum_m', name, source)
        max_m = required(params, 'travel_maximum_m', name, source)
        if drive == 'linmot':
            max_velocity = params.get('max_velocity_mps', LINMOT_MAX_VELOCITY_MPS)
        else:  # indradrive: as IndraDriveActuator::max_velocity_mps()
            max_velocity = (required(params, 'max_velocity_rpm', name, source)
                            / params.get('drive_velocity_per_unit_per_s', 1 / 6)
                            * params.get('drive_m_per_unit', 1.0))
```

- No direction here, so no `abs`: the result is already positive.
- `required` as in the angular branch: the controller's default (100) would give a wrong limit
  without anyone seeing it.
- **lilletanken and mclab: add `max_velocity_rpm` to both blocks (decided 2026-11-09).** They
  have none in `wavemakers.yaml` (55-81), so after this `play_dat` on them would fail with
  "max_velocity_rpm not set". Add `max_velocity_rpm: 100.0` to each block: the controller's
  default, which they already run with unseen, now visible. Replace it with the drive's value
  when they are deployed (TODO item 19 note, T9). Keep `required` in the code.
- Add `drive_scale.hpp` to the rules the module docstring lists (5-8).
- **Check:**

  ```bash
  cd ~/ros_ws/src/hydrolab_rigs/rig_controller/src
  python3 -c "import rig_limits; print(rig_limits.load_limits('oscillator', 'vertical').max_velocity_mps)"   # 0.1
  python3 -c "import rig_limits; print(rig_limits.load_limits('wavemaker', 'mclab').max_velocity_mps)"       # 600.0
  python3 -c "import rig_limits; print(rig_limits.load_limits('wavemaker', 'ladertanken').max_velocity_mps)" # 0.7632, unchanged
  ```

  Later, `play_dat <file> vertical --rig oscillator --dry-run` without `--max-velocity-mps`
  shows `0.1000 m/s`. 600 m/s for mclab is right for its placeholder factor 1 and shows the
  factor is not set yet (TODO item 19).
- Tests: `test_rig_limits.py` (Claude, see the list above).

### T9: documentation

What happened on 2026-11-09 and the new parameters, in the files people read. Claude can write
these once you hand them over.

| File | Where | What |
|---|---|---|
| `README.md` | *Actuator and limits* (103-113) | `drive_motion`: drop "Linear units are not verified yet (TODO 19)". The `travel_* / min_position_m` row: IndraDrive linear uses only `travel_*`; `min/max_position_m` are LinMot only. Add `drive_m_per_unit` (default 1), `drive_velocity_per_unit_per_s` (default 1/6, the deg/s → rpm rule) and `drive_direction` (±1, int). `max_velocity_rpm` / `hold_velocity_rpm`: velocity tag units, rpm only on angular drives |
| `README.md` | new section after *Running a wavemaker* | Vertical oscillator (ladertanken): `oscillator:=vertical` and `OSCILLATOR=`, replugging (link to COMMISSIONING), the page, Stop holds in place and *Return to upright* |
| `README.md` | after *Generating motion files* (after T7) | *Generating oscillator motion files*: the test matrix with stroke columns, `generate_oscillations`, its four files, the jerk warning, `play_dat ... --rig oscillator` |
| `COMMISSIONING.md` | item 5 (38-41) | MGate fault timeout set 2026-11-09 (slave 2, slot 4, 500 ms, fault value 0); kill test passed on the oscillator (A0108 → C0401, brake holds). Left: the kill test on the wavemaker drive. Note: only one Modbus client on slave 2's outputs while a rig runs |
| `COMMISSIONING.md` | new section *Vertical oscillator* | The replug procedure (*Commissioning* step 0 here) and steps 2-7 with the results (direction, scaling, the five no-load waves, kill test) and the open loaded tests |
| `TODO.md` | item 19 (24-25) | Close it: the three parameters and one set of limits. lilletanken and mclab still need their factors (and a real `max_velocity_rpm`) when deployed: a short note or a new item |
| `TODO.md` | item 30 (32-33) | Half done: the actuator always declares `travel_*`; the controller still declares them only for linear rigs |
| `TODO.md` | *Oscillator motion generator* (269-297) | In progress: the decisions of 2026-11-09 (test matrix like the wavemaker's, regular and irregular, `motion_files.py`, `generate_oscillations`); for the vertical rig too, not only sloshing |
| `TODO.md` | new item | Tracking under changing loads (*Open questions*): drive tuning with Robert (about 2026-11-23); record and report the measured motion per run |
| `rpi/README.md` | *Oscillator* (450-) | Both oscillators on one unit; `OSCILLATOR=` in `/etc/default/oscillator` and how to switch (after the T5 decision); the LinMot-only homing texts; stopping one unit per command (the sudoers rule); a stop after a crashed controller takes about 45 s and ends `Failed` |

## Commissioning (on the rig, one rig at a time)

On the Pi (`ssh ladertanken@ladertanken-rpi.local`), in tmux:

```bash
cd ~/ros_ws && source install/setup.bash
```

0. **Switching drives (every time).**

   ```bash
   sudo systemctl stop wavemaker@ladertanken     # or oscillator@ladertanken
   ```

   1. Replug the PROFIBUS cable to the other drive. That drive's plug is the end of the line:
      termination ON. The oscillator needs slave 2's user parameters `C0 00 08` (Fail_Safe);
      if the wavemaker does not run with them, Import the matching MGate configuration
      (*PROFIBUS diagnosis log*, backups).
   2. Run the monitor and check that the position is the one you expect from
      that drive: about 0 mm on the oscillator, the flap angle on the wavemaker.

      ```bash
      ~/ros_ws/build/mgate5101_driver/mgate_example
      ```

      **In parameter mode the monitor shows nothing useful.** After a `mgate_example` run or a
      service stop the drive is in A0050 (control word 0). The monitor then prints
      `SW=0x0888 PM --` with position, velocity and diag exactly 0 (seen 2026-11-09): the drive
      sends no actual values. And ladertanken's upright is 0° (`actuator_upright_angle_deg`), so a
      wavemaker at upright reads near 0 too. Identify the drive by the plug and its display; the
      position check then happens in the controller's `Drive status` line after `enable()`
      (operating mode, target parked on the current position, so nothing moves).
   3. Only then start the matching service. The oscillator's mains must be switched on at the
      cabinet (display A0012 `Ab`, not A0013 `bb`) before anything can enable it.
1. **Drive layout, read only (decision 2).** Done 2026-11-06 from `Default.par`: it matches
   (see *The oscillator drive*).

   Steps 2-4 done 2026-11-09 (live, brake holding, `--move` +5 mm and back; see *PROFIBUS
   diagnosis log*, first move). Step 6 (fault timeout): see below.

   Under the controller 2026-11-09 (`oscillator@ladertanken`, `OSCILLATOR=vertical`, commit
   `cecf951`): configure and activate clean, `Drive status: SW=0xC81E OM AF inpos pos=0.0814`
   (the target parked on the position, nothing moved), all nodes in `/oscillators/vertical`, the
   page connected and idle. `systemctl stop` took 4.5 s with the ordered shutdown (bridge, then
   controller, 0.33 s) before SIGINT.

   **Step 6, first try 2026-11-09 failed:** after `kill -9` the drive stayed at A0108 (enabled,
   holding position) because the MGate's output fault timeout was 0 (off). Switching off by hand
   after a dead controller: `systemctl stop`, then `mgate_example` for 1-2 s (it writes control
   word 0), or the E-stop.

   **Step 6 passed 2026-11-09 after the fix:** in MGate Manager, slave 2, slot 4 (`0xE5 Output 6
   Words`, output 0…11) had timeout 0; set to 500 ms, fault value all `00` (the user parameters
   `C0 00 08` untouched). `kill -9` on the controller: A0108 → C0401 (the same code as after
   control word 0 from `mgate_example`) at once, the brake holding: the drive off without
   anything on the Pi. The setting is
   on slave 2, so it covers the wavemaker too; its kill test is COMMISSIONING item 5. Only one
   Modbus client may touch slave 2's output block while a rig runs: any access restarts the
   MGate's timer.
   After a fault switch-off the drive stays in C0401 (status word `OM Ab`); the next service
   start recovers it: `enable()` → A0108, `SW=0xC81E OM AF` in 0.3 s. The service stop after a
   `kill -9` takes about 45 s: the lifecycle manager waits for the dead controller and is killed
   (SIGINT 20 s, SIGTERM 25 s, then SIGKILL; the unit ends `Failed with result 'exit-code'`).

   **First wave 2026-11-09 16:14** (page: height 0.005 m, period 2 s, about 5 periods; bag
   `~/bags/osc_first_wave` on the lab PC, recorded from the PC over domain 1):
   `oscillator_actuator_setpoint = −1.000 × setpoint` (mm), so `drive_direction: -1` and
   `drive_m_per_unit` act as intended; position follows with a lag of about 160 ms (8 control
   cycles; 0.07 mm rms after the shift, 0.87 mm without); stroke 4.87 of 5.00 mm (97 %). The
   wave started from 0.92 mm, where the earlier unrecorded run had stopped. Stop on the page
   halts in place (by design, README *Check and play*); *Return to upright* brings it to 0.

   **Second wave 2026-11-09 16:16** (height 0.02 m, period 2 s, 34.6 s; bag
   `~/bags/osc_wave_20mm_2s`): actuator = −1.000 × setpoint; lag about 180 ms (9 cycles; 0.28 mm
   rms after the shift); stroke 19.46 of 20.00 mm (97.3 %), steady from the 2nd period. Both
   waves start up (about +2 mm), then the ramp grows. Stop at +8.1 mm held there; *Return to
   upright* 8.1 → 0 mm in 1.0 s (setpoint peak 15 mm/s, under
   `return_to_upright_max_velocity_mps: 0.02`), the position 0.4 mm behind at the last sample.

   **Third wave 2026-11-09 16:19** (height 0.02 m, period 1 s, 10 periods; bag
   `~/bags/osc_wave_20mm_1s`; computed jerk 2.5 m/s³, drive limit 3): lag again about 180 ms (now
   65° of phase); stroke **17.9 of 20.0 mm (89.7 %)**, symmetric (±8.95 mm), steady from the 2nd
   period. At 2 s the same stroke reached 97.3 %.

   **Fourth wave 2026-11-09 16:21** (10 mm, 1 s, jerk 1.2 m/s³; bag `~/bags/osc_wave_10mm_1s`):
   89.7 %, lag 180 ms, the same as 20 mm at 1 s. **The loss depends on the period, not on the jerk
   limit:** linear, a fixed 180 ms lag. Suspect: the positioning-speed cap, peak speed ×
   `positioning_velocity_margin` (default 1.2, not set in the vertical block), too low for the
   drive to catch up after the lag.

   **Fifth wave 2026-11-09 16:32** (10 mm, 1 s, `positioning_velocity_margin: 2.0` on the Pi only;
   bag `~/bags/osc_wave_10mm_1s_m2`): 89.7 % again, lag 160 ms. **The speed cap is not the cause.**
   The two periods fit one first-order lag: τ = 76 ms at 2 s, 78 ms at 1 s (amplitude
   1/√(1+(ωτ)²)), plus about 100 ms of pure delay. In `Default.par`: Kv S-0-0104 = 1.00
   (1000/min), a position-loop time constant of 60 ms if it acts without feedforward; velocity
   feedforward P-0-0040 = 100 %, command smoothing P-0-0099 = 0, average filter P-0-0041 = 0.
   Open: where the 77 ms comes from (Kv with the drive's interpolation of a target that steps every
   20 ms is the candidate), and whether to fix it in the drive (with its owner) or compensate the
   stroke per period in the controller or the files.

   **All five waves were without load and without water** (bare slide). They are a no-load
   baseline: direction, scaling, Stop and return are right, and a period-dependent loss exists
   even unloaded. Don't tune or compensate from them. Under load: repeat 5 and 20 mm at 2 s and the
   brake test (step 3), then a sweep at a fixed stroke (e.g. 10 mm at 3, 2, 1.5, 1, 0.8 s; a T7
   file played with `play_dat`), with current and torque watched in IndraWorks.
2. **Monitor.** With the oscillator plugged in, `mgate_example` must print `live`, a position
   near 0 that follows a jog from IndraWorks (mm), and the diag number IndraWorks shows.
   **The monitor is not read-only.** It writes the whole output block every cycle
   (`write_only_dirty = false`), starting with control word 0 (drive off). Never run it while a
   controller runs. A jog from IndraWorks during the monitor needs IndraWorks to take control
   (e.g. easy startup mode), because the bus is commanding drive off.
3. Brake: switch the drive off at standstill at mid-height; the slide must not drop. Done
   unloaded (2026-11-09); repeat with the load.
4. **Driver-level test move, no controller running.** E-stop within reach:

   ```bash
   ~/ros_ws/build/mgate5101_driver/mgate_example --move
   ```

   The slide must go 5 mm up or down and back at about 1.7 mm/s, then the drive switches off
   with the brake holding. Note which way + goes. Ctrl-C mid-move must halt and disable. A
   5 mm move confirms the position factor (`drive_m_per_unit: 0.001`). About 3 s per move
   confirms the velocity factor (5 mm at 100 mm/min).
5. Dry run of a file (needs T7, and T8 to drop `--max-velocity-mps`):

   ```bash
   sudo systemctl start oscillator@ladertanken
   ros2 run rig_controller generate_oscillations vertical session.csv -o session_osc
   ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator --dry-run
   ```

   The first line must show the travel (−0.2…0.2 m) and upright from the yaml, and
   `namespace /oscillators/vertical`.
6. **Fault timeout:** with the drive enabled, kill the controller (`kill -9` on the
   `controller_node` process). The drive must switch off within about 500 ms (MGate fault
   timeout, COMMISSIONING item 5: set for slave 2, so it covers the oscillator too) and the
   brake must hold the slide.
7. Waves. Done from the web page 2026-11-09, unloaded (waves 1-5 above). Next, with the load:
   a file from `generate_oscillations` played with `play_dat`, recorded from the lab PC (the PC
   sees domain 1; no copying afterwards):

   ```bash
   # lab PC
   source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=1
   ros2 bag record -o ~/bags/<name> --topics /oscillators/vertical/oscillator_actuator_setpoint \
     /oscillators/vertical/oscillator_position /oscillators/vertical/oscillator_setpoint \
     /oscillators/vertical/oscillator_velocity /oscillators/vertical/oscillator_state
   # Pi
   ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator
   ```

   Ctrl-C cancels the current run and stops the program. Then increase stroke and frequency
   step by step, watching the stroke and lag (*Open questions*, tracking).
   From where the lag or the mechanics become a problem, set `max_goal_acceleration_mps2` and
   `min_period_s` in the vertical block (0 = no limit until then; only the drive's acceleration
   and jerk limit the motion). The controller then rejects a goal that is too steep instead of
   the drive rounding it off.

Back to the wavemaker: step 0 the other way round, then `start_wavemaker.bat` or
`sudo systemctl start wavemaker@ladertanken`.

## Phase 2: daisy chain (when the cable arrives)

Both drives on one line, one rig running at a time. One drive must leave address 2. The
sections below assume the oscillator moves to 3, which keeps the wavemaker and its yaml
unchanged. If the oscillator's existing setup must keep 2, move the wavemaker instead and swap
the numbers (open question).

Switching from phase 1:
1. Change the address, wire the chain, and set termination at both ends only.
2. Add slave 3 on the gateway and read the address map.
3. Edit the oscillator's yaml (`input_base_reg`, `output_base_reg`, `profibus_address`).
4. Do the phase 2 code tasks below (block-size check, address-check tool, `mgate_example`
   address options).
5. Drop the replug procedure (Commissioning step 0) from the docs.

- **Hardware:** a line, gateway → drive → drive; termination ON only at the two physical ends.
  Add slave 3 on the MGate from the same GSD with the same modules; fault value 0 and fault
  timeout about 500 ms on its output module.
- **Addresses:** read the gateway's Modbus address map (web console *Protocol Settings → Modbus
  TCP → Modbus address map*). Expected: slave 3 inputs at register 7 (bytes 14-27), outputs at
  register 6 (bytes 12-23); the wavemaker stays at 0/0. Both starts are even, so the tag offsets
  stay relative to the block. An odd start would need `in_byte_base` / `out_byte_base` in
  `IndraDriveConfig`. Confirm with a read-only watch of FC4 at register 7 while jogging, and by
  writing the override (register 6 + 5) with both drives off, using a small address-check
  tool (task T6): `watch HOST INPUT_BASE` (FC4, 7 registers every 100 ms, decoded) and
  `override HOST OUTPUT_BASE PERCENT` (FC6 to `OUTPUT_BASE + 5`, refused unless the control word
  at `OUTPUT_BASE` reads 0), in the style of `tests/probe_map.py`.
- **Code:** reject a wrong block size at configure (`output_word_count` ≠ 6, `input_word_count`
  < 7), since a larger output block writes over the other drive every cycle (task T1). Also
  `mgate_example` needs `--input-base`, `--output-base` and `--profibus-address`, with
  `--move` refusing to run without an explicit address.
- **Config:** the oscillator's `input_base_reg: 7`, `output_base_reg: 6`, `profibus_address: 3`.

## Deferred: both rigs at once

Not planned yet. It needs phase 2 first.

- **Fault timeout per node.** It is unknown what restarts the MGate's fault timer: any Modbus
  write, writes from that connection, or writes to that slave's module. If any write counts,
  the wavemaker node keeps a crashed oscillator node's last outputs alive. Test: both
  controllers running, kill one, and only its drive must switch off. The fallback: one process
  owns the gateway, with both drives on one `MGateDriver`. That needs the byte bases and a
  multi-subscriber cycle callback (`set_cycle_callback`, `mgate_driver.cpp:81`, call site 400).
- **Gateway load: count requests per second, not bandwidth.** Each node makes 3 requests per
  cycle (`MGateDriver::exchange()`, `mgate_driver.cpp:365`). Two nodes at 20 ms make 300
  requests/s, about 75 kB/s: under 1 % of the Ethernet. On PROFIBUS both drives take about 1 ms
  per bus cycle at 1.5 Mbit/s. The limit is the gateway. One client at 20 ms (150 requests/s)
  was clean, but at 10 ms (300 requests/s) there were stalls of ~50-210 ms with TCP
  retransmissions (COMMISSIONING.md, *Verified so far*). Solutions in order: one shared
  connection (3 requests per cycle for both rigs); the status area every Nth cycle; FC23
  (probably not usable, since the inputs are FC4); a slower oscillator poll.
- **Load test (no code change; the probes stand in for two nodes):**

  ```bash
  nstat -az TcpRetransSegs
  ros2 run mgate5101_driver poll_rate_probe --host 192.168.1.70 --seconds 60 --intervals 20 > ~/probe_a.txt &
  ros2 run mgate5101_driver poll_rate_probe --host 192.168.1.70 --seconds 60 --intervals 20 > ~/probe_b.txt
  wait
  nstat -az TcpRetransSegs
  cat ~/probe_a.txt ~/probe_b.txt
  ```

  Both controllers stopped and both drives off (the probes write control word 0). Pass: both
  report 20 ms as usable, no ~210 ms stalls, TcpRetransSegs about unchanged.
- **Also needed then:** a test with two `IndraDriveActuator`s on one fake gateway; a check of
  the MGate's maximum number of Modbus TCP clients; and a launch or service that allows both
  rigs, which `refuse_if_busy.sh` deliberately prevents (shared web port, soft stop on GPIO17).

## PROFIBUS diagnosis log (2026-11-06 – 2026-11-09)

**Status: solved 2026-11-09.** The oscillator drive (MPB-05) requires a Fail_Safe master and
rejected the parameter telegram with code 137 = "FailSafe not supp. by contr.". Slave 2's user
parameters `80 00 08` → **`C0 00 08`** fixed it: P-0-4073 `Data-Exch WD+`, `slave=live`. See
*Cause and fix* below; the sections after it are the 2026-11-06 search, kept for reference.

### Cause and fix (2026-11-09)

- A newer Rexroth GSD set (`Profibus/` on the stick: RX04, RX06, RX08, RX16, RX17 and a new
  `Info_eng.txt`) says: *"For the firmware release MPX05 the GSD File RX060107.GSD (GSD-Revision
  5) has to be used. Older GSD Files (RX010107.GSD/RX040107.GSD) can not be used for the
  firmware release MPX05."* The repo's `resources/profibus_gsd/` only has the 2006 set.
- `RX060107.gsd` defines `Value(137)="FailSafe not supp. by contr."`. Against `RX030107.gsd` it
  sets `Fail_Safe=1` (was 0) and the first user parameter byte `0xC0` (DPV1 + Fail_Safe; was
  `0x80`). The modules and their identifiers are the same (`00 00 D6 E5`).
- Fix applied: on slave 2, still set up from `RX030107.gsd`, user parameters `C0 00 08`.
  **Gap:** the MGate still treats slave 2 as `Fail_Safe=0`; only the drive is told. Whether the
  MGate's Clear state turns the drive off is checked by the kill test (Commissioning step 6).
  Setting up slave 2 from `RX060107.gsd` is the proper form (at the latest in phase 2, when the
  oscillator gets its own slave). Re-adding slave 2 from RX03 silently restores `80 00 08`.
- Not yet checked: the wavemaker (MPB-04) with `C0 00 08`. `Info_eng.txt` assigns MPB-04 to
  `RX040107.gsd` (Fail_Safe=0).

### After the fix (2026-11-09)

- **`wait_for_data()` race** in `MGateDriver`: it returned after the first input read, before
  the status area and `online_` were set, so `mgate_example --move` failed at once with
  "gateway offline" and `slave=DOWN`. Fixed: `have_data_` is set only after a whole successful
  cycle (`mgate_driver.cpp`, `poll_loop()`). The controller was not affected (it waits 200 ms).
- **Mains:** `enable()` timed out waiting for `Ab` (status word 0x488E, A0013 `bb`). The power
  section needs mains switched on at the cabinet (a button); then A0012 `Ab`.
- After every `mgate_example` run the drive shows A0050 (parameter mode): `shutdown()` sends
  control word 0 on purpose. The next `enable()` switches back to operating mode.
- **Brake:** holds the slide with the drive disabled.
- **First move** (`mgate_example --move`, +5 units at 10 units, sensor "6047 Position" logged
  in `TestOscillator.bin` on the stick, catman, 200 Hz):

  | | Drive feedback | Sensor | Expected |
  |---|---|---|---|
  | Step | 4.925 mm | 4.940 mm | 5 mm |
  | Speed (10-90 %) | – | 1.68 mm/s | 10 units = 100 mm/min = 1.67 mm/s |
  | Back at start | 0.027 mm off | 0.027 mm off | – |
  | Direction | – | sensor value rose; sensor counts down as + | **+ command = down** |

  Enable `Ab` → `AF` → A0108 and disable were clean; 159 cycles, no errors. The velocity
  feedback (S-0-0040) jitters about ±0.8 mm/s at standstill.
- MGate output fault timeout (500 ms, fault value 0) and the kill test: done 2026-11-09
  (*Commissioning* step 6). P-0-4088 → 0x0028: decision pending (*Open questions*).

### Symptom

`mgate_example` on the Pi: `slave=DOWN`, status word, position, velocity and diag all 0. The
Modbus link is fine (207 cycles, no read or write errors, about 2.8 ms per cycle). The gateway
status is `0003` (Operate), with an empty live list.

### Checked and ruled out

| What | How | Result |
|---|---|---|
| Pi ↔ gateway link | `mgate_example` statistics | OK |
| Gateway, cable, plug, slave 2 setup | Wavemaker drive plugged back in | Live at once (`0003 0400`) |
| Cable and plug | Wavemaker's cable on the oscillator | No change |
| Termination | Both ends | ON at both |
| Address | P-0-4025 on the drive; drive's fieldbus view | 2. The drive receives the gateway's 30 ms watchdog, which only arrives in Set_Prm |
| Baud rate | Drive's fieldbus view | 1500 kBaud found |
| Ident | Diagnosis octets 5-6; MGate Autoscan | `0x0107`, as in the GSD |
| Data lengths | `Default.par` P-0-4080/4081; drive's fieldbus view | 2+4+4+4 = 14 in, 2+4+4+2 = 12 out; no parameter channel |
| Module configuration | MGate Manager slave 2; MGate Autoscan | Both `00 00 D6 E5` (ParamCh not used, F-module not used, Input 7 Words, Output 6 Words) |
| Fieldbus parameters vs the wavemaker | `Default.par` vs `WMDefault.par` | Same P-0-4074, P-0-4080/4081 lengths, P-0-4084 = 0xFFFE. Differ only in P-0-4088 and P-0-4076 (1000 vs 2000 µs) |
| GSD | MGate's GSD folder copied to the stick | `RX030107.gsd` V3.00, identical to the repo copy. No newer GSD on the Windows PC |
| User parameter data | MGate Manager slave 2 | `80 00 08`, as in the GSD |
| DPV1 | First user parameter byte `80` → `00`, MGate restarted | Same diagnosis. Restored from the backup |
| Easy startup mode | P-0-4085 / P-0-4086 | 0, not active |
| Leftover state, IndraWorks holding control | Closed IndraWorks, power-cycled the control section | Still DOWN (display A0013) |
| Per-slave diagnosis in the gateway's Modbus registers | FC4 read of 64 words from 768 | Only status and live list |
| Earlier master configuration | Searched for a Configurator III project; LabVIEW code | None. The 2018 vertOSc project (cRIO-9113) used address 11, two axes, 24 out / 28 in: not this drive's setup |

### What the drive reports

- **P-0-4073** alternates between `Wait-Prm` and `Wait-Cfg`; warning E4005. `Wait-Cfg` means
  the drive accepted a parameter telegram (Reference Book p. 1318), so it gets past the standard
  part of Set_Prm, then falls back to `Wait-Prm`.
- **Slave diagnosis** `42 05 00 FF 01 07 05 81 00 00 89`:

  | Octet | Value | Meaning |
  |---|---|---|
  | 1 | `42` | Prm_Fault, Station_Not_Ready. Cfg_Fault is **not** set |
  | 2 | `05` | Prm_Req (asks to be parameterized again), plus the always-1 bit |
  | 3 | `00` | – |
  | 4 | `FF` | No master has parameterized it |
  | 5-6 | `0107` | Ident, matches the GSD |
  | 7-11 | `05 81 00 00 89` | Device-related DPV1 status block, value 137. `RX030107.gsd` lists 128-136, all parameter-telegram errors; 137 is not defined |

### Tried before the cause was found (2026-11-06 – 2026-11-09)

- User parameters `00 00 00` (DPV1 and structure flag off): identical diagnosis, code 137. The
  Fail_Safe bit was 0 in every try.
- Windows PC searched for newer GSD files (with and without the KUNBUS folders): only the 2006
  Indramat set; KUNBUS Configurator III holds no Rexroth GSD. IndraWorks Ds 14V24 ships none.
- The cRIO master's FPGA build (`C:\NIFPGA\compilation\ladertanken_...CScRIO-PBDP-Mast...`):
  no slave configuration in it.
- gsd-db.org / gsd-db.com unreachable; Rexroth support not available.

### Side findings

- **P-0-4088 = 0x002A is a safety question for a vertical axis.** Bits 2/1 = 01 make a bus
  failure only a warning and keep the last command (hence E4005). With Fail_Safe now on, the
  drive's reaction to the master's Clear state also goes through these bits. The wavemaker has the
  default 00: bus failure is an F4xxx error with the drive's error reaction. The controller's
  fault path (control word 0, MGate fault value) relies on that. Proposal: 0x0028 before the
  oscillator moves under the controller; it changes the drive's existing setup.
- **The MGate's output fault timeout was off** (timeout 0 on slave 2's output module). Set to
  500 ms on 2026-11-09 (*Commissioning* step 6).
- `S-0-0051` (oscillator) vs `S-0-0386` (wavemaker) as the position input: same length,
  same value with one encoder.

### Backups (USB stick `LADERSCHTIC`)

- `Mgate Backup/mgate_ladertanken_wavemaker_OK_2026_11_06.dat`: gateway configuration with
  the wavemaker working, including GSD files.
- `Default.par` (oscillator), `WMDefault.par` (wavemaker): IndraWorks exports, 2026-11-06.
- `Profibus/`: the newer Rexroth GSD set (RX04-RX17) with `Info_eng.txt`.
- `TestOscillator.bin`: sensor log of the first move (2026-11-09).
- To do: export the MGate configuration as it is now (`C0 00 08`, fault timeout 500 ms) as the
  oscillator's backup, if not done after the 2026-11-09 change.

## Open questions

- **Tracking under changing loads (2026-11-09).** Unloaded, the slide follows with a lag of
  about 77 ms (first order) plus 100 ms delay: 97 % of the stroke at 2 s, 90 % at 1 s
  (*Commissioning*, waves 1-5). The load is not fixed: different models, profiles, water
  resistance and depths, so one fixed correction cannot hold. Options: (1) a stiffer drive
  (Kv S-0-0104 = 1.0, acceleration feedforward S-0-0348 / P-0-1126 = 0; with the drive's owner);
  (2) always record and report the measured motion per run (`oscillator_position`); (3) per-setup
  calibration with a T7 sweep and a gain table per period (the controller's table is wavemaker
  only today); (4) amplitude correction during the run (closed loop, steady sines only).
  Recommended: 1 and 2 first. **To decide with the experimenters:** must the slide reach the
  requested stroke exactly, or is it enough to know exactly what it did?
  **Tuning (option 1) waits for Robert, back about 2026-11-23.** Until then: code.

- Does the wavemaker (MPB-04) run with slave 2's user parameters `C0 00 08`? If not, Import
  the matching MGate configuration when switching drives (Commissioning step 0).
- Set up slave 2 from `RX060107.gsd` instead of RX03 with edited user parameters? Add the newer
  GSD set to `resources/profibus_gsd/`?
- P-0-4088 bits 2/1: set the oscillator to 0x0028 (bus failure as error)?
- Phase 2: which drive leaves address 2, the oscillator (wavemaker yaml unchanged) or the
  wavemaker (oscillator setup unchanged)?
- Raise the jerk S-0-0193 (3 m/s³) for faster strokes? Part of the drive tuning with Robert.
- Travel, stroke, max speed and rest position wanted for the vertical oscillator.
- Is the axis counterbalanced, or does the motor hold the load? The brake is 5 Nm.
- The soft stop on GPIO17: the vertical rig uses it (released at every start, 2026-11-09). Is the
  button within reach of the vertical rig, or only at the wavemaker?
