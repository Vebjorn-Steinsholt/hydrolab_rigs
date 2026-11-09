# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

Line numbers are as of commit `6b994c7` plus the uncommitted work on `bugfix-items-20-25`, except
*Your tasks* and *Task details*: commit `3befe17` on `feat-vertical-oscillator` plus the
uncommitted T2, T3 and T4 work (2026-11-09).

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
| S-0-0193 positioning jerk | **3000 mm/s³ (3 m/s³)** | 10 000 rad/s³ | **Limits the oscillation** (decision 5) |
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
- **Linear IndraDrive scaling is unfinished, and this drive hits it (TODO item 19):**
  - Position: `DriveScale`'s linear branch has `m_per_unit = 1` (`drive_scale.hpp`). A 0.1 m
    goal is sent as 0.1 and becomes **0.1 mm**. The controller's limits (`min/max_position_m`)
    are checked in the same wrong unit.
  - Velocity: `write_setpoint()` sends `max(0.1, |v| / 6)` (`indradrive_actuator.cpp:234`),
    the deg/s → rpm rule, with `v` in position tag units per second. Today `v` is the metre
    value (0.05 for 0.05 m/s), so the cap is clamped to 0.1 units = **1 mm/min**. With only
    the position factor fixed, `v` = 50 mm/s and the cap is 8.3 units = 1.4 mm/s, still
    **36 × too low**, because the drive needs `6 × v`. `max_velocity_mps()` (252) has the
    inverse error.
  - The errors are on the safe side (small, slow moves), but the rig cannot run until T2 is done.
- **Only one rig runs on the Pi at a time, already.** `wavemaker@` and `oscillator@` refuse to
  start while the other runs (`rpi/refuse_if_busy.sh`, which also catches a launch by hand
  through the web port), and `windows/_rig.bat` stops the other before starting one. But the
  `oscillator` case in `rpi/run.sh` always starts `oscillator.launch.py`, which hard-codes the
  LinMot rig (`NAMESPACE = 'oscillators/sloshing'`).
- **`play_dat` already plays to an oscillator** (`--rig oscillator`, action `move_oscillator`).
  With a rig name it reads limits and namespace from `oscillator.yaml` (`rig_limits.py:20`).
  For a linear IndraDrive, `rig_limits` gives no speed limit (`rig_limits.py:87-92`), so
  `--max-velocity-mps` is required until T8. `generate_waves` is wavemakers only; the
  oscillator generator is a TODO (*Oscillator motion generator*).
- **`mgate_example` works on the oscillator as it is**, because blocks 0/0 and address 2 are
  the same (`drivers/mgate5101_driver/examples/example_main.cpp`). `--move` steps +5 and back
  at 10: on this drive **5 mm at 100 mm/min (1.7 mm/s)**, about 3 s each way. Its limits default
  to −550…500 (`IndraDriveConfig`), outside ±211 mm, so the drive's limits are the only
  backstop; that is fine for a 5 mm step from about 0. It is built but not installed
  (`~/ros_ws/build/mgate5101_driver/mgate_example`).

## Decisions (proposed)

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
     0x0004, P-0-0540; motor `-UG1`). At 20 mm/rev that holds about 1.5 kN. Still check that the
     stop reaction (P-0-0119 = 0) brakes the slide before the brake engages rather than letting
     it coast, and decide where the slide rests when disabled.
   - **The drive limits every move.** The controller sends no acceleration
     (`max_acceleration_mps2()` is infinite), so the drive's positioning acceleration (S-0-0260
     = 20 m/s²) and **jerk (S-0-0193 = 3 m/s³)** do.
   - **Jerk is the binding limit.** A sine of stroke `s` and period `T` needs acceleration
     `(s/2)(2π/T)²` and jerk `(s/2)(2π/T)³`. Examples:
     - 20 mm at 1 s: 0.39 m/s², 2.5 m/s³.
     - 20 mm at 0.8 s: 0.62 m/s², 4.8 m/s³.
     - 50 mm at 2 s: 0.25 m/s², 0.78 m/s³.

     Above 3 m/s³ the drive rounds the motion off, so the slide lags and the stroke shrinks.
     Raising S-0-0193 changes the drive's existing setup, so decide that with whoever owns it.
     Until then, keep test files under 3 m/s³ (T7 prints it).
6. **One rig at a time stays enforced by the Pi services.** The vertical oscillator runs as
   `oscillator@ladertanken`, which cannot start while `wavemaker@ladertanken` runs. The
   oscillator launch gets an `oscillator` argument (`sloshing` or `vertical`), so the LinMot rig
   keeps working.
7. **Files are played with `play_dat` as they are.** Until the oscillator generator exists, a
   small stopgap tool makes sine test files (task T7). Pass `--max-velocity-mps` until T8.

## Your tasks

| # | File | Where | What | Done when | Left (2026-11-09) |
|---|---|---|---|---|---|
| T5 | the ladertanken Pi: `/etc/systemd/system/oscillator@.service`, `/etc/default/oscillator` | — | Decision 6: install the changed unit, set `OSCILLATOR=vertical` | `sudo systemctl start oscillator@ladertanken` brings up `/oscillators/vertical/controller`; it refuses while the wavemaker runs | Pi install, checks. **Decide: switching between sloshing and vertical** |
| T7 | new tool, e.g. `rig_controller/src/make_osc_dat.py` | stopgap for *Oscillator motion generator* | Sine test runs as a `.dat` (spec below) | `play_dat FILE --rig oscillator ... --dry-run` lists every run as `OK` | All. Ramp: cosine (decided) |
| T8 | `rig_controller/src/rig_limits.py`, `wavemaker_bringup/config/wavemakers.yaml` | `limits_from_parameters` linear branch (87-92); lilletanken and mclab blocks | Speed limit for a linear IndraDrive from T2's parameters: `max_velocity_rpm / drive_velocity_per_unit_per_s × drive_m_per_unit` | `play_dat` without `--max-velocity-mps` uses the drive's limit | All, plus `max_velocity_rpm: 100.0` in lilletanken's and mclab's blocks (decided) |
| T9 | `README.md`, `COMMISSIONING.md`, `TODO.md`, `rpi/README.md` | parameter table; new rig section; replug procedure; items 19 and 30 | Document the rig, close item 19 | — | All |
| T10 (optional) | `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt` | `main()` (87-116), `test_move()` (58-83); `install(TARGETS ...)` (49) | `--step`, `--speed`, `--min`/`--max` (tag units) and installing `mgate_example`. Not needed: `--move` did its job on 2026-11-09 | `ros2 run mgate5101_driver mgate_example --move --step 50` moves 50 mm and back | Not needed now |

Order: T5 → T7 → T8; T9 alongside.

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

### T7 spec: oscillator test file

- Arguments: output file, then runs as `STROKE_M:PERIOD_S:PERIODS`; options `--dt-ms` (default
  20, as `generate_waves --dt`), `--ramp-periods` (2), `--pause-s` (5, at least 1: `play_dat`
  splits runs at 1 s of rest).
- Each run: a sine of half the stroke, a whole number of periods, the amplitude ramped at both
  ends, first and last sample exactly 0. Rest before, between and after.
- File: the layout `read_motion()` reads (`play_dat.py:88`): `<i4` time step in ms, `<i4`
  channels (2), then `<f4` frames of position (m, relative to upright) and trigger (0).
- Print each run's peak speed, peak acceleration and peak jerk (third difference / dt³), and
  warn above the drive's jerk of 3 m/s³ (S-0-0193; decision 5).

**Ramp shape: cosine (decided 2026-11-09, option (a) below).** A linear ramp (as `apply_ramps` in `generate_waves.py:301`) makes the
amplitude's slope jump from A/T_ramp to 0 at the ramp's ends. That happens where sin = 0, so the
velocity stays continuous, but the acceleration steps by 2Aω/T_ramp, and the third difference
sees that as a jerk spike of about step / dt. Computed for stroke 20 mm, T = 1 s, dt 20 ms,
2 ramp periods:

| Ramp | Peak speed | Peak accel. | Peak jerk (3rd difference) |
|---|---|---|---|
| Linear (`apply_ramps`) | 0.063 m/s | 0.39 m/s² | **5.6 m/s³** |
| Cosine | 0.063 m/s | 0.40 m/s² | 2.7 m/s³ |
| Steady sine, `(s/2)(2π/T)ⁿ` | 0.063 | 0.39 | 2.5 |

The spike grows as dt shrinks, so with a linear ramp the warning fires on files the drive
handles: its jerk limit only smooths the step a little. Either:

- (a) **Cosine ramp (recommended):** `r = (1 - np.cos(np.pi * np.linspace(0, 1, n_ramp))) / 2`
  in place of `apply_ramps`' `linspace`. Its slope is 0 at both ends, so the measured peak
  matches the sine's.
- (b) Keep the linear ramp and warn on the steady-state jerk `(s/2)(2π/T)³`, printing the
  measured peak next to it.

Structure:

| Function | Does |
|---|---|
| `parse_run(text) -> (stroke_m, period_s, periods)` | Splits `"0.02:1.0:10"`; stroke > 0, period > 0, periods a whole number ≥ 2 × ramp periods + 1 |
| `make_run(stroke_m, period_s, periods, dt_s, ramp_periods) -> ndarray` | Sine with ramps, exactly 0 at both ends |
| `peaks(x, dt_s) -> (v, a, j)` | `np.abs(np.diff(x, k)).max() / dt_s**k` for k = 1, 2, 3 |
| `write_dat(path, dt_ms, x)` | Header and frames |
| `main()` | Rest, runs and rest; prints the table and warnings; writes the file |

Details that matter for `play_dat`:

- **Samples:** `n = round(periods * period_s / dt_s) + 1`, `t = np.arange(n) * dt_s`. The `+ 1`
  puts the last sample on `t = periods × T`, where sin = 0. Refuse a period that is not a whole
  number of samples (0.7 s at 20 ms is 35, 0.73 s is 36.5): the run would stop mid-period and
  the ramp would force the end to 0.
- **Rest:** `np.zeros(round(pause_s / dt_s))`. `play_dat` splits only where |x| is at most the
  threshold (default **exact zeros**) and the speed is at most 1e-6 m/s for at least 1 s
  (`play_dat.py:53-54`), so the rest must be real `0.0`; float32 keeps it exact.
- **Positions:** metres relative to upright (`play_dat.py:10`). Upright is 0 here and + is up
  after T2, so the sine is the rig position directly.
- **Writing:**

  ```python
  with open(path, 'wb') as f:
      np.array([dt_ms, 2], dtype='<i4').tofile(f)
      np.column_stack([x, np.zeros_like(x)]).astype('<f4').tofile(f)
  ```

- **Peaks** on the float64 array, before the float32 cast.
- **Output**, for example:

  ```
  run  stroke_m  period_s  periods  v_peak  a_peak  j_peak   duration_s
  1    0.005     2.0       10       0.008   0.025   0.08     20.0
  2    0.020     0.8       20       0.079   0.62    4.84 !   16.0
  ! jerk above 3.0 m/s^3 (S-0-0193): the drive will round the motion off
  ```

- **Install** next to `play_dat` in `rig_controller/CMakeLists.txt` (57-61); Claude can add it:

  ```cmake
  install(PROGRAMS src/make_osc_dat.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME make_osc_dat
  )
  ```

- **Check:**

  ```bash
  ros2 run rig_controller make_osc_dat osc_test.dat 0.005:2:10 0.02:1:10
  ros2 run rig_controller play_dat osc_test.dat vertical --rig oscillator --max-velocity-mps 0.05 --dry-run
  ```

  Every run `OK`, and as many runs as `STROKE:PERIOD:PERIODS` arguments.

## Task details

What is left of each task, with the reasons. Done parts are left out.

### T5: the Pi

The repo part is done (launch argument `oscillator`, `run.sh`, `stop.sh`, the unit's texts). Left:
putting it on the ladertanken Pi, the switching decision, and the checks on the rig.

**On the Pi, after `git pull` and `colcon build`:** the unit file changed, so copy it again:

```bash
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/oscillator@.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Then pick the vertical oscillator:

```bash
sudoedit /etc/default/oscillator     # add: OSCILLATOR=vertical
cat /etc/default/oscillator          # ROS_DOMAIN_ID=1, PORT=8080, OSCILLATOR=vertical
```

The file exists if the sloshing rig was installed (rpi/README.md, *Oscillator*); if not, create it
with `ROS_DOMAIN_ID` as in `/etc/default/wavemaker` (run.sh refuses domain 0 or none). The
service requires the file (`EnvironmentFile=` without `-`).

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

**Check, in order:**

1. On the Pi, with the oscillator drive plugged in and its mains on (*Commissioning* step 0):

   ```bash
   sudo systemctl start oscillator@ladertanken
   journalctl -u oscillator@ladertanken -f -o cat     # "Drive status: ..." from the controller
   ros2 node list   # /oscillators/vertical/{controller,bridge,lifecycle_manager,soft_stop,web}
   ```

2. The page on port 8080 shows the bridge's state (`ready`), which proves the bridge block and
   the web namespace.
3. `sudo systemctl stop oscillator@ladertanken` finishes in a few seconds, not 15 s
   (`stop.sh`), and the drive switches off with the brake holding.
4. With `wavemaker@ladertanken` running, starting `oscillator@ladertanken` is refused
   (`refuse_if_busy.sh`, already there).
5. The sloshing rig still starts with `OSCILLATOR` removed or set to `sloshing`, when it is
   connected (or check with `--show-args` and the default).

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
- **Check:** `play_dat osc_test.dat vertical --rig oscillator --dry-run` runs without
  `--max-velocity-mps` and shows `0.1000 m/s`.

### T9: documentation

| File | Where | What |
|---|---|---|
| `README.md` | *Actuator and limits* (103-113) | `drive_motion`: drop "Linear units are not verified yet (TODO 19)". The `travel_* / min_position_m` row: IndraDrive linear uses only `travel_*`; `min/max_position_m` are LinMot only. Add `drive_m_per_unit` (default 1), `drive_velocity_per_unit_per_s` (default 1/6, the deg/s → rpm rule) and `drive_direction` (±1, int). `max_velocity_rpm` / `hold_velocity_rpm`: velocity tag units, rpm only on angular drives |
| `README.md` | new section after *Running a wavemaker* | Vertical oscillator (ladertanken): `oscillator:=vertical`, replugging (link to COMMISSIONING), `make_osc_dat` → `play_dat` |
| `COMMISSIONING.md` | new section | The replug procedure (*Commissioning* step 0) and steps 2-7 with the results so far (2026-11-09: live, brake, +5 mm move, direction) |
| `TODO.md` | item 19 (24-25) | Close it: the three parameters and one set of limits. lilletanken and mclab still need their factors from their drives when deployed: a short note or a new item |
| `TODO.md` | item 30 (32-33) | Half done: the actuator always declares `travel_*`; the controller still declares them only for linear rigs |
| `rpi/README.md` | `/etc/default/oscillator` | `OSCILLATOR=` |

Claude can write the README and docstring parts once you hand them over.

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

   3. Only then start the matching service. The oscillator's mains must be switched on at the
      cabinet (display A0012 `Ab`, not A0013 `bb`) before anything can enable it.
1. **Drive layout, read only (decision 2).** Done 2026-11-06 from `Default.par`: it matches
   (see *The oscillator drive*).

   Steps 2-4 done 2026-11-09 (live, brake holding, `--move` +5 mm and back; see *PROFIBUS
   diagnosis log*, first move). Step 6 (fault timeout) is still open.
2. **Monitor.** With the oscillator plugged in, `mgate_example` must print `live`, a position
   near 0 that follows a jog from IndraWorks (mm), and the diag number IndraWorks shows.
   **The monitor is not read-only.** It writes the whole output block every cycle
   (`write_only_dirty = false`), starting with control word 0 (drive off). Never run it while a
   controller runs. A jog from IndraWorks during the monitor needs IndraWorks to take control
   (e.g. easy startup mode), because the bus is commanding drive off.
3. Brake: switch the drive off at standstill at mid-height; the slide must not drop.
4. **Driver-level test move, no controller running.** E-stop within reach:

   ```bash
   ~/ros_ws/build/mgate5101_driver/mgate_example --move
   ```

   The slide must go 5 mm up or down and back at about 1.7 mm/s, then the drive switches off
   with the brake holding. Note which way + goes. Ctrl-C mid-move must halt and disable. A
   5 mm move confirms the position factor (`drive_m_per_unit: 0.001`). About 3 s per move
   confirms the velocity factor (5 mm at 100 mm/min).
5. Start the oscillator (needs T2-T5) and check the limits:

   ```bash
   sudo systemctl start oscillator@ladertanken
   ros2 run rig_controller play_dat osc_test.dat vertical --rig oscillator --max-velocity-mps 0.05 --dry-run
   ```

   The first line must show the travel (−0.2…0.2 m) and upright from the yaml, and
   `namespace /oscillators/vertical`. Drop `--max-velocity-mps` after T8.
6. **Fault timeout:** with the drive enabled, kill the controller (`kill -9` on the
   `controller_node` process). The drive must switch off within about 500 ms (MGate fault
   timeout, COMMISSIONING item 5: set for slave 2, so it covers the oscillator too) and the
   brake must hold the slide.
7. Play a small file first (e.g. stroke 5 mm, period 2 s), recording setpoint and position:

   ```bash
   ros2 bag record -o ~/bags/osc_test \
     /oscillators/vertical/oscillator_actuator_setpoint /oscillators/vertical/oscillator_position &
   ros2 run rig_controller play_dat osc_test.dat vertical --rig oscillator --max-velocity-mps 0.05
   ```

   Ctrl-C cancels the current run and stops the program. Then increase stroke and frequency
   step by step, watching the lag against S-0-0260 (decision 5).
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
- Not yet done before the oscillator runs under the controller: MGate output fault timeout
  (about 500 ms, fault value 0) and the kill test; P-0-4088 → 0x0028 (decision pending).

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
- **The MGate's output fault timeout may be off** (Autoscan showed timeout 0). COMMISSIONING
  item 5 / Commissioning step 6 above: check it before any `--move`.
- `S-0-0051` (oscillator) vs `S-0-0386` (wavemaker) as the position input: same length,
  same value with one encoder.

### Backups (USB stick `LADERSCHTIC`)

- `Mgate Backup/mgate_ladertanken_wavemaker_OK_2026_11_06.dat`: gateway configuration with
  the wavemaker working, including GSD files.
- `Default.par` (oscillator), `WMDefault.par` (wavemaker): IndraWorks exports, 2026-11-06.
- `Profibus/`: the newer Rexroth GSD set (RX04-RX17) with `Info_eng.txt`.
- `TestOscillator.bin`: sensor log of the first move (2026-11-09).
- To do: export the MGate configuration with `C0 00 08` as the oscillator's backup.

## Open questions

- Does the wavemaker (MPB-04) run with slave 2's user parameters `C0 00 08`? If not, Import
  the matching MGate configuration when switching drives (Commissioning step 0).
- Set up slave 2 from `RX060107.gsd` instead of RX03 with edited user parameters? Add the newer
  GSD set to `resources/profibus_gsd/`?
- P-0-4088 bits 2/1: set the oscillator to 0x0028 (bus failure as error)?
- Phase 2: which drive leaves address 2, the oscillator (wavemaker yaml unchanged) or the
  wavemaker (oscillator setup unchanged)?
- Raise the jerk S-0-0193 (3 m/s³) for faster strokes? It changes the drive's existing setup.
- Travel, stroke, max speed and rest position wanted for the vertical oscillator.
- Is the axis counterbalanced, or does the motor hold the load? The brake is 5 Nm.
- Rig name: `vertical`? It becomes the namespace `oscillators/vertical` and the `play_dat` name.
- The soft stop on GPIO17: is the vertical rig wired to the same button as the wavemaker?
