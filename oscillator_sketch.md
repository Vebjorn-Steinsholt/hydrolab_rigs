# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

Line numbers are as of `feat-vertical-oscillator` on 2026-11-10 where checked; older ones may
be a few lines off.

## Scope (2026-11-06)

- **Phase 1 (2026-11-06 – 2026-11-10): replugging.** The oscillator drive had PROFIBUS
  address 2, as the wavemaker drive does, and the cable was replugged between the two.
- **Phase 2, since 2026-11-10: daisy chain**
  ([Phase 2: daisy chain](#phase-2-daisy-chain-done-2026-11-10)). Both drives stay on the line;
  the oscillator is PROFIBUS address 3 (MGate slave 3, registers 7 / 6), the wavemaker keeps 2
  (0 / 0). One rig still runs at a time. Running both at once is TODO 47.
- **Phase 1 work carries over.** Everything that differs between the phases is in the yaml
  (blocks, PROFIBUS address), so the change is a config edit plus the phase 2 tasks, not a
  rewrite.
- **The oscillator is developed by playing motion files** (`play_dat --rig oscillator`).
- **You write the code.** Each task says what to build, where, and how to check it. Claude
  writes tests and docs, and maintains `mgate_example` (handed over 2026-11-10).

## Goal

A second Rexroth IndraDrive C (rotary motor, linear drive scaling) runs a vertical oscillator.
It is daisy-chained behind the wavemaker drive on the ladertanken MGate (`192.168.1.70`) and
runs from the ladertanken Pi, playing motion files, with the same safety behaviour as the wavemaker.

## The oscillator drive (read 2026-11-06)

| | Value | Meaning for the code |
|---|---|---|
| PROFIBUS address | 3 since 2026-11-10 (2 before) | MGate slave 3, inputs at register 7, outputs at 6; the wavemaker is slave 2 at 0 / 0 |
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
| PROFIBUS address (P-0-4025) | **3** since 2026-11-10 (2 when read) | not in the file (2) | P-0-4031 = 0x00020002 in both files. Writable only with the PROFIBUS plug pulled; takes effect after a 24 V restart (*Phase 2*) |
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
  `MGateDriver` and one `IndraDrive` (`rig_controller/src/indradrive_actuator.cpp:145-146`).
  The blocks, sizes and status area are parameters: the wavemaker uses inputs 0 (7 words),
  outputs 0 (6 words); the oscillator 7 and 6; status 768/9 for both. `IndraDrive` refuses any
  other block size (T1).
- `IndraDrive` assumes a fixed cyclic layout (`indradrive.h:1-15`): P-0-4084 = 0xFFFE,
  drive-internal interpolation (S-0-0032), and P-0-4080/4081 = status, position, velocity, diag
  in; control, target, speed cap, override out. The tags carry the factor 0.0001 and call the
  units "deg" and "rpm" (`indradrive.cpp:27-33`).
- **Linear IndraDrive scaling (TODO item 19)** was unfinished: a 0.1 m goal became 0.1 mm and
  the speed cap was 36 × too low. Fixed by T2 (2026-11-09): `drive_m_per_unit`,
  `drive_velocity_per_unit_per_s`, `drive_direction`, limits from travel.
- **Only one rig runs on the Pi at a time (TODO 47 will allow wavemaker + vertical).** `wavemaker@` and `oscillator@` refuse to start while
  the other runs (`rpi/refuse_if_busy.sh`, which also catches a launch by hand through the web
  port), and `windows/_rig.bat` stops the other before starting one. Since T5 (2026-11-09) the
  oscillator launch takes `oscillator:=sloshing|vertical`, set through `OSCILLATOR=` in
  `/etc/default/oscillator`.
- **`play_dat` already plays to an oscillator** (`--rig oscillator`, action `move_oscillator`).
  With a rig name it reads limits and namespace from `oscillator.yaml` (`rig_limits.py:20`).
  For a linear IndraDrive, `rig_limits` derives the speed limit from `max_velocity_rpm` and the
  drive's units (T8, 2026-11-09), so `--max-velocity-mps` is optional. `generate_waves` is wavemakers only; the
  oscillators have `generate_oscillations` (T7, done 2026-11-10).
- **`mgate_example` reaches the oscillator with options:** `--input-base 7 --output-base 6
  --profibus-address 3` (`drivers/mgate5101_driver/examples/example_main.cpp`); without them it is
  the wavemaker. `--move` refuses without `--profibus-address`. It steps +5 and back
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
   **Ended 2026-11-10 with phase 2:** both drives are on the line at their own addresses and
   blocks, so each yaml can only reach its own drive. No replugging.
2. **The oscillator drive must speak the layout `IndraDrive` expects.** It sits on the gateway's
   slave 3 (slave 2 in phase 1), whose modules are 7 words in / 6 words out. Check its
   P-0-4084, P-0-4080/4081 and S-0-0032 against `indradrive.h:1-15`, **read only**. If the lengths differ, the drive does
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
6. **One rig at a time stays enforced by the Pi services, for now.** The vertical oscillator runs
   as `oscillator@ladertanken`, which cannot start while `wavemaker@ladertanken` runs. TODO 47
   will allow the wavemaker and the vertical oscillator together (*Deferred* below); sloshing
   and vertical never run together. The
   oscillator launch gets an `oscillator` argument (`sloshing` or `vertical`), so the LinMot rig
   keeps working.
7. **Files are played with `play_dat` as they are,** made by `generate_oscillations` (T7, built
   like `generate_waves`); the speed limit comes from the yaml (T8). Single waves also run from the web
   page (height = stroke, period).

## Your tasks

| # | File | Where | What | Done when | Left (2026-11-10) |
|---|---|---|---|---|---|
| T5 | the ladertanken Pi | — | Decision 6: the last check | The sloshing rig still starts | When the sloshing rig is connected (the refusal passed 2026-11-10) |
| T11-T15 | `/etc/default/oscillator`, `rpi/` (units, `run.sh`, `stop.sh`, `refuse_if_busy.sh`, new `soft_stop@.service` + `soft_stop.sh`), `soft_stop.py`, both launches, `windows/_rig.bat` | *Code: wavemaker + vertical together* | Port 8081, oscillator marker, refuse only sloshing, one shared soft stop, Windows start without stopping the other | The rig checks listed there | After Test 2 |
| T10 (optional, Claude's: `mgate_example` handed over 2026-11-10) | `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt` | `main()` (87-116), `test_move()` (58-83); `install(TARGETS ...)` (49) | `--step`, `--speed`, `--min`/`--max` (tag units) and installing `mgate_example`. Not needed: `--move` did its job on 2026-11-09; the block and address options are done (2026-11-10) | `ros2 run mgate5101_driver mgate_example --move --profibus-address 3 --input-base 7 --output-base 6 --step 50` moves 50 mm and back | Not needed now |

Order: Test 2, then T11-T15; T5's check when the sloshing rig is connected. T7 (`generate_oscillations`, with tests and README
section) and T9 were done 2026-11-10.

## Task details

What is left of each task, with the reasons. Done parts are left out.

### T5: the Pi

Left: one check. The refusal passed 2026-11-10: with `wavemaker@ladertanken` running,
`sudo systemctl start oscillator@ladertanken` failed in `ExecStartPre` (`refuse_if_busy.sh`).

1. **The sloshing rig, when it is connected.** Remove `OSCILLATOR` from `/etc/default/oscillator`
   (or set `sloshing`) and start; the journal shows `LinMot: homed` and the nodes are in
   `/oscillators/sloshing`.
2. **Afterwards.** The Pi stays on `feat-vertical-oscillator`, so the wavemaker runs this branch's
   code too. T2 leaves ladertanken's numbers unchanged (the tests check it), but the first wavemaker
   run after the switch should be a small wave. Back to `master` once the branch is merged:
   `git checkout master && git pull`, `colcon build`.

**Decided 2026-11-10: switching between sloshing and vertical by editing `OSCILLATOR=`** in
`/etc/default/oscillator` (option (a)). Both stay one unit, `oscillator@ladertanken`, so systemd
itself keeps the two oscillators from running together; that is wanted, not only a phase-1 limit.
No second unit. The wavemaker and the vertical oscillator should later run together (TODO 47);
until then the refusal in both directions stays as it is.

## Commissioning (on the rig, one rig at a time)

On the Pi (`ssh ladertanken@ladertanken-rpi.local`), in tmux:

```bash
cd ~/ros_ws && source install/setup.bash
```

0. **Switching drives (every time).**

   ```bash
   sudo systemctl stop wavemaker@ladertanken     # or oscillator@ladertanken
   ```

   Since phase 2 (2026-11-10) nothing is replugged: the wavemaker is slave 2 (blocks 0 / 0),
   the oscillator slave 3 (7 / 6), each yaml reaches only its own drive.
   1. For the oscillator: mains on at the cabinet (display A0012 `Ab`, not A0013 `bb`) and
      `OSCILLATOR=vertical` in `/etc/default/oscillator`.
   2. Start the matching service; the controller's `Drive status` line is the check (`OM AF`,
      the position where the slide or flap stands).
   3. **`mgate_example` without options is slave 2 (the wavemaker)**; the oscillator is
      `--input-base 7 --output-base 6 --profibus-address 3`. In parameter mode it shows
      `SW=0x0888 PM --` with everything 0 (the drive sends no actual values).
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
   anything on the Pi. Since phase 2 each slave has its own setting (slave 3: the oscillator).
   The timer runs per slave: another rig's writes do not keep a dead rig alive (*Test 1*,
   2026-11-10). Only one Modbus client may write a slave's output block while its rig runs.
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
2. **Monitor.** `mgate_example --input-base 7 --output-base 6 --profibus-address 3` must print
   `live`, a position
   near 0 that follows a jog from IndraWorks (mm), and the diag number IndraWorks shows.
   **The monitor is not read-only.** It writes the whole output block every cycle
   (`write_only_dirty = false`), starting with control word 0 (drive off). Never run it while a
   controller runs. A jog from IndraWorks during the monitor needs IndraWorks to take control
   (e.g. easy startup mode), because the bus is commanding drive off.
3. Brake: switch the drive off at standstill at mid-height; the slide must not drop. Done
   unloaded (2026-11-09); repeat with the load.
4. **Driver-level test move, no controller running.** E-stop within reach:

   ```bash
   ~/ros_ws/build/mgate5101_driver/mgate_example --move --profibus-address 3 --input-base 7 --output-base 6
   ```

   The slide must go 5 mm up or down and back at about 1.7 mm/s, then the drive switches off
   with the brake holding. Note which way + goes. Ctrl-C mid-move must halt and disable. A
   5 mm move confirms the position factor (`drive_m_per_unit: 0.001`). About 3 s per move
   confirms the velocity factor (5 mm at 100 mm/min).
5. Dry run of a file:

   ```bash
   sudo systemctl start oscillator@ladertanken
   ros2 run rig_controller generate_oscillations vertical session.csv -o session_osc
   ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator --dry-run
   ```

   The first line must show the travel (−0.2…0.2 m) and upright from the yaml, and
   `namespace /oscillators/vertical`.
6. **Fault timeout:** with the drive enabled, kill the controller (`kill -9` on the
   `controller_node` process). The drive must switch off within about 500 ms (MGate fault
   timeout on slave 3, COMMISSIONING item 5) and the brake must hold the slide. Passed again on
   the daisy chain 2026-11-10 (*Phase 2*).
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

Back to the wavemaker: stop `oscillator@ladertanken`, then `start_wavemaker.bat` or
`sudo systemctl start wavemaker@ladertanken`.

## Phase 2: daisy chain (done 2026-11-10)

Both drives on one line, one rig running at a time. The oscillator moved to address 3; the
wavemaker and `wavemakers.yaml` are unchanged.

**Result 2026-11-10:**
- MGate: slave 2 (wavemaker) user parameters `80 00 08`, input bytes 0-13, output 0-11; slave 3
  (oscillator) `C0 00 08`, input 14-27 (register 7), output 12-23 (register 6). Both: modules
  ParamCh not used ×2, Input 7 Words, Output 6 Words, output timeout 500 ms, fault value 0.
  Live list `0x0C` (addresses 2 and 3).
- `oscillator.yaml`: `input_base_reg: 7`, `output_base_reg: 6`, `profibus_address: 3`.
- Vertical run 1 of `rigtest.dat` played over the chain; the wavemaker's block untouched.
- Kill tests (`kill -9` on `controller_node`, both blocks logged at 10 Hz, read only): the
  oscillator left `AF` about 0.6 s later (→ C0401), the wavemaker about 0.5 s later (→ `0x0888`
  parameter mode, as after a service stop; its slave has no Fail_Safe). The other drive's block
  unchanged both times. `sudo pkill -9 -f controller_node` also matches its own `sudo`; use
  `'[c]ontroller_node'`.
- T5 refusal: `oscillator@` refused while `wavemaker@` runs.

**Learned on the way (2026-11-10):**
- The MGate assigns each slave's bytes when it is added and never moves them. After slave 2 was
  re-added it landed behind slave 3; delete both and add slave 2 first, then 3. The start
  addresses cannot be typed in.
- The MGate's scan fills user parameters from the old GSD (`80 00 08`); slave 3 needs
  `C0 00 08` typed in.
- MGate status `diag` with octet 1 `01` (`station_non_existent`) and P-0-4073 `baud search` on
  the drive: no bus signal at the drive (here a wrong cable at the oscillator).
- A cleared bus fault: the wavemaker showed F4009 after being powered off on the chain; clear it
  before starting its service.

What was done, in order (kept for the record):
1. Change the address, wire the chain, and set termination at both ends only. Chain wired
   2026-11-10: MGate (termination ON) → wavemaker (OFF) → oscillator (ON). The oscillator's
   P-0-4025 was set to 3 in IndraWorks. While the MGate is connected, IndraWorks refuses the
   write (it says the parameter is write protected) and can't switch the phase: the master holds
   the drive in phase 2 (`mgate_example`: `PM`), and the write needs phase 4. It works with the
   PROFIBUS plug pulled from the drive: pick "phase switch to phase 4", write, save, then
   power-cycle the control voltage.
2. Add slave 3 on the gateway and read the address map.
3. Edit the oscillator's yaml (`input_base_reg`, `output_base_reg`, `profibus_address`).
4. The phase 2 code: T1 (block-size check) and the `mgate_example` options, done 2026-11-10.
   The address-check tool (T6) was not needed: a read-only FC4 read of the live list and both
   blocks did the job.
5. The replug procedure was dropped from the docs (2026-11-10).

### Phase 2 code (2026-11-10)

Little changes: the IndraDrive and MGate code already address each drive by its own block and
PROFIBUS address. The guard against overlapping blocks (T1) was added 2026-11-10.

**Already works, no change:**
- **Blocks:** the tag offsets in `IndraDrive::IndraDrive` (`indradrive.cpp`) are bytes 0, 2, 6,
  10 *inside the block*, and each controller has its own `MGateDriver` with its own
  `input_base_reg` / `output_base_reg`. The oscillator at 7 / 6 is a yaml change, provided both
  starts are even (14 input bytes and 12 output bytes give exactly registers 7 and 6).
- **Live list:** `slave_live()` (`indradrive.cpp:50-63`) takes the bit for `profibus_address`;
  address 3 is byte 2, bit 3.
- **The drive that is not running:** nobody writes its block, so the MGate's fault timeout on
  that slave forces control word 0. An MGate setting (500 ms, fault value 0), not code.

**T1: refuse an output block other than 6 registers.** Done 2026-11-10: `MGateDriver::config()`
and a check at the top of `IndraDrive::IndraDrive` (`indradrive.cpp`): `output_word_count` must be
6, `input_word_count` at least 7, else `invalid_argument`, which `IndraDriveActuator` turns into a
failed configure. It does not catch a wrong `output_base_reg`: any base is valid on its own.

**`mgate_example`: block and address options.** Done 2026-11-10: `--input-base`,
`--output-base`, `--profibus-address`; defaults 0 / 0 / 2; `--move` refuses without
`--profibus-address`.

**Tests (done 2026-11-10):** `FakeMGate(input_base, output_base, live_list)` plays a second
slave in front of the drive and counts writes to it; `RejectsBlockSizesOtherThanTheDriveLayout`,
`DaisyChainedDriveLeavesTheOtherSlaveAlone` (slave 3 at 7 / 6 enables and moves, no write to slave
2), `DaisyChainedDriveNotLiveRefusesToStart` (`test_indradrive_actuator.cpp`).

**For both rigs at once (TODO 47):** the kill test passed (*Test 1* below), so separate nodes
are enough for the fault timeout. If the load test (*Test 2*) stalls the gateway, both drives
move to one `MGateDriver`, and `set_cycle_callback` needs more than one subscriber (`IndraDrive`
takes the only slot today).

## Deferred: both rigs at once

TODO 47. Phase 2 is done (2026-11-10), so this can start. The first test decides the design.

### Test 1: kill test with both rigs running (passed 2026-11-10)

**Result:** the MGate's fault timer runs per slave. One controller process per rig stays.
- **1a.** Oscillator `kill -9` at 14:49:26.75; at 14:49:27.314 the oscillator went to
  `SW=88AE` C0401, about 0.55 s later. The wavemaker stayed `AF` (A0108), and a wave from the
  page ran afterwards.
- **1b.** Wavemaker `kill -9` at 14:51:08.43; at 14:51:09.005 the wavemaker went to `SW=0888`,
  about 0.57 s later. The oscillator stayed `AF` (A0108).
- The setup worked as written: the namespaces are on the command line, and `pgrep` found exactly
  one controller per pattern. The oscillator's mains were off at the first start (`timeout
  waiting for power section ready (Ab)`, status word `0x488E`); pressing the cabinet button
  fixed it. Don't switch to OM from IndraWorks: `enable()` does that.

Next: Test 2 (gateway load with both rigs), then T11-T15 (*Code: wavemaker + vertical together*).

Procedure, kept for repeats:

**Question:** does the MGate's fault timer run per slave? Or does the other rig's traffic keep a
dead rig's outputs alive? The single-rig kill tests (2026-11-10) only had one writer on the
gateway.

**Setup, without code changes.** The wavemaker runs as its service. The vertical oscillator is
launched by hand, because `refuse_if_busy.sh` only guards the units and port 8080:
- web page on 8081;
- soft stop on GPIO27, an unconnected pin. With the internal pull-up it reads high, which means
  stopped, so the oscillator is enabled (`AF`, holding) but rejects every goal. A kill test only
  needs `AF`.
- The wavemaker keeps GPIO17 and the real button.

```bash
# Pi, terminal 1. Wavemaker drive's mains on; oscillator's mains on (A0012).
sudo systemctl start wavemaker@ladertanken
journalctl -u wavemaker@ladertanken -f                  # Drive status ... OM AF

# Pi, terminal 2 (tmux). OSCILLATOR= in /etc/default is not used here.
cd ~/ros_ws && source install/setup.bash
ros2 launch oscillator_bringup oscillator.launch.py oscillator:=vertical port:=8081 soft_stop_pin:=27
# expect: Drive status ... OM AF; soft stop on GPIO27 starting stopped

# Pi, terminal 3: the two controllers are told apart by their namespace on the command line
pgrep -af '[c]ontroller_node'                           # one /wavemakers/..., one /oscillators/...
```

**Logging, from the lab PC.** A read-only FC4 logger of registers 0-13 at 10 Hz (scratchpad
`killwatch_wm.py` style). It prints both status words and diag numbers on every change. Reads
did not keep a dead rig alive in the 2026-11-10 single-rig tests.

**1a. Kill the oscillator, the wavemaker alive:**

```bash
date +%T.%N; sudo pkill -9 -f '[c]ontroller_node.*oscillators'; date +%T.%N
```

Pass:
- the oscillator leaves `AF` within about 0.6 s (C0401);
- the wavemaker stays `AF` (A0108) and still moves: a small wave from
  `http://ladertanken-rpi.local:8080/`.

Then Ctrl-C terminal 2. It takes up to about 45 s, because the lifecycle manager waits for the
dead controller.

**1b. The other way round:** start the oscillator again in terminal 2 (`AF`), then:

```bash
date +%T.%N; sudo pkill -9 -f '[c]ontroller_node.*wavemakers'; date +%T.%N
```

Pass: the wavemaker leaves `AF` within about 0.5 s (parameter mode `0x0888`), and the oscillator
stays `AF`. Then `sudo systemctl stop wavemaker@ladertanken` (about 45 s) and Ctrl-C terminal 2.

**Result decides:**
- **Both pass:** the timer runs per slave, so two controller processes, one per rig, stay as
  today. TODO 47 is then Pi-side only: the refusal only for sloshing, a second web port, the
  shared soft stop unit, and a ROS name check. Before that, the gateway load test below.
- **A dead rig stays `AF`:** any write keeps it alive, so separate processes are unsafe. One
  process must own the gateway, with both drives on one `MGateDriver` (see the first bullet
  below). That is a bigger code change; plan it before anything else.

**While running both:** the soft stop button stops only the wavemaker (GPIO17), so keep the
E-stop within reach. Nothing in this test commands the oscillator to move.

### Test 2: gateway load with both rigs (next)

**Question:** do two controllers on one MGate (300 requests/s) cause the ~50-210 ms gateway
stalls? Those stalls appeared on 2026-10-07 at the same rate from one client polling every 10 ms.
A stall freezes both rigs' setpoints. Over 500 ms, it would switch a drive off.

**Why the oscillator only holds.** Each controller writes its whole output block and reads its
inputs and the status area every 20 ms (`write_only_dirty: false`), moving or not. So a holding
oscillator loads the gateway exactly like a moving one. The setup from Test 1 is enough: the
oscillator in `AF` with its soft stop on GPIO27 (stopped), and the wavemaker playing waves. Both
rigs moving at once needs the shared soft stop (TODO 47 (4)) and comes after.

**Measured:**
1. `TcpRetransSegs` on the Pi, before and after. This counts lost packets; it rose by 54 with
   the stalls in 2026-10-12.
2. Freezes in the wavemaker's position: `wavemaker_position` is published every control cycle
   (50 Hz) from the drive's feedback. A stall shows as a flat stretch of 100 ms or more in the
   middle of a moving wave. Recorded in a bag on the lab PC.
3. Response time of the gateway, from the lab PC: a read-only FC4 reader (Claude's scratchpad
   script) times one read of registers 0-13 at 10 Hz. That is a third client, but adds only
   about 3 % to the request rate. A stall shows as reads slower than 50 ms.

**Baseline (5 min, the wavemaker alone).** With only the wavemaker service running, play the
same waves as below while the reader and the bag run. Without a baseline, a stall in the test
could also be the wavemaker's own.

**Test (10 min, both rigs):**

```bash
# Pi, terminal 1: the wavemaker service runs, drive in AF (journal: Drive status ... OM AF)
nstat -az TcpRetransSegs                 # note the number

# Pi, terminal 2 (tmux): the oscillator, holding, as in Test 1
cd ~/ros_ws && source install/setup.bash
ros2 launch oscillator_bringup oscillator.launch.py oscillator:=vertical port:=8081 soft_stop_pin:=27
# wait for: Drive status ... OM AF

# lab PC: record the wavemaker
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=1
ros2 bag record -o ~/bags/load_both --topics /wavemakers/ladertanken/wavemaker_setpoint \
  /wavemakers/ladertanken/wavemaker_actuator_setpoint /wavemakers/ladertanken/wavemaker_position

# then: the wavemaker plays waves for about 10 minutes (page http://ladertanken-rpi.local:8080/,
# or play_dat with a wavemaker file); Claude runs the reader on the PC meanwhile

# Pi, at the end
nstat -az TcpRetransSegs                 # the difference is what counts
```

Then Ctrl-C the bag, Ctrl-C terminal 2 (the ordered stop, a few seconds), and stop the wavemaker
service.

**Pass:**
- no read slower than 50 ms, and no flat stretch of 100 ms or more in `wavemaker_position`
  during waves;
- `TcpRetransSegs` up by no more than in the baseline;
- both drives stay in `AF` the whole time.

**If it fails:** the remedies in *Background* (gateway load), in this order:
1. one shared connection for both drives (3 requests per cycle for both rigs), which means one
   process owns the gateway;
2. the status area only every Nth cycle;
3. a slower oscillator poll.

Each of these is a code change, so plan it before the Pi side of TODO 47.

### Code: wavemaker + vertical together (TODO 47 (2)-(5), proposed 2026-11-10)

After Test 2 passes. **Rules (decided 2026-11-10):** sloshing never runs with anything;
wavemaker + vertical may run together; one soft stop button stops both. Test 1 showed separate
controller processes are safe. Everything below is your code unless handed over; Claude writes
the docs (`rpi/README.md`, TODO 47) and the tests.

**T11: own web port for the oscillator.** `/etc/default/oscillator`:

```bash
ROS_DOMAIN_ID=1
PORT=8081
OSCILLATOR=vertical
```

`refuse_if_busy.sh` already checks only its own `PORT`. Sloshing keeps 8081 too: it never runs
next to the wavemaker.

**T12: which oscillator runs, as a marker.** `/etc/default/oscillator` can be edited while the
unit runs, so it cannot tell the wavemaker's unit (or `stop.sh`, which reads `OSCILLATOR` today to
find the namespace) which oscillator is running. `oscillator@.service` gets a runtime directory,
created owned by the user and removed when the unit stops:

```ini
[Service]
RuntimeDirectory=oscillator-%i
```

`run.sh`, oscillator branch:

```bash
    args=(port:="${PORT:-8081}" oscillator:="${OSCILLATOR:-sloshing}" soft_stop:=false)
    # Which oscillator runs, for refuse_if_busy.sh and stop.sh (the env file can change meanwhile).
    [[ -n "$RUNTIME_DIRECTORY" ]] && echo "${OSCILLATOR:-sloshing}" > "$RUNTIME_DIRECTORY/oscillator"
```

`stop.sh`:

```bash
case "${RIG:-wavemaker}" in
  oscillator)
    running="$(cat "$RUNTIME_DIRECTORY/oscillator" 2>/dev/null || echo "${OSCILLATOR:-sloshing}")"
    namespace="/oscillators/$running" ;;
  *) namespace="/wavemakers/$WAVEMAKER" ;;
esac
```

**T13: refuse only sloshing.** Both units call `refuse_if_busy.sh <rig> %i` (`wavemaker` or
`oscillator`) instead of the other unit's name:

```bash
#!/usr/bin/env bash
# ExecStartPre: refuse combinations that cannot run together on this Pi. The wavemaker and the
# vertical oscillator may run together (TODO 47); the sloshing oscillator only alone.
# $1: this unit's rig (wavemaker | oscillator), $2: the Pi user (%i).
set -eo pipefail
rig="$1" pi="$2"
marker="/run/oscillator-$pi/oscillator"   # written by run.sh, removed when oscillator@ stops

if [[ $rig == wavemaker ]]; then
  if systemctl is-active --quiet "oscillator@$pi"; then
    running="$(cat "$marker" 2>/dev/null || echo unknown)"
    if [[ $running != vertical ]]; then
      echo "oscillator@$pi runs '$running'; only the vertical oscillator can run with the wavemaker" >&2
      exit 1
    fi
  fi
else
  if [[ ${OSCILLATOR:-sloshing} != vertical ]] && systemctl is-active --quiet "wavemaker@$pi"; then
    echo "wavemaker@$pi is running; only the vertical oscillator can run with it" >&2
    exit 1
  fi
fi

# Catches a launch by hand on this unit's own port.
port="${PORT:-8080}"
if ss -Hltn "sport = :$port" | grep -q .; then
  echo "port $port is in use, probably a launch by hand; stop it first" >&2
  exit 1
fi
```

**T14: one soft stop for both (TODO 47 (4)).** GPIO17 can be claimed by one process only. The
node leaves the launches and gets a unit of its own, which publishes to every controller that can
run on the Pi; no controller is remapped.

- `rig_controller/src/soft_stop.py`: a `topics` parameter. The default `['soft_stop']` keeps a
  launch by hand as today.

  ```python
          self.topics = self.declare_parameter('topics', ['soft_stop']).value
          ...
          # Not self.publishers: rclpy's Node already has that name.
          self.stop_publishers = [self.create_publisher(Bool, t, qos) for t in self.topics]
          self.get_logger().info(
              f"Soft stop on GPIO{self.pin} to {', '.join(self.topics)}, starting stopped")
      ...
          msg = Bool(data=self.stopped)
          for publisher in self.stop_publishers:
              publisher.publish(msg)
  ```

- `lab.launch.py` and `oscillator.launch.py`: leave their own node out on request.

  ```python
  from launch.conditions import IfCondition
  ...
          DeclareLaunchArgument(
              'soft_stop', default_value='true',
              description='Start a soft stop node; false when soft_stop@.service owns the button'),
          ...
          Node(
              package='rig_controller',
              executable='soft_stop',
              ...
              condition=IfCondition(LaunchConfiguration('soft_stop')),
          ),
  ```

- New `rpi/soft_stop@.service`:

  ```ini
  [Unit]
  Description=Soft stop button on %i's Pi (GPIO17), shared by every rig running there
  # Started by the rig units (Requires=); stops when the last of them stops.
  StopWhenUnneeded=yes

  [Service]
  Type=exec
  User=%i
  WorkingDirectory=/home/%i
  Environment=PI=%i PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0
  EnvironmentFile=/etc/default/wavemaker
  ExecStart=/home/%i/ros_ws/src/hydrolab_rigs/rpi/soft_stop.sh
  KillSignal=SIGINT
  Restart=no
  ```

- New `rpi/soft_stop.sh`:

  ```bash
  #!/usr/bin/env bash
  set -eo pipefail
  [[ -n "$ROS_DOMAIN_ID" && "$ROS_DOMAIN_ID" != 0 ]] || { echo "set ROS_DOMAIN_ID" >&2; exit 1; }
  source /opt/ros/jazzy/setup.bash
  source "$HOME/ros_ws/install/setup.bash"
  # One button, published to every controller that can run on this Pi.
  exec ros2 run rig_controller soft_stop --ros-args -r __ns:="/$PI" -p pin:=17 \
    -p "topics:=['/wavemakers/$PI/soft_stop', '/oscillators/vertical/soft_stop', '/oscillators/sloshing/soft_stop']"
  ```

- Both rig units:

  ```ini
  [Unit]
  Requires=soft_stop@%i.service
  After=network-online.target soft_stop@%i.service
  ```

  and `run.sh` passes `soft_stop:=false` to both launches (see T12 for the oscillator line;
  `args=(wavemaker:="$WAVEMAKER" port:="${PORT:-8080}" soft_stop:=false)` for the wavemaker).

How it behaves:
- One press stops both rigs; releasing lets both start again.
- Stopping one rig leaves the button working for the other (`StopWhenUnneeded`).
- If the soft stop process dies, both controllers treat it as stopped within 0.2 s
  (`soft_stop_timeout_s`), and systemd stops both rigs (`Requires=`).
- A rig started while the other runs gets the button state at once: the topic is
  `TRANSIENT_LOCAL`.

**T15: `windows/_rig.bat` starts without stopping the other rig.** Today `start oscillator`
stops the wavemaker first. Start only the requested unit, leave the refusal to T13, and open the
page on the rig's own port:

```bat
if /i "%~2"=="wavemaker" (set "PORT=8080") else if /i "%~2"=="oscillator" (set "PORT=8081") else goto usage
set "UNIT=%~2@%PI_USER%"
echo Starting %UNIT% on %PI_HOST% ...
%SSH% "sudo -n systemctl start %UNIT% && systemctl is-active %UNIT%"
```

A refused start prints `refuse_if_busy.sh`'s reason in the window. To switch the Pi to
sloshing, `stop_rigs.bat` first. The sudoers rule is unchanged: systemd starts the soft stop
unit through `Requires=`, not sudo.

**Already fine, no change:** ROS names (`/wavemakers/ladertanken`, `/oscillators/vertical`; the
soft stop node moves to `/ladertanken/soft_stop`; both rigs ran this way in Test 1), the driver
and gateway (per-slave blocks, T1, the per-slave fault timer).

**Order:** Test 2, then T11-T15, then on the rig:
- wavemaker and vertical start in either order;
- sloshing is refused in both directions;
- one press stops both;
- stopping one rig leaves the other and the button working;
- stopping the soft stop unit stops both.

**Claude, after T11-T15:**
- the docs: `rpi/README.md` (the new unit, `PORT=8081`, start and stop), `windows/README.md`;
- tests: `soft_stop.py`'s `topics`, and the launch argument;
- TODO 47 marked done.

### Background

- **Fault timeout per node: settled by Test 1** (the timer runs per slave). The fallback it
  ruled out, one process owning the gateway with both drives on one `MGateDriver`, would need a
  multi-subscriber cycle callback (`set_cycle_callback`); it comes back only if Test 2 fails.
- **Gateway load: count requests per second, not bandwidth.** Each node makes 3 requests per
  cycle (`MGateDriver::exchange()`, `mgate_driver.cpp:365`). Two nodes at 20 ms make 300
  requests/s, about 75 kB/s: under 1 % of the Ethernet. On PROFIBUS both drives take about 1 ms
  per bus cycle at 1.5 Mbit/s. The limit is the gateway. One client at 20 ms (150 requests/s)
  was clean, but at 10 ms (300 requests/s) there were stalls of ~50-210 ms with TCP
  retransmissions (COMMISSIONING.md, *Verified so far*). Solutions in order: one shared
  connection (3 requests per cycle for both rigs); the status area every Nth cycle; FC23
  (probably not usable, since the inputs are FC4); a slower oscillator poll.
- **Probe load test (an alternative to Test 2, without the rigs; both probes write control word 0 to slave 2's block 0 / 0 only):**

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
- **Also needed then:** a test with two `IndraDriveActuator`s on one fake gateway (`FakeMGate`
  plays only one drive and counts writes to the other slave today); a check of the MGate's
  maximum number of Modbus TCP clients (3 worked: two controllers and the PC reader, 2026-11-10);
  and the Pi side in TODO 47 (2)-(5): refusal only for sloshing, a second web port, the shared
  soft stop, ROS names.

## PROFIBUS diagnosis log (2026-11-06 – 2026-11-09)

**Status: solved 2026-11-09.** The oscillator drive (MPB-05) requires a Fail_Safe master and
rejected the parameter telegram with code 137 = "FailSafe not supp. by contr.". Slave 2's user
parameters `80 00 08` → **`C0 00 08`** fixed it (since phase 2: slave 3 has `C0 00 08`, the
wavemaker's slave 2 is back at `80 00 08`): P-0-4073 `Data-Exch WD+`, `slave=live`. See
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
  Setting up the oscillator's slave (3 since phase 2) from `RX060107.gsd` is the proper form
  (*Open questions*). Re-adding it from RX03 silently restores `80 00 08`.
- The wavemaker (MPB-04) never ran with `C0 00 08`: since phase 2 its slave 2 has `80 00 08`
  again, as `Info_eng.txt` (`RX040107.gsd`, Fail_Safe=0) expects.

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
- To do: export the phase 2 MGate configuration (slave 2 `80 00 08`, slave 3 `C0 00 08`, both
  500 ms) to the stick.

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

- Set up slave 3 (oscillator) from `RX060107.gsd` instead of RX03 with edited user parameters,
  so the MGate also treats it as Fail_Safe? Add the newer GSD set to `resources/profibus_gsd/`?
- P-0-4088 bits 2/1: set the oscillator to 0x0028 (bus failure as error)?
- Raise the jerk S-0-0193 (3 m/s³) for faster strokes? Part of the drive tuning with Robert.
- Travel, stroke, max speed and rest position wanted for the vertical oscillator.
- Is the axis counterbalanced, or does the motor hold the load? The brake is 5 Nm.
- The soft stop on GPIO17: the vertical rig uses it (released at every start, 2026-11-09). Is the
  button within reach of the vertical rig, or only at the wavemaker?
