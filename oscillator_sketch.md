# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

Line numbers are as of commit `6b994c7` plus the uncommitted work on `bugfix-items-20-25`.

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
   - The drive limits come from `travel_minimum_m/maximum_m` (one set of limits, not two).
     `max_velocity_rpm` / `hold_velocity_rpm` stay in velocity tag units (here 10 mm/min each;
     0.1 m/s = 600). The ±211 mm in the drive stay as the backstop.
5. **Vertical axis: gravity, brake and acceleration.** Control word 0 (our fault path, and the
   gateway fault value) removes torque. Confirm that the holding brake is controlled by the
   drive (P-0-0525 and related), that the stop reaction is best possible deceleration followed
   by the brake, and decide where the slide rests when disabled. The controller sends no
   acceleration (`max_acceleration_mps2()` is infinite), so the drive's positioning acceleration
   (S-0-0260, 0.001 mm/s²) limits every move. A sine of stroke `s` and period `T` needs
   `a = (s/2)(2π/T)²`; for example, 20 mm at 1 s needs 395 mm/s². If S-0-0260 is lower, the slide
   lags and the stroke shrinks.
6. **One rig at a time stays enforced by the Pi services.** The vertical oscillator runs as
   `oscillator@ladertanken`, which cannot start while `wavemaker@ladertanken` runs. The
   oscillator launch gets an `oscillator` argument (`sloshing` or `vertical`), so the LinMot rig
   keeps working.
7. **Files are played with `play_dat` as they are.** Until the oscillator generator exists, a
   small stopgap tool makes sine test files (task T7). Pass `--max-velocity-mps` until T8.

## Your tasks

| # | File | Where | What | Done when |
|---|---|---|---|---|
| T2 | `rig_controller/src/drive_scale.hpp`, `indradrive_actuator.cpp` | `DriveScale::from_parameters` linear branch; `write_setpoint()` (234); `max_velocity_mps()` (252); limits (113-119) | Decision 4: `drive_m_per_unit`, `drive_velocity_per_unit_per_s`, limits from travel | A 10 mm goal moves 10 mm; a 0.05 m/s cap is sent as 300; ladertanken's numbers unchanged |
| T3 | `rig_controller/src/controller_node.cpp` | `make_actuator()` (52-68), `on_configure` rig checks | Decision 3: `rig: oscillator` + `indradrive` configures | Fake gateway: configure and activate succeed |
| T4 | `oscillator_bringup/config/oscillator.yaml` | new block after `sloshing` | The vertical oscillator's parameters (see *Config*). It must be in this file: `play_dat` / `rig_limits` look the name up there | `play_dat ... vertical --rig oscillator --dry-run` prints the right limits and namespace |
| T5 | `oscillator_bringup/launch/oscillator.launch.py`, `rpi/run.sh`, `/etc/default/oscillator` on the Pi | `NAMESPACE`; `oscillator)` case | Decision 6: launch argument `oscillator` (default `sloshing`) → namespace `oscillators/<name>`; `run.sh` passes `oscillator:="${OSCILLATOR:-sloshing}"`; the Pi sets `OSCILLATOR=vertical`. The docstring and homing comments are LinMot-specific | `sudo systemctl start oscillator@ladertanken` brings up `/oscillators/vertical/controller`; it refuses while the wavemaker runs |
| T7 | new tool, e.g. `rig_controller/src/make_osc_dat.py` | stopgap for *Oscillator motion generator* | Sine test runs as a `.dat` (spec below) | `play_dat FILE --rig oscillator ... --dry-run` lists every run as `OK` |
| T8 | `rig_controller/src/rig_limits.py` | `limits_from_parameters` linear branch (87-92) | Speed limit for a linear IndraDrive from T2's parameters: `max_velocity_rpm / drive_velocity_per_unit_per_s × drive_m_per_unit` | `play_dat` without `--max-velocity-mps` uses the drive's limit |
| T9 | `README.md`, `COMMISSIONING.md`, `TODO.md` | parameter table; new rig section; replug procedure; item 19 | Document the rig, close item 19 | — |
| T10 (optional) | `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt` | `main()` (87-116), `test_move()` (58-83); `install(TARGETS ...)` (49) | `--step`, `--speed`, `--min`/`--max` (tag units) and installing `mgate_example`. Not needed for the first test: the defaults suit the oscillator | `ros2 run mgate5101_driver mgate_example --move --step 50` moves 50 mm and back |

Order: T2 → T3/T4 → T5 → T7 → T8; T9 alongside.

Tests Claude can write once you hand them over (`rig_controller/test/test_indradrive_actuator.cpp`,
with the `FakeMGate` from item 21):
- T2: with `drive_m_per_unit: 0.001` and `drive_velocity_per_unit_per_s: 6`, a 0.1 m setpoint
  writes raw 1 000 000 to TargetPosition (100 mm), and a 0.05 m/s cap writes raw 3 000 000 to
  PosVelocity (300 units = 3000 mm/min = 50 mm/s). `max_velocity_mps()` with
  `max_velocity_rpm: 600` returns 0.1. The angular defaults leave ladertanken's numbers
  unchanged.
- `test_rig_limits.py` for T8: the same 0.1 m/s from the yaml.

### T7 spec: oscillator test file

- Arguments: output file, then runs as `STROKE_M:PERIOD_S:PERIODS`; options `--dt-ms` (default
  20, as `generate_waves --dt`), `--ramp-periods` (2), `--pause-s` (5, at least 1: `play_dat`
  splits runs at 1 s of rest).
- Each run: a sine of half the stroke, a whole number of periods, the amplitude ramped linearly
  at both ends (as `apply_ramps` in `generate_waves.py:301`), first and last sample exactly 0.
  Rest before, between and after.
- File: the layout `read_motion()` reads (`play_dat.py:88`): `<i4` time step in ms, `<i4`
  channels (2), then `<f4` frames of position (m, relative to upright) and trigger (0).
- Print each run's peak speed and peak acceleration, so they can be compared with the drive's
  limits (decision 5).

## Config

Block for T4, in `oscillator_bringup/config/oscillator.yaml`. The gateway part is ladertanken's,
because the oscillator takes the wavemaker's place on slave 2:

```yaml
/oscillators/vertical/controller:
  ros__parameters:
    rig: oscillator
    drive_name: indradrive
    driver_address: "192.168.1.70"   # the ladertanken gateway, slave 2 (replugged)
    driver_port: 502
    driver_unit_id: 1
    input_base_reg: 0
    input_word_count: 7              # 14 bytes: status, position, velocity, diag (U32)
    output_base_reg: 0
    output_word_count: 6             # 12 bytes: control, target, speed cap, override
    input_uses_fc4: true
    status_base_reg: 768
    status_word_count: 9
    poll_interval_ms: 20
    write_only_dirty: false
    profibus_address: 2
    drive_motion: linear
    drive_m_per_unit: 0.001              # T2: position tag in mm (0.0001 mm resolution)
    drive_velocity_per_unit_per_s: 6.0   # T2: velocity tag in 10 mm/min (0.001 mm/min resolution)
    upright_position_m: 0.0          # drive position about 0 mm at mid-travel
    travel_minimum_m: -0.200         # inside the drive's -211 mm
    travel_maximum_m: 0.200          # inside the drive's +211 mm
    max_velocity_rpm: 600.0          # velocity tag units: 600 x 10 mm/min = 0.1 m/s - set from the rig
    hold_velocity_rpm: 60.0          # 10 mm/s
    in_position_tol_deg: 0.1         # position tag units: 0.1 mm
    step_timeout_ms: 10000
    return_to_upright_max_velocity_mps: 0.02
    # acceleration caps (max_goal_acceleration_mps2) and min_period_s: from the rig
```

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
      termination ON.
   2. Run the monitor (read only) and check that the position is the one you expect from
      that drive: about 0 mm on the oscillator, the flap angle on the wavemaker.

      ```bash
      ~/ros_ws/build/mgate5101_driver/mgate_example
      ```

   3. Only then start the matching service.
1. **Drive layout, read only (decision 2).** In IndraWorks, compare the oscillator drive's
   P-0-4084, P-0-4080, P-0-4081 and S-0-0032 with `indradrive.h:1-15`, and note S-0-0260
   (positioning acceleration) and the brake settings. Change nothing yet.
2. **Monitor.** With the oscillator plugged in, `mgate_example` must print `live`, a position
   near 0 that follows a jog from IndraWorks (mm), and the diag number IndraWorks shows.
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

## Open questions

- Does the oscillator drive's cyclic layout match `indradrive.h` (Commissioning step 1)?
- Phase 2: which drive leaves address 2, the oscillator (wavemaker yaml unchanged) or the
  wavemaker (oscillator setup unchanged)?
- Its positioning acceleration S-0-0260 and jerk: enough for the strokes and periods wanted?
- Travel, stroke, max speed and rest position wanted for the vertical oscillator.
- Does the motor have a holding brake; is the axis counterbalanced?
- Which way + goes (up or down).
- Rig name: `vertical`? It becomes the namespace `oscillators/vertical` and the `play_dat` name.
- The soft stop on GPIO17: is the vertical rig wired to the same button as the wavemaker?
