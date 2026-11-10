# Wavemaker commissioning checklist

One-time physical checks on the rig, before and during first motion, and the record of what has
been verified. Repeat the relevant items after a hardware change (drive, gateway, belt, flap,
mechanics). The everyday checks before a lab session are in [STARTUP.md](STARTUP.md); open
software work is in [TODO.md](TODO.md).

Item numbers are shared with [TODO.md](TODO.md), so references such as "item 12" stay valid in
both files; gaps in the numbering are the items kept there.

- **(rig)** needs the real wavemaker; the unit tests use fakes for the drive and the controller.
- Paths are relative to the repository root.
- Do not enable motion until every item in [Before any motion](#before-any-motion) is done.
- How to run the tests is in [Rig test procedure](#rig-test-procedure) at the end.

## Before any motion

- [ ] **1. Narrow the ladertanken limits once the transfer gain is known.** (rig)
  The limits are ±275° (`min_position_deg` / `max_position_deg`, ±14.6 cm at the flap top) and 240 rpm (`max_velocity_rpm`), raised 2026-10-20 from ±200° / 170 rpm (verified with the lab waves on 2026-10-08, see *Verified so far*). With the provisional gain table and `hinge_height: 0.09`, the largest lab wave (0.171 m / 1.3 s) needs ±248° and 200 rpm; the new limits add 10 % travel and 1.2 × that speed. Not yet run at these strokes. There are no mechanical end stops, so these are the only travel limits.
  - Once the transfer gain is measured (item 33), recompute the required angles and speeds (theory / gain) and narrow the limits to the largest lab wave plus 10–20 %; keep `max_velocity_rpm` at 1.2 × the largest wave's peak speed.
  - If the old system's settings or logs exist, compare its largest commanded stroke and speed with these numbers.
  - Optional: confirm the drive's degrees are motor degrees by counting the motor pulley's teeth and measuring the belt pitch (teeth × pitch ≈ 190 mm), and check the drive's velocity limit (S-0-0091) is at least 240 rpm.
  - (Mechanical, for whoever owns the mechanics.) The belt can jump a tooth under hard turns, and the motor encoder does not see it: the flap would then be offset from the zero and the travel limits without any warning. Check belt tension and condition regularly; after long lab sessions run `return_to_upright` and check the flap is visually upright.

- [ ] **2. Linear drive limits.** (rig)
  Confirm the configured minimum and maximum (0.0–0.5 m in `wavemakers.yaml`) for lilletanken and mclab.

- [ ] **3. Direction, feedback and zero are correct.** (rig)
  - Positive command gives the expected physical direction. The drive moves positive in degrees first, as commanded (2026-10-07); which way the paddle moves has not been seen yet. Check it with someone at the paddle or a phone camera, at the latest when the flap is reconnected (item 11).
  - Done 2026-10-07: position feedback follows the setpoint with the same sign; `return_to_upright` settles at 0° (±0.002° in the drive software), the upright reference.
  - Angular conversion (`actuator_lead_m_per_degree: 0.00053`) matches the real mechanism; see [Actuator conversion calibration](#actuator-conversion-calibration).

- [ ] **4. Communication is correct and stable.** (rig)
  - IP address, Modbus port, unit ID and PROFIBUS address are correct and unique.
  - Cable shielding, grounding, power and network connections are secure.
  - The drive has no active faults and reports live on PROFIBUS (`mgate_example` in monitor mode).

- [ ] **5. Make the gateway switch the drive off if the PC or the node dies.** (rig; MGate configuration)
  The MGate keeps sending the last outputs (Drive On, Drive Start, last target) if Modbus writes stop. Our driver only sets control word 0 while its process is alive.
  - On the MGate's output module, set a fault value timeout of about 500 ms (it must ride out the ~210 ms gateway stalls; see *Verified so far*) with fault value 0 (MGate manual p. 28). Test by killing the controller while the drive is enabled; the drive must switch off.
  - Done 2026-11-09 (MGate Manager, slave 2, slot 4 `0xE5 Output 6 Words`): the timeout was 0 (off) and is now 500 ms, fault value all `00`. Kill test passed with the vertical oscillator's drive on slave 2: `kill -9` on the controller, the drive left A0108 at once, the brake held (see [Vertical oscillator](#vertical-oscillator-ladertanken)).
  - Done 2026-11-10 on the daisy chain: the same setting on slave 3 (oscillator); kill test passed on both drives, the wavemaker's drive went to parameter mode about 0.5 s after `kill -9`.
  - Any Modbus access to a slave's output block restarts the MGate's timer for it: only one client (the controller) may use it while a rig runs; never run `mgate_example` alongside.
  - The fault timeout relies on writes every cycle; the matching code change is done: `write_only_dirty` defaults to `false` in `IndraDriveActuator` (2026-10-27).

- [ ] **34. Spring balance matches the water depth.** (rig; mechanical, for whoever owns the mechanics)
  The blue ropes run over a pulley to large springs whose tension counteracts the water pressure on the flap, so the motor is stable at upright. The water's moment on the flap grows roughly with depth cubed (1.0 m ≈ 2.5 × 0.73 m), so the tension must be reset whenever the water level changes, together with `water_depth` in `wavemakers.yaml`.
  - Check: with the drive holding upright, motor torque or current in IndraWorks is close to zero. A steady holding torque means the balance is off; it heats the motor and takes from the torque the waves need.
  - The ropes are frayed (fuzzy surface along their length) and carry the preload all the time. If one breaks, the full water moment lands on the motor (overcurrent or following-error fault) or swings the flap toward the dry side. Inspect them now and replace if worn; check them again before the larger lab waves.
  - Release the tension before draining the tank, or the springs pull the flap the other way.

- [ ] **7. Lifecycle without motion.** (rig)
  Configure, activate, deactivate, cleanup and configure again with no goal; activation must cause no motion.

## First motion

- [ ] **9. Confirm the drive keeps following after a stop.** (rig)
  `IndraDriveActuator::halt()` holds the last commanded target instead of using Drive Halt, which made the drive ignore later setpoints. Every stop, cancel and return relies on this.
  - Done 2026-10-07: after a Ctrl-C cancel, a new wave starts and the paddle moves, and `return_to_upright` brings it back to 0° (drive reading −0.0020° to +0.0017° at rest).
  - Done 2026-10-08, flap connected: the `cancel_all_goals` service stops a running wave, and new waves start afterwards.
  - Still to check: stop at speed (cancel as the paddle passes the middle) and check it stops without moving back.

- [ ] **10. Stopping and failures while moving.** (rig)
  Test while a wave runs: bridge stop, emergency stop, network and gateway disconnection. The actuator must disable safely after deactivation or communication loss. Cancel by Ctrl-C (2026-10-07), by the `cancel_all_goals` service and lifecycle deactivation through the lifecycle manager (2026-10-08) are done.

- [ ] **11. Rechecks with the flap reconnected.** (rig)
  The flap was reconnected and run in water on 2026-10-08 (lab waves, see *Verified so far*), before items 9 and 10 were finished. Still to recheck with the flap connected: which way the flap moves for a positive command (item 3), and the emergency stop while a wave runs (item 10).

## Tuning and calibration

- [ ] **13. Tune the drive's speed cap.** (rig)
  Plot `wavemaker_setpoint` against `wavemaker_position`. If the paddle lags at its fastest point, raise `positioning_velocity_margin` (default 1.2); if it moves in steps, lower it or the drive's acceleration (`S-0-0260`).
  - Also look for setpoint freezes of about 0.2 s (gateway stalls; see *Verified so far*). If they occur, check `nstat -az TcpRetransSegs` before and after (it prints the total since boot) and the cable and switch; if they persist, raise `poll_interval_ms`.
  - Activation measures one gateway cycle and fails if it is slow; retry once before suspecting anything else.

- [ ] **14. Complete and record the conversion calibration.** (rig)
  Record the angle/position pairs, measure in both directions (backlash), repeat positions, check linearity, validate with positions not used for the fit. See [Actuator conversion calibration](#actuator-conversion-calibration).

- [ ] **15. Keep a known-good parameter backup** of the drive and gateway, with the tested zero position and limits recorded. (rig)

- [ ] **19. Linear drive scaling (lilletanken, mclab).** (rig)
  Read each drive's position and velocity scaling (S-0-0076/77/78, S-0-0044/45/46) and set `drive_m_per_unit`, `drive_velocity_per_unit_per_s`, `drive_direction` and a real `max_velocity_rpm` in `wavemakers.yaml` (today 1.0, 1/6, 1 and 100: placeholders). The vertical oscillator shows how: [Vertical oscillator](#vertical-oscillator-ladertanken), first move.

- [ ] **33. Calibrate the wave-height transfer gain.** (rig; wave probes, flap connected)
  `wavemaker_transfer_gain` (default 1.0 = linear theory, in `wavemakers.yaml`) divides the stroke of regular waves to correct the generated height; `wavemaker_transfer_gain_periods` / `wavemaker_transfer_gain_values` replace it with a gain per period. Follow [Transfer gain calibration](#transfer-gain-calibration).
  - The gain holds for one water depth; recalibrate after changing it, or after the calibration from item 14.
  - Open (2026-10-21): 0.7 s and 1.05–1.15 s came out 11–46 % too low with the table (see *Verified so far*). Jumps that large over 0.05–0.1 s are unlikely to be the flap's transfer function. Before changing the table, check the paddle tracking in the bags (setpoint against position) and repeat 0.063 m / 1.1 s with the probe moved about a quarter wavelength (0.47 m); a large change there points to reflection. Until then, expect lower waves than targeted at those periods.
  - Not applied to pregenerated waves; decide whether it should be.

## Vertical oscillator (ladertanken)

Checks for the vertical oscillator's IndraDrive on the ladertanken MGate (slave 3, daisy-chained
behind the wavemaker's drive since 2026-11-10). What to code, and why:
[oscillator_sketch.md](oscillator_sketch.md).

- [Switching drives](#switching-drives-every-time), [Done](#done), [Open](#open)
- [The oscillator drive](#the-oscillator-drive-read-2026-11-06): the parameters
- [Procedures](#procedures): monitor, brake, test move, dry run, fault timeout, recorded runs
- [Test 2](#test-2-gateway-load-with-both-rigs-next) (next), [Test 1](#test-1-kill-test-with-both-rigs-running-passed-2026-11-10)
- [Log](#log-2026-11-09--2026-11-10): what was measured, phase 2
- [PROFIBUS diagnosis log](#profibus-diagnosis-log-2026-11-06--2026-11-09), [Backups](#backups-usb-stick-laderschtic)

On the Pi (`ssh ladertanken@ladertanken-rpi.local`), in tmux:

```bash
cd ~/ros_ws && source install/setup.bash
```

### Switching drives (every time)

Both drives stay on the PROFIBUS line (MGate → wavemaker, termination OFF → oscillator,
termination ON): wavemaker slave 2 (blocks 0 / 0), oscillator slave 3 (7 / 6). Nothing is
replugged; each yaml reaches only its own drive.

1. Stop the running service: `sudo systemctl stop wavemaker@ladertanken` (or `oscillator@`).
2. The oscillator's mains must be on at the cabinet (A0012 `Ab`, not A0013 `bb`).
3. For the oscillator: `OSCILLATOR=vertical` in `/etc/default/oscillator`; for the sloshing rig,
   `sloshing` (or no line). One unit runs either oscillator, so the two can never run together.
4. Start the matching service. The controller's `Drive status` line in the journal is the
   position check: `OM AF`, the position near where the slide stands.
5. **`mgate_example` without options is slave 2 (the wavemaker)**; the oscillator is
   `--input-base 7 --output-base 6 --profibus-address 3`. In parameter mode it shows
   `SW=0x0888 PM --` with everything 0 (the drive sends no actual values).

Back to the wavemaker: stop `oscillator@ladertanken`, then `start_wavemaker.bat` or
`sudo systemctl start wavemaker@ladertanken`. A wavemaker drive powered off on the chain shows
F4009; clear it before starting its service.

### Done

- Drive layout matches what `IndraDrive` expects (P-0-4084, P-0-4080/4081, S-0-0032; 2026-11-06,
  `Default.par`). PROFIBUS needs the user parameters `C0 00 08` (Fail_Safe, MPx-05 firmware); the
  wavemaker keeps `80 00 08`.
- Daisy chain (2026-11-10): oscillator P-0-4025 = 3 (written in IndraWorks with the PROFIBUS plug
  pulled, "phase switch to phase 4", then a 24 V restart), MGate slave 3 at registers 7 / 6,
  `oscillator.yaml` changed to match. A vertical run played; kill tests passed on both drives
  (oscillator off about 0.6 s, wavemaker about 0.5 s after `kill -9`), the other drive's block
  untouched; `oscillator@` refused while `wavemaker@` runs. `mgate_example` reaches the oscillator
  with `--input-base 7 --output-base 6 --profibus-address 3`; `--move` needs `--profibus-address`.
  An IndraDrive block other than 7 in / 6 out now fails configure (T1). Details:
  [Phase 2: daisy chain](#phase-2-daisy-chain-done-2026-11-10).
- First move with `mgate_example --move` (2026-11-09): +5 units → 4.925 mm on the drive, 4.940 mm
  on the sensor, 1.68 mm/s: the drive's units are mm and 10 mm/min. **+ command = down**, hence
  `drive_direction: -1`.
- Brake: holds the slide at every switch-off (no load).
- Under the controller (2026-11-09): configure and activate clean, nothing moves at enable, all
  nodes in `/oscillators/vertical`, the page connected; `systemctl stop` in 4.5 s with the ordered
  shutdown.
- `generate_oscillations` file played (2026-11-10, no load, from the PC, bag in
  `~/oscillator_sessions/2026-11-10_rigtest`): 10 mm at 2 s, 10 mm at 1 s, irregular 10 mm Tp 1.5 s.
  Setpoints equal the file within 0.003 mm; slide stroke 97.3 %, 89.5 %, 4σ 92.9 %; lag 190, 170,
  160 ms; start and end within 0.1 mm of upright. The first attempt was refused: the soft stop was
  engaged and released 10 s later.
- Kill test (item 5): passed after the MGate fault timeout was set. After a fault switch-off the
  drive shows C0401; the next service start recovers it.
- Five waves from the page, no load, no water (bags on the lab PC, `~/bags/osc_*`): drive command
  = −1.000 × setpoint; lag about 180 ms; stroke 97 % at 2 s, 90 % at 1 s, the same at 10 and
  20 mm (it depends on the period, not on the jerk limit S-0-0193). Stop holds in place; Return
  to upright works.
- Test 1, kill test with both rigs running (2026-11-10): the MGate's fault timer runs per slave.

### Open

- [ ] **Test 2: gateway load with both rigs** ([procedure](#test-2-gateway-load-with-both-rigs-next)).
  Gates the code for running both rigs (TODO 47).
- [ ] With the load and water: repeat the brake test (the slide must not drop at mid-height) and
  5 and 20 mm at 2 s, then a sweep at a fixed stroke (for example 10 mm at 3, 2, 1.5, 1, 0.8 s; a
  `generate_oscillations` file played with `play_dat`), watching current and torque in IndraWorks.
  All runs so far were unloaded: a baseline, don't tune or compensate from them.
- [ ] Drive tuning with the drive's owner (Robert, from about 2026-11-23): Kv (S-0-0104 = 1.0),
  acceleration feedforward, jerk S-0-0193 (3 m/s³) for faster strokes. TODO item 46.
- [ ] P-0-4088 = 0x002A makes a bus failure only a warning (the last command is kept); 0x0028
  would make it an error. Decide with the drive's owner (see *Side findings*).
- [ ] Brake under load: does the stop reaction (P-0-0119 = 0, "best possible deceleration")
  brake the slide before the brake engages, and where does the slide rest when disabled?
- [ ] Is the axis counterbalanced, or does the motor hold the load? The brake is 5 Nm (about
  1.5 kN at 20 mm/rev).
- [ ] Set up slave 3 from `RX060107.gsd` instead of RX03 with edited user parameters, so the MGate
  also treats it as Fail_Safe? Add the newer GSD set to `resources/profibus_gsd/`?
- [ ] The soft stop on GPIO17: the vertical rig uses it (released at every start, 2026-11-09). Is
  the button within reach of the vertical rig, or only at the wavemaker?
- [ ] Export the phase 2 MGate configuration (slave 2 `80 00 08`, slave 3 `C0 00 08`, both 500 ms
  timeouts) to the stick as a backup.

### The oscillator drive (read 2026-11-06)

| | Value | Meaning |
|---|---|---|
| PROFIBUS address | 3 since 2026-11-10 (2 before) | MGate slave 3, inputs at register 7, outputs at 6; the wavemaker is slave 2 at 0 / 0 |
| Scaling type | linear (the drive converts motor turns to mm) | `drive_motion: linear` |
| Position resolution | 0.0001 mm | Tag factor 0.0001 → the "deg" tag is **mm** |
| Velocity resolution | 0.001 mm/min | Tag factor 0.0001 → the "rpm" tag is **10 mm/min** per unit (1 mm/s = 6 units) |
| Acceleration resolution | 0.001 mm/s² | Not in the cyclic data; the drive's own positioning acceleration applies |
| Torque/force resolution | 0.1 % | Not in the cyclic data |
| Position limits | −211 … +211 mm | The drive's backstop; the controller's travel goes inside it |
| Current position | about 0 mm | Mid-travel |
| Direction (measured 2026-11-09) | **+ command = down** | The rig uses + = up: `drive_direction: -1` |
| Scaling check (2026-11-09) | `--move` +5: drive 4.925 mm, sensor 4.940 mm at 1.68 mm/s | `drive_m_per_unit: 0.001` and `drive_velocity_per_unit_per_s: 6.0` confirmed (*First move*) |
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
| P-0-4088 master communication config | 0x002A | not in the file (default 0) | Bits 15/14 = 00: commands from PROFIBUS. **Bits 2/1 = 01: a bus failure is only a warning (E4xxx) and the last command is kept** (Reference Book R911297317-08, p. 1335). See *Side findings* |
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
| S-0-0193 positioning jerk | **3000 mm/s³ (3 m/s³)** | 10 000 rad/s³ | Expected to limit the oscillation; below 3 m/s³ it did not |
| P-0-0525 / P-0-0540 holding brake | 0x0004 / **5 Nm** | 0x0000 / 0 Nm | Brake configured; about 1.5 kN at 20 mm/rev |
| P-0-0119 best possible deceleration | 0 | 0 | Check its meaning for a vertical axis |
| S-0-0273 max drive-off delay | 10 000 ms | | — |
| S-0-0108 feedrate override (stored) | 0 % | | `enable()` writes 100 % (`indradrive.cpp:197`) |
| S-0-0104 Kv, P-0-0040 velocity feedforward, P-0-0099 command smoothing, P-0-0041 average filter | 1.00 (1000/min), 100 %, 0, 0 | | The tracking lag (*Waves from the page*) |

### Procedures

E-stop within reach for every step that can move the slide.

1. **Monitor.** `mgate_example --input-base 7 --output-base 6 --profibus-address 3` must print
   `live`, a position near 0 that follows a jog from IndraWorks (mm), and the diag number
   IndraWorks shows.
   **The monitor is not read-only.** It writes the whole output block every cycle
   (`write_only_dirty = false`), starting with control word 0 (drive off). Never run it while a
   controller runs. A jog from IndraWorks during the monitor needs IndraWorks to take control
   (e.g. easy startup mode), because the bus is commanding drive off. After every run the drive
   shows A0050 (parameter mode); the next `enable()` switches back to operating mode.
2. **Brake:** switch the drive off at standstill at mid-height; the slide must not drop.
3. **Driver-level test move, no controller running:**

   ```bash
   ~/ros_ws/build/mgate5101_driver/mgate_example --move --profibus-address 3 --input-base 7 --output-base 6
   ```

   The slide must go 5 mm up or down and back at about 1.7 mm/s, then the drive switches off
   with the brake holding. Ctrl-C mid-move must halt and disable. A 5 mm move confirms the
   position factor (`drive_m_per_unit: 0.001`); about 3 s per move confirms the velocity factor
   (5 mm at 100 mm/min).
4. **Dry run of a file:**

   ```bash
   sudo systemctl start oscillator@ladertanken
   ros2 run rig_controller generate_oscillations vertical session.csv -o session_osc
   ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator --dry-run
   ```

   The first line must show the travel (−0.2…0.2 m) and upright from the yaml, and
   `namespace /oscillators/vertical`.
5. **Fault timeout:** with the drive enabled, kill the controller
   (`sudo pkill -9 -f '[c]ontroller_node.*oscillators'`; without the brackets the pattern also
   matches its own `sudo`). The drive must switch off within about 500 ms (MGate fault timeout
   on slave 3, item 5) and the brake must hold the slide. Afterwards the drive stays in C0401
   (status word `OM Ab`); the next service start recovers it (`enable()` → A0108,
   `SW=0xC81E OM AF` in 0.3 s). The service stop after a `kill -9` takes about 45 s: the
   lifecycle manager waits for the dead controller and is killed (SIGINT 20 s, SIGTERM 25 s,
   then SIGKILL; the unit ends `Failed with result 'exit-code'`).
   Switching off by hand after a dead controller, if the timeout ever fails: `systemctl stop`,
   then `mgate_example` for 1-2 s (it writes control word 0), or the E-stop.
6. **A recorded run**, from the lab PC (the PC sees domain 1; no copying afterwards):

   ```bash
   # lab PC
   source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=1
   ros2 bag record -o ~/bags/<name> --topics /oscillators/vertical/oscillator_actuator_setpoint \
     /oscillators/vertical/oscillator_position /oscillators/vertical/oscillator_setpoint \
     /oscillators/vertical/oscillator_velocity /oscillators/vertical/oscillator_state
   # Pi
   ros2 run rig_controller play_dat session_osc.dat vertical --rig oscillator
   ```

   Ctrl-C cancels the current run and stops the program. Increase stroke and frequency step by
   step, watching the stroke and lag. Keep files under 3 m/s³ jerk (`generate_oscillations`
   warns above it).

### Test 2: gateway load with both rigs (next)

**Question:** do two controllers on one MGate (300 requests/s) cause the ~50-210 ms gateway
stalls? Those stalls appeared on 2026-10-07 at the same rate from one client polling every 10 ms.
A stall freezes both rigs' setpoints. Over 500 ms, it would switch a drive off. If it fails, the
code remedies are in [oscillator_sketch.md](oscillator_sketch.md#if-test-2-fails).

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

**Alternative without the rigs: probe load test.** Both probes write control word 0 to slave 2's
block 0 / 0 only, so both controllers must be stopped and both drives off:

```bash
nstat -az TcpRetransSegs
ros2 run mgate5101_driver poll_rate_probe --host 192.168.1.70 --seconds 60 --intervals 20 > ~/probe_a.txt &
ros2 run mgate5101_driver poll_rate_probe --host 192.168.1.70 --seconds 60 --intervals 20 > ~/probe_b.txt
wait
nstat -az TcpRetransSegs
cat ~/probe_a.txt ~/probe_b.txt
```

Pass: both report 20 ms as usable, no ~210 ms stalls, TcpRetransSegs about unchanged.

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

**While running both:** the soft stop button stops only the wavemaker (GPIO17), so keep the
E-stop within reach. Nothing in this test commands the oscillator to move.

### Log (2026-11-09 – 2026-11-10)

#### Under the controller (2026-11-09)

`oscillator@ladertanken`, `OSCILLATOR=vertical`, commit `cecf951`: configure and activate clean,
`Drive status: SW=0xC81E OM AF inpos pos=0.0814` (the target parked on the position, nothing
moved), all nodes in `/oscillators/vertical`, the page connected and idle. `systemctl stop` took
4.5 s with the ordered shutdown (bridge, then controller, 0.33 s) before SIGINT.

#### Fault timeout (2026-11-09)

**First try failed:** after `kill -9` the drive stayed at A0108 (enabled, holding position)
because the MGate's output fault timeout was 0 (off).

**Passed after the fix:** in MGate Manager, slave 2, slot 4 (`0xE5 Output 6 Words`, output
0…11) had timeout 0; set to 500 ms, fault value all `00` (the user parameters `C0 00 08`
untouched). `kill -9` on the controller: A0108 → C0401 (the same code as after control word 0
from `mgate_example`) at once, the brake holding: the drive off without anything on the Pi.
Since phase 2 each slave has its own setting (slave 3: the oscillator). The timer runs per slave:
another rig's writes do not keep a dead rig alive (*Test 1*). Only one Modbus client may write a
slave's output block while its rig runs. Passed again on the daisy chain 2026-11-10.

#### Waves from the page (2026-11-09, unloaded)

All five without load and without water (bare slide): a no-load baseline. Direction, scaling,
Stop and return are right, and a period-dependent loss exists even unloaded.

- **Wave 1, 16:14** (height 0.005 m, period 2 s, about 5 periods; bag `~/bags/osc_first_wave`
  on the lab PC, recorded from the PC over domain 1): `oscillator_actuator_setpoint = −1.000 ×
  setpoint` (mm), so `drive_direction: -1` and `drive_m_per_unit` act as intended; position
  follows with a lag of about 160 ms (8 control cycles; 0.07 mm rms after the shift, 0.87 mm
  without); stroke 4.87 of 5.00 mm (97 %). The wave started from 0.92 mm, where the earlier
  unrecorded run had stopped. Stop on the page halts in place (by design, README *Check and
  play*); *Return to upright* brings it to 0.
- **Wave 2, 16:16** (height 0.02 m, period 2 s, 34.6 s; bag `~/bags/osc_wave_20mm_2s`): actuator
  = −1.000 × setpoint; lag about 180 ms (9 cycles; 0.28 mm rms after the shift); stroke 19.46 of
  20.00 mm (97.3 %), steady from the 2nd period. Both waves start up (about +2 mm), then the ramp
  grows. Stop at +8.1 mm held there; *Return to upright* 8.1 → 0 mm in 1.0 s (setpoint peak
  15 mm/s, under `return_to_upright_max_velocity_mps: 0.02`), the position 0.4 mm behind at the
  last sample.
- **Wave 3, 16:19** (height 0.02 m, period 1 s, 10 periods; bag `~/bags/osc_wave_20mm_1s`;
  computed jerk 2.5 m/s³, drive limit 3): lag again about 180 ms (now 65° of phase); stroke
  **17.9 of 20.0 mm (89.7 %)**, symmetric (±8.95 mm), steady from the 2nd period.
- **Wave 4, 16:21** (10 mm, 1 s, jerk 1.2 m/s³; bag `~/bags/osc_wave_10mm_1s`): 89.7 %, lag
  180 ms, the same as 20 mm at 1 s. **The loss depends on the period, not on the jerk limit:**
  linear, a fixed 180 ms lag. Suspect: the positioning-speed cap, peak speed ×
  `positioning_velocity_margin` (default 1.2).
- **Wave 5, 16:32** (10 mm, 1 s, `positioning_velocity_margin: 2.0` on the Pi only; bag
  `~/bags/osc_wave_10mm_1s_m2`): 89.7 % again, lag 160 ms. **The speed cap is not the cause.**
  The two periods fit one first-order lag: τ = 76 ms at 2 s, 78 ms at 1 s (amplitude
  1/√(1+(ωτ)²)), plus about 100 ms of pure delay. Kv S-0-0104 = 1.00 (1000/min) gives a
  position-loop time constant of 60 ms if it acts without feedforward. Open: where the 77 ms
  comes from (Kv with the drive's interpolation of a target that steps every 20 ms is the
  candidate), and whether to fix it in the drive (with its owner) or compensate in the
  controller or the files ([oscillator_sketch.md](oscillator_sketch.md#open-questions)).

#### Phase 2: daisy chain (done 2026-11-10)

Both drives on one line, one rig running at a time. The oscillator moved to address 3; the
wavemaker and `wavemakers.yaml` are unchanged.

**Result:**
- MGate: slave 2 (wavemaker) user parameters `80 00 08`, input bytes 0-13, output 0-11; slave 3
  (oscillator) `C0 00 08`, input 14-27 (register 7), output 12-23 (register 6). Both: modules
  ParamCh not used ×2, Input 7 Words, Output 6 Words, output timeout 500 ms, fault value 0.
  Live list `0x0C` (addresses 2 and 3).
- `oscillator.yaml`: `input_base_reg: 7`, `output_base_reg: 6`, `profibus_address: 3`.
- Vertical run 1 of `rigtest.dat` played over the chain; the wavemaker's block untouched.
- Kill tests (`kill -9` on `controller_node`, both blocks logged at 10 Hz, read only): the
  oscillator left `AF` about 0.6 s later (→ C0401), the wavemaker about 0.5 s later (→ `0x0888`
  parameter mode, as after a service stop; its slave has no Fail_Safe). The other drive's block
  unchanged both times.
- `oscillator@` refused while `wavemaker@` runs.

**Learned on the way:**
- The MGate assigns each slave's bytes when it is added and never moves them. After slave 2 was
  re-added it landed behind slave 3; delete both and add slave 2 first, then 3. The start
  addresses cannot be typed in.
- The MGate's scan fills user parameters from the old GSD (`80 00 08`); slave 3 needs
  `C0 00 08` typed in.
- MGate status `diag` with octet 1 `01` (`station_non_existent`) and P-0-4073 `baud search` on
  the drive: no bus signal at the drive (here a wrong cable at the oscillator).
- A cleared bus fault: the wavemaker showed F4009 after being powered off on the chain; clear it
  before starting its service.

What was done, in order:
1. Change the address, wire the chain, and set termination at both ends only. Chain wired
   2026-11-10: MGate (termination ON) → wavemaker (OFF) → oscillator (ON). The oscillator's
   P-0-4025 was set to 3 in IndraWorks. While the MGate is connected, IndraWorks refuses the
   write (it says the parameter is write protected) and can't switch the phase: the master holds
   the drive in phase 2 (`mgate_example`: `PM`), and the write needs phase 4. It works with the
   PROFIBUS plug pulled from the drive: pick "phase switch to phase 4", write, save, then
   power-cycle the control voltage.
2. Add slave 3 on the gateway and read the address map.
3. Edit the oscillator's yaml (`input_base_reg`, `output_base_reg`, `profibus_address`).
4. The phase 2 code (T1, the `mgate_example` options).

### PROFIBUS diagnosis log (2026-11-06 – 2026-11-09)

**Status: solved 2026-11-09.** The oscillator drive (MPB-05) requires a Fail_Safe master and
rejected the parameter telegram with code 137 = "FailSafe not supp. by contr.". Slave 2's user
parameters `80 00 08` → **`C0 00 08`** fixed it (since phase 2: slave 3 has `C0 00 08`, the
wavemaker's slave 2 is back at `80 00 08`): P-0-4073 `Data-Exch WD+`, `slave=live`. See
*Cause and fix* below; the sections after it are the 2026-11-06 search, kept for reference.

#### Cause and fix (2026-11-09)

- A newer Rexroth GSD set (`Profibus/` on the stick: RX04, RX06, RX08, RX16, RX17 and a new
  `Info_eng.txt`) says: *"For the firmware release MPX05 the GSD File RX060107.GSD (GSD-Revision
  5) has to be used. Older GSD Files (RX010107.GSD/RX040107.GSD) can not be used for the
  firmware release MPX05."* The repo's `resources/profibus_gsd/` only has the 2006 set.
- `RX060107.gsd` defines `Value(137)="FailSafe not supp. by contr."`. Against `RX030107.gsd` it
  sets `Fail_Safe=1` (was 0) and the first user parameter byte `0xC0` (DPV1 + Fail_Safe; was
  `0x80`). The modules and their identifiers are the same (`00 00 D6 E5`).
- Fix applied: on slave 2, still set up from `RX030107.gsd`, user parameters `C0 00 08`.
  **Gap:** the MGate still treats the slave as `Fail_Safe=0`; only the drive is told. The kill
  test showed the MGate's Clear state turns the drive off. Setting up the oscillator's slave (3
  since phase 2) from `RX060107.gsd` is the proper form (*Open*). Re-adding it from RX03
  silently restores `80 00 08`.
- The wavemaker (MPB-04) never ran with `C0 00 08`: since phase 2 its slave 2 has `80 00 08`
  again, as `Info_eng.txt` (`RX040107.gsd`, Fail_Safe=0) expects.

#### After the fix (2026-11-09)

- **`wait_for_data()` race** in `MGateDriver`: `mgate_example --move` failed at once with
  "gateway offline" and `slave=DOWN`. Fixed in code (oscillator_sketch.md, *Done*). The
  controller was not affected (it waits 200 ms).
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

#### Symptom

`mgate_example` on the Pi: `slave=DOWN`, status word, position, velocity and diag all 0. The
Modbus link is fine (207 cycles, no read or write errors, about 2.8 ms per cycle). The gateway
status is `0003` (Operate), with an empty live list.

#### Checked and ruled out

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

#### What the drive reports

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

#### Tried before the cause was found (2026-11-06 – 2026-11-09)

- User parameters `00 00 00` (DPV1 and structure flag off): identical diagnosis, code 137. The
  Fail_Safe bit was 0 in every try.
- Windows PC searched for newer GSD files (with and without the KUNBUS folders): only the 2006
  Indramat set; KUNBUS Configurator III holds no Rexroth GSD. IndraWorks Ds 14V24 ships none.
- The cRIO master's FPGA build (`C:\NIFPGA\compilation\ladertanken_...CScRIO-PBDP-Mast...`):
  no slave configuration in it.
- gsd-db.org / gsd-db.com unreachable; Rexroth support not available.

#### Side findings

- **P-0-4088 = 0x002A is a safety question for a vertical axis.** Bits 2/1 = 01 make a bus
  failure only a warning and keep the last command (hence E4005). With Fail_Safe now on, the
  drive's reaction to the master's Clear state also goes through these bits. The wavemaker has the
  default 00: bus failure is an F4xxx error with the drive's error reaction. The controller's
  fault path (control word 0, MGate fault value) relies on that. Proposal: 0x0028; it changes
  the drive's existing setup (*Open*).
- **The MGate's output fault timeout was off** (timeout 0 on slave 2's output module). Set to
  500 ms on 2026-11-09 (*Fault timeout*).
- `S-0-0051` (oscillator) vs `S-0-0386` (wavemaker) as the position input: same length,
  same value with one encoder.

### Backups (USB stick `LADERSCHTIC`)

- `Mgate Backup/mgate_ladertanken_wavemaker_OK_2026_11_06.dat`: gateway configuration with
  the wavemaker working, including GSD files.
- `Default.par` (oscillator), `WMDefault.par` (wavemaker): IndraWorks exports, 2026-11-06.
- `Profibus/`: the newer Rexroth GSD set (RX04-RX17) with `Info_eng.txt`.
- `TestOscillator.bin`: sensor log of the first move (2026-11-09).
- To do: the phase 2 MGate configuration (*Open*).

## Verified so far

Kept as a record of physical checks already done.

- Emergency stop is accessible, tested, and removes actuator power.
- Mechanical guards, couplings, linkage, attachment and fasteners are secure.
- Full travel is physically clear (wavemaker, actuator, tank walls, personnel area).
- The actuator is mounted rigidly, without backlash or free play.
- Wavemaker type, flap attachment height and water depth match `wavemakers.yaml`.
- Drive limits −550° / 500° are configured in IndraWorks and are kept there by decision (2026-10-07); the controller's ±275° is the travel limit (±200° until 2026-10-20).
- No mechanical end stops. The lab waves above have been run on this hardware at 1 m depth (Pål Lader, 2026-10-08), which by linear theory needs about ±9.3 cm at the flap top; the earlier ±5 cm figure (2026-10-07) was too low.
- `upright_position_m` matches the physical upright: 0.0 m for ladertanken.
- Activation caused no physical motion.
- Small position oscillation at standstill (the servo holding position) is accepted; it shows as larger velocity oscillation, but the controller only uses position. Measured below ±0.5° (≈ ±0.27 mm on ladertanken), well inside the 2 mm return-to-upright tolerance (≈ 3.8°).
- The flap was disconnected for the first motor tests (2026-10-07) and reconnected for the lab waves (2026-10-08).
- Calibration: 5–10 positions used, measured at the flap top, which is at `flap_attachment_height`; scale `b = 0.053 / 100 = 0.00053 m/deg`; offset `a = 0.0 m`; actuator zero = upright (`actuator_upright_angle_deg: 0.0`).
- The PC reaches the gateway at 192.168.1.70 over the wired network.
- Motor nameplate (ladertanken): Rexroth MSK071E-0450-NN-M1-UG0-NNNN, 3-phase permanent magnet; n max 6000 min⁻¹ (4500 rated), MdN 23.0 Nm / IdN 20.0 A natural convection (34.5 Nm / 30.0 A surface cooled), Km 1.29 Nm/A, KE 82.7 V/1000 min⁻¹, 23.5 kg.
- Poll interval (2026-10-07): `poll_interval_ms` is 20 ms (50 before). Normal gateway cycles take 4–8 ms with no read/write errors or reconnects. The 10 s sweep had single cycles of about 210 ms at 40, 10, 5 and 2 ms; at 2 ms nearly every cycle stalled, so the stalls grow with polling rate (the gateway, or packets lost under load). A 60 s run at 20 ms had none (3000 cycles, max 8.1 ms). Stalls during waves are watched in item 13.
- First motion, flap disconnected (2026-10-07): wave `amplitude: 0.0005` (the action's old field, H/2; now `height: 0.001`), `period: 10.0` (about ±1 cm at the flap top, ±19.5°); `wavemaker_position` follows `wavemaker_setpoint` with the same sign and size.
- Larger wave, flap disconnected (2026-10-07): `amplitude: 0.002` (now `height: 0.004`), `period: 10.0` runs as calculated (first movement positive, peaks about ±79°, ±4.1 cm at the flap top); Ctrl-C stops and holds.
- Terminal-mode launch, interface inspection, configure without activate, and goal rejection while inactive.
- Lab waves, flap connected, 1.01 m water depth, limits ±200° and 170 rpm (2026-10-08). All eight steps ran, each followed by `return_to_upright` with the flap upright afterwards. In order of rising acceleration, as `height` [m] / `period` [s] (run with the action's old `amplitude` field, H/2; lab steepness; expected drive angle and peak speed by linear theory): 0.026 / 1.0 (1/60; ±21°, 22 rpm), 0.049 / 1.2 (1/45; ±45°, 40 rpm), 0.074 / 1.2 (1/30; ±68°, 60 rpm), 0.108 / 1.5 (1/30; ±132°, 92 rpm), 0.051 / 0.7 (1/15; ±35°, 53 rpm), 0.104 / 1.0 (1/15; ±83°, 87 rpm), 0.148 / 1.2 (1/15; ±137°, 119 rpm), 0.171 / 1.3 (1/15, the largest lab wave; ±173°, 139 rpm). Wave heights were not measured (item 33).
- `cancel_all_goals` stops a running wave, flap connected (2026-10-08).
- Flap geometry (2026-10-20): the pivot is about 9 cm above the tank floor, with the wall below it closed (`hinge_height: 0.09`; it was 0.0 before). The flap top can travel 52 cm forward of upright (≈ 981° at 0.00053 m/deg), so the mechanism is not the forward limit: the drive's 500° (26.5 cm) and the controller's 275° (14.6 cm) are. The backward limit has not been measured. Which drive direction is forward has not been confirmed (item 3).
- Wave heights at 1.3 s with the provisional gain table, `hinge_height: 0.09` and limits ±275° / 240 rpm (2026-10-20, 1.0 m depth): 0.137 m (1/19) measured 0.13977 m (+2.0 %); 0.171 m (1/15, the largest lab wave; ±248°, 200 rpm, the largest stroke run so far) measured 0.1649 m (−3.6 %). The mechanics showed no problems.
- Gain calibration with WP1 (2026-10-20, `Calib20oct.BIN`, 24 runs at 0.6–2.0 s, 1/60–1/20, 1.0 m depth): new gain = gain used × measured / target. 0.9–2.0 s is flat at 0.76 ± 3.7 % for 1/60–1/28; 0.6 s gives 0.64 and 0.8 s about 0.70; 1/20 waves come out about 4 % lower. Repeats of 1.0 s / 0.026 m at the start and end differ by 6 %; the pauses between runs (8–20 s) were too short for the tank to calm. Table set to one point per period: [0.6, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0] → [0.64, 0.70, 0.78, 0.80, 0.71, 0.76, 0.76, 0.75, 0.75, 0.75, 0.74, 0.76].
- Gain table check (2026-10-21, 1.0 m depth, gain table as merged): (measured − target) / target for 0.026 m / 1.0 s (1/60) 0 % at the start, middle and end of the session; 0.052 m / 1.0 s −1.0 %; 0.112 m / 1.5 s 0 %; 0.020 m / 0.5 s (outside the table, 0.64 used) +17.5 %; 0.026 m / 0.7 s −46.2 %; 0.057 m / 1.05 s −14.0 %; 0.063 m / 1.1 s −16.7 %; 0.069 m / 1.15 s −10.9 %. The table points at 1.0 s and 1.5 s hold, and the repeat scatter was zero, but 0.7 s and 1.05–1.15 s come out far too low (see item 33).
- The lab Pi runs everything (2026-10-12): `ladertanken-rpi` (Ubuntu Server 24.04, ROS 2 Jazzy, default Fast DDS, ROS domain 1) on the wired lab network at 192.168.1.98 runs `lab.launch.py` (controller, bridge, lifecycle manager, web page), started over SSH; the web page works from the lab network. The PC is no longer needed for the gateway.
- Gateway from the Pi over the cable (2026-10-12): cycles 2.7 ms (p99 about 3 ms), no read/write errors or reconnects; a second sweep had no stalls at 20–50 ms. The ~50–210 ms stalls at faster polling come with TCP retransmissions (`TcpRetransSegs` +54 during one sweep): the gateway loses packets when polled fast. `poll_interval_ms: 20` stays.
- Lifecycle manager, controller only (`enable_lifecycle_manager:=true bond_timeout:=1.0`), flap connected (2026-10-08): the manager configures and activates the controller by itself; a wave runs; PAUSE (`manage_nodes` command 1) during a wave aborts the goal, deactivates the controller and disables the drive; RESUME (2) reactivates it, `return_to_upright` and a new wave work; SHUTDOWN (4) finalizes the controller with the drive disabled. The 1 s bond timeout did not trigger falsely. Not tested: a crashed controller (needs the MGate fault timeout first, item 5).

## Actuator conversion calibration

Calibrate both the scale and the zero offset before setting `actuator_lead_m_per_degree`. For several safe
actuator angles, record the angle and the physical flap position in the same coordinate system as the
wavemaker limits, and fit `x = a + b * theta`:

| Actuator angle (degrees) | Physical flap position (m) |
| ---: | ---: |
| theta_1 | x_1 |
| theta_2 | x_2 |
| ... | ... |

If the fit has significant residual error, use a geometry-based conversion or a calibrated lookup table
instead of one constant scale.

## Transfer gain calibration

Finds the gain per wave period for item 33. The controller computes the stroke for a regular wave
from linear theory and divides it by the gain, so for each run

    new gain = gain used × measured height / target height

Measured above 1 the waves came out too high, and the higher gain shrinks the next stroke.

### Before you start

- Items 3, 13, 14 and 34 are done: the flap moves the right way, follows the setpoint at the
  periods to be calibrated, the angle-to-position conversion is right and the springs suit the
  water depth. A tracking or conversion error would otherwise end up in the gain.
- `water_depth` in `wavemakers.yaml` is the depth measured today, and the water is still.
- The beach or absorber is in place, as it will be for experiments.
- The wave probes have a fresh static calibration (raise and lower each probe by known steps in
  still water, at least 5 points over the expected range; check that the fit is linear).
- Probe placement:
  - Put the main probe on the tank centreline at least 3 water depths from the flap. Closer than
    that, the local (evanescent) waves near the paddle add to the reading.
  - Optional: put a second probe across the tank at the same distance. If the two differ by more
    than a few per cent at some period, there are cross waves; note those periods.
  - Optional: 3 probes along the tank (Mansard & Funke spacing) separate the incident wave from
    the reflected one, which allows long records.
- Record the depth, the probe positions and the beach state in the results table below.

### Run the waves

1. Start in terminal mode with the gain set to theory, so "gain used" is 1.0 for every run.
   Set all `wavemaker_transfer_gain_values` in `wavemakers.yaml` to 1.0 (the starting table), or
   comment out both table lists and set `wavemaker_transfer_gain: 1.0`. When the table is set,
   it takes the place of `wavemaker_transfer_gain`. Then launch, configure and activate as in
   [Rig test procedure](#rig-test-procedure).
2. Record the paddle alongside the probes:
   ```bash
   ros2 bag record $NS/wavemaker_setpoint $NS/wavemaker_position
   ```
3. Test matrix:
   - Periods: cover every period used in experiments, plus one point just beyond each end, so no
     experiment falls outside the table. Use steps of 0.1 s where the gain changes quickly
     (usually the short periods) and up to 0.2–0.3 s elsewhere.
   - Heights: two per period, at steepness H/L ≈ 1/60 and 1/30, where L is the wavelength at
     that period and depth. Steep waves (1/15) are nonlinear and are not used to set the gain.
     - Keep every wave at least 20 mm high. Smaller waves are too small to measure well, because
       probe resolution and the servo's standstill wobble become a large part of them. At short
       periods use 1/30 and 1/20 instead.
     - Where 1/30 needs more than about 150° (three quarters of the travel limit), use 1/45 instead.
     - Short waves lose height along the tank, so put the probe where experiments measure.
   - Ladertanken at 1.0 m depth (angle and peak speed by linear theory at gain 1.0; limits
     ±200°, 170 rpm when computed; now ±275°, 240 rpm). Recompute for another depth or wavemaker:

     | T [s] | L [m] | Low H [m] | Steepness | Angle, speed | High H [m] | Steepness | Angle, speed |
     | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
     | 0.6 | 0.56 | 0.020 | 1/28 | ±13°, 24 rpm | 0.028 | 1/20 | ±19°, 33 rpm |
     | 0.7 | 0.77 | 0.020 | 1/38 | ±14°, 21 rpm | 0.039 | 1/20 | ±27°, 41 rpm |
     | 0.8 | 1.00 | 0.020 | 1/50 | ±15°, 19 rpm | 0.033 | 1/30 | ±24°, 31 rpm |
     | 0.9 | 1.26 | 0.021 | 1/60 | ±16°, 19 rpm | 0.042 | 1/30 | ±32°, 37 rpm |
     | 1.0 | 1.56 | 0.026 | 1/60 | ±21°, 22 rpm | 0.052 | 1/30 | ±42°, 44 rpm |
     | 1.1 | 1.88 | 0.031 | 1/60 | ±27°, 26 rpm | 0.063 | 1/30 | ±55°, 52 rpm |
     | 1.2 | 2.23 | 0.037 | 1/60 | ±35°, 30 rpm | 0.074 | 1/30 | ±69°, 61 rpm |
     | 1.3 | 2.60 | 0.044 | 1/60 | ±45°, 36 rpm | 0.088 | 1/30 | ±90°, 73 rpm |
     | 1.4 | 2.97 | 0.050 | 1/60 | ±56°, 42 rpm | 0.100 | 1/30 | ±113°, 84 rpm |
     | 1.5 | 3.35 | 0.056 | 1/60 | ±70°, 49 rpm | 0.112 | 1/30 | ±139°, 97 rpm |
     | 1.7 | 4.11 | 0.068 | 1/60 | ±102°, 63 rpm | 0.091 | 1/45 | ±136°, 84 rpm |

     That is 22 runs. Use 1.0 s / 0.026 m as the repeat condition. Also run two lab waves at
     1/15 (0.104 m / 1.0 s and 0.171 m / 1.3 s) after the table is set, to see how far the
     calibrated gain is off for steep waves.
   - Repeat one middle condition at the start, the middle and the end of the session. This gives
     the scatter and shows drift, for example from the water level changing.
4. For each run, send the wave and let it run long enough to fill the analysis window (next
   section), then stop it:
   ```bash
   ros2 action send_goal $NS/move_wavemaker rig_interfaces/action/MoveRig \
     "{height: 0.026, period: 1.0}"                 # Ctrl-C cancels
   ```
   The controller rejects a wave that would exceed the travel or speed limits; that period and
   height cannot be run, so note it and continue. Wait until the probe is back within about
   1 mm of still water before the next run.

### Analyse each run

1. Analysis window: use only waves at full height that have not yet been disturbed by the
   reflection.
   - Start: the paddle reaches full stroke one period after the start, and the waves then travel
     at the group velocity `cg`. Start at about `T + x / cg + 3 T`, where x is the distance from
     the flap to the probe.
   - End: the reflection from the beach arrives at `(2 L_tank − x) / cg`, where L_tank is the
     distance from the flap to the beach.
   - Group velocity: `cg = (ω / k) · ½ · (1 + 2kh / sinh 2kh)`, with k from the dispersion
     relation `ω² = g k tanh kh`.
   - Aim for 10 or more whole periods in the window. If the window is shorter, move the probe
     closer to the flap (but no closer than 3 depths) or use the 3-probe reflection analysis.
2. Wave height: fit a sine at the known frequency over a whole number of periods in the window,
   and take H = 2 × amplitude. This is less sensitive to noise than single crests and troughs.
   ```python
   import numpy as np
   # t [s], eta [m]: the probe record cut to the window; T: wave period [s]
   w = 2 * np.pi / T
   A = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones_like(t)])
   a, b, _ = np.linalg.lstsq(A, eta, rcond=None)[0]
   H_measured = 2 * np.hypot(a, b)
   ```
   Fit the second harmonic (2ω) the same way. If its amplitude is more than about 10 % of the
   first harmonic's, the wave is nonlinear; do not use that run to set the gain.
3. Paddle check: fit `wavemaker_position` from the bag in the same way and compare it with
   `wavemaker_setpoint`. If the position amplitude is more than about 2 % below the setpoint
   amplitude, the drive is lagging: fix item 13 first, because that error would end up in the
   gain.
4. Ratio: r = H_measured / H_target. With gain 1.0 used, r is the gain.

### Build the table

- Per period, average r over the heights.
  - If the two steepnesses differ by more than about 5 %, the response depends on height. Use the
    value at the steepness used most in experiments and note the difference.
  - If the repeat runs scatter more than the differences between periods, the table only fits
    noise: take more runs or improve the probe setup before going on.
- If all periods are within about ±3 % of one value, one gain is enough: set
  `wavemaker_transfer_gain` to the mean and leave the table out.
- Otherwise set the table in `wavemakers.yaml`, under the wavemaker's controller:
  ```yaml
      # Transfer gain calibration <date>, water depth <m>; see COMMISSIONING.md.
      wavemaker_transfer_gain_periods: [0.6, 0.7, 0.8, 1.0, 1.2, 1.5, 1.7]   # [s], ascending
      wavemaker_transfer_gain_values:  [0.90, 0.93, 0.95, 0.97, 0.98, 1.00, 1.01]   # example values only
  ```
  - The two lists must be the same length, with at least 2 points; the periods must ascend
    strictly, and all values must be finite and above 0.
  - Between points the gain is interpolated linearly. Outside the table the nearest end value is
    used and the controller logs a warning, so any period that shows that warning in an
    experiment is outside the calibration.
  - The yaml is only read at launch. To try a table without relaunching, set both lists while the
    controller is unconfigured (deactivate, then cleanup), then configure and activate again:
    ```bash
    ros2 param set $NS/controller wavemaker_transfer_gain_periods "[0.6, 1.0, 1.7]"
    ros2 param set $NS/controller wavemaker_transfer_gain_values "[0.90, 0.97, 1.01]"
    ```
    A configure that fails logs why (lengths, order or values).

### Verify

Relaunch with the new table and run about four waves:

- two at table periods;
- two halfway between table points, where the interpolation is least certain;
- each at the main experiment steepness.

The measured height should be within ±3 % of the target (or within the repeat scatter, if that
is larger). Where a between-points run misses, add that period to the table.

### Results

Keep one table per calibration, including the depth, and copy the final gains into *Verified so
far* with the date.

| Date | Depth [m] | Probe x [m] | Period [s] | H target [m] | H measured [m] | r | Paddle amp. / setpoint | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| | | | | | | | | |

## Rig test procedure

Terminal mode, ladertanken; replace the name for another wavemaker. Do not start motion until the
*before motion* items are done.

```bash
cd ~/ros_ws && source install/setup.bash
NS=/wavemakers/ladertanken

# communication only, no motion (controller stopped)
ros2 run mgate5101_driver poll_rate_probe --seconds 10

# launch (controller only, lifecycle by hand)
ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken
ros2 lifecycle set $NS/controller configure
ros2 lifecycle set $NS/controller activate          # enables the drive

# watch
ros2 run rqt_plot rqt_plot $NS/wavemaker_setpoint/data $NS/wavemaker_position/data

# small slow wave, then stop
ros2 action send_goal --feedback $NS/move_wavemaker rig_interfaces/action/MoveRig \
  "{height: 0.0001, period: 10.0}"                # Ctrl-C cancels
ros2 service call $NS/return_to_upright rig_interfaces/srv/ReturnToUpright "{requester: 'cli', tolerance: 0.0}"

# finish
ros2 lifecycle set $NS/controller deactivate        # disables the drive
```

Bridge (the web page's role played from the terminal), with the controller active:

```bash
ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken with_bridge:=true
# configure + activate the controller, then the bridge ($NS/bridge)
ros2 topic pub -r 5 $NS/heartbeat std_msgs/msg/Bool "{data: true}"
ros2 topic echo $NS/wavemaker_message
ros2 topic pub --once -w 1 $NS/height std_msgs/msg/Float64 "{data: 0.0001}"
ros2 topic pub --once -w 1 $NS/period std_msgs/msg/Float64 "{data: 10.0}"
ros2 topic pub --once -w 1 $NS/start std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once -w 1 $NS/stop std_msgs/msg/Bool "{data: true}"
```

Without hardware, run `ros2 run rig_controller fake_controller --ros-args -r __ns:=$NS`
instead of the controller (options `--reject`, `--abort-after SECONDS`).
