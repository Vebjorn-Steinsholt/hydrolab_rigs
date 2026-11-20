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
`git show c6bba03:oscillator_sketch.md` (done).

- [Starting the rigs](STARTUP.md#oscillator-ladertanken), [Done](#done), [Open](#open)
- [The oscillator drive](#the-oscillator-drive-read-2026-11-06): the parameters
- [Procedures](#procedures): monitor, brake, test move, dry run, fault timeout, recorded runs
- [Backups](#backups-usb-stick-laderschtic)

Tests 1-3, the log of 2026-11-09/10 and the PROFIBUS diagnosis (2026-11-06/09) are done; their
steps, commands and measurements are in `git show c6bba03:COMMISSIONING.md`.

On the Pi (`ssh ladertanken@ladertanken-rpi.local`), in tmux:

```bash
cd ~/ros_ws && source install/setup.bash
```

### Starting the rigs

Every time: [STARTUP.md, *Oscillator*](STARTUP.md#oscillator-ladertanken).

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
  An IndraDrive block other than 7 in / 6 out now fails configure (T1).
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
- PROFIBUS (solved 2026-11-09): the oscillator drive (MPB-05 firmware) needs a Fail_Safe master
  and rejected the parameter telegram with code 137 ("FailSafe not supp. by contr."). Fixed with
  the user parameters `C0 00 08` on its slave. For MPx-05 Rexroth's newer GSD `RX060107.gsd` is
  the proper file (*Open*); re-adding the slave from RX03 silently restores `80 00 08`.
- Kill test (item 5): passed after the MGate's output fault timeout was set (500 ms, 2026-11-09;
  it was 0, off). After a fault switch-off the
  drive shows C0401; the next service start recovers it.
- Five waves from the page, no load, no water (bags on the lab PC, `~/bags/osc_*`): drive command
  = −1.000 × setpoint; lag about 180 ms; stroke 97 % at 2 s, 90 % at 1 s, the same at 10 and
  20 mm (it depends on the period, not on the jerk limit S-0-0193). Stop holds in place; Return
  to upright works.
- Test 1, kill test with both rigs running (2026-11-10): the MGate's fault timer runs per slave.
- Test 2, gateway load with both rigs (passed 2026-11-11): with two controllers on the MGate,
  both rigs froze for 50-210 ms, from the Pi's 200 ms minimum TCP resend time. Fixed by
  `rto_min` 20 ms on the Pi's route to the gateway, set at every rig start (T18).
- Test 3, the speed limit (passed 2026-11-12, naked load): up to 0.257 m/s and ±122.5 mm the
  stroke stayed at 97.3 % (2 s) and 98.8 % (3 s), lag 160-180 ms, no drive error. 0.3 m/s
  (`max_velocity_rpm` 1800) is safe to use; the axis allows about 1.0 m/s (ball nut), not
  tested above 0.3 m/s. TODO 51.
- Endurance (passed 2026-11-20, naked load, no water; `~/oscillator_sessions/2026-11-20_endurance`
  on the Pi, `Endurance.ASC` on the stick): one `generate_oscillations` file of 30 regular runs,
  each 600 s with 10 s pauses (2.0 s 20 mm, 1.5 s 15 mm, 3.0 s 30 mm, 2.5 s 25 mm, 1.2 s 10 mm,
  six times; at most 0.031 m/s and 0.78 m/s³), 5 h 5 min played with `play_dat` in tmux.
  - All 30 runs succeeded; no `late on the file`; run 29 started 7 ms off its time in the file.
    The journal's one `No Soft stop message received` was at the service start, before the file
    (item 36).
  - `oscillator_position` (bag): 900 036 messages, a steady 50 Hz (median 20.00 ms, p99 20.2 ms);
    the only gaps over 100 ms are the 29 pauses.
  - Pi: used memory 691 MiB at the start, 461 MiB at the end; 52.1 °C both times.
  - Slide (catman `6047 Position`, 200 Hz; a sine fit per 20 periods, ramps and 5 s skipped):
    the stroke is the same in all six repeats and from the start to the end of each run, within
    0.01 mm: 99.3 % (3.0 s), 98.8 % (2.5 s), 98.3 % (2.0 s), 95.9 % (1.5 s), 92.8 % (1.2 s).
    Run 25 against run 0 (2.0 s): 19.65 against 19.66 mm.
- Wavemaker fault value `0x0002` (2026-11-11, was all `00`): after a stop or a `kill -9` the drive
  goes straight to A0012 (operating mode, no torque) instead of parameter mode, so a start after
  the flap fell to its rest no longer trips F2074 (checked from 0° to 211°).

### Open

- [ ] With the load and water: repeat the brake test (the slide must not drop at mid-height) and
  5 and 20 mm at 2 s, then a sweep at a fixed stroke (for example 10 mm at 3, 2, 1.5, 1, 0.8 s; a
  `generate_oscillations` file played with `play_dat`), watching current and torque in IndraWorks.
  All runs so far were unloaded: a baseline, don't tune or compensate from them.
- [ ] Drive tuning with the drive's owner (Robert, from about 2026-11-23): Kv (S-0-0104 = 1.0),
  acceleration feedforward, jerk S-0-0193 (3 m/s³) for faster strokes. TODO item 46.
- [ ] P-0-4088 = 0x002A makes a bus failure only a warning (the last command is kept); 0x0028
  would make it an error. The wavemaker has the default 00 (a bus failure is an F4xxx error),
  which the controller's fault path relies on. Proposal: 0x0028. Decide with the drive's owner.
- [ ] Brake under load: does the stop reaction (P-0-0119 = 0, "best possible deceleration")
  brake the slide before the brake engages, and where does the slide rest when disabled?
- [ ] Is the axis counterbalanced, or does the motor hold the load? The brake is 5 Nm (about
  1.5 kN at 20 mm/rev).
- [ ] Set up slave 3 from `RX060107.gsd` instead of RX03 with edited user parameters, so the MGate
  also treats it as Fail_Safe? Add the newer GSD set to `resources/profibus_gsd/`?
- [ ] The soft stop on GPIO17: the vertical rig uses it (released at every start, 2026-11-09). Is
  the button within reach of the vertical rig, or only at the wavemaker?

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
| Clearance around 0 | 270 mm up, more than 1 m down (2026-11-12, by eye on the rig) | Beyond the drive's ±211 mm both ways: the drive's limits and the yaml's ±0.2 m are the travel limits, not the mechanics |
| Axis | Ball screw, about 660 mm long, driven from the motor by an AT5 timing belt (screw pulley AT5/36) (2026-11-12, photo). Gear 1:1 (S-0-0121/0122), so the lead is the feed constant, 20 mm; travel range S-0-0278 = 600 mm | Screw 20 mm outer diameter, bearing blocks at both ends (2026-11-12, photo). Critical speed (fixed–supported, root ≈17 mm, L 600 mm, 80 %) ≈5700 rpm, above the motor's 4700. The ball nut sets the limit instead: d·n 60 000-100 000 gives 3000-5000 rpm; take 3000 rpm = 1.0 m/s until the nut's datasheet says more |
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
| S-0-0104 Kv, P-0-0040 velocity feedforward, P-0-0099 command smoothing, P-0-0041 average filter | 1.00 (1000/min), 100 %, 0, 0 | | The tracking lag (*Done*: five waves from the page) |

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

### Backups (USB stick `LADERSCHTIC`)

- `Mgate Backup/mgate_ladertanken_wavemaker_OK_2026_11_06.dat`: gateway configuration with
  the wavemaker working, including GSD files.
- `Default.par` (oscillator), `WMDefault.par` (wavemaker): IndraWorks exports, 2026-11-06.
- `Profibus/`: the newer Rexroth GSD set (RX04-RX17) with `Info_eng.txt`.
- `TestOscillator.bin`: sensor log of the first move (2026-11-09).
- The MGate configuration of 2026-11-11 (both rigs working, wavemaker fault value `0x0002`) is in
  the repository: [`MGATE_config_backup/`](MGATE_config_backup/README.md).

## Recommended test order on the rig

1. Complete the physical checks above.
2. Run `poll_rate_probe` and set `poll_interval_ms`; configure the MGate output fault timeout.
3. Launch one wavemaker with the lifecycle manager off, and check communication without motion
   (`mgate_example` in monitor mode, or configure only).
4. Test the lifecycle transitions by hand and confirm that activation causes no motion.
5. Call `return_to_upright`, then send a very small, low-frequency goal.
6. Test cancel, then a second goal, deactivation while moving, communication loss and the
   emergency stop.
7. Plot `wavemaker_setpoint` against `wavemaker_position` and tune `positioning_velocity_margin`.
8. Enable the lifecycle manager only after manual testing succeeds.

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
- Small position oscillation at standstill (the servo holding position) is accepted; it shows as larger velocity oscillation, but the controller only uses position. Measured below ±0.5° (≈ ±0.27 mm on ladertanken), well inside the 2 mm return-to-upright tolerance (≈ 3.8° then, ≈ 4.8° at 0.000413 m/deg).
- The flap was disconnected for the first motor tests (2026-10-07) and reconnected for the lab waves (2026-10-08).
- Rig tests through the service on ladertanken, all passed: goal type per goal and `play_dat` (TODO 26, 2026-11-02, `e096801`); `play_dat` limits from the yaml (TODO 40, 2026-11-04, `8a458eb`); `generate_waves` (TODO 38, 2026-11-05; probes measured the same day, TODO 43); controller and driver fixes (TODO 20-25, 2026-11-06, `ca87b35`). Steps and results: `git show c6bba03:rpi/README.md`, *Rig test*.
- Calibration: 5–10 positions used, measured at the flap top, which is at `flap_attachment_height`; scale `b = 0.053 / 100 = 0.00053 m/deg`; offset `a = 0.0 m`; actuator zero = upright (`actuator_upright_angle_deg: 0.0`).
- **Recalibrated 2026-11-13:** `actuator_lead_m_per_degree: 0.000413` and `flap_attachment_height: 1.39`. The attachment is 1.30 m above the hinge, so 1.39 m above the floor, which is the reference the code uses (it had 1.30). Six positions held by a staircase motion file (±0.03, ±0.06, ±0.09 m), flap tilt read with an inclinometer on the arm's bracket: the flap turned 0.78 ± 0.03 of what 0.00053 m/deg predicted, with the drive at its setpoint to 0.1 mm (bag). Two earlier Stop-held extremes gave 0.80. The draw wire (`WM_Position`, 1.375 m above the floor) is linear but reads 0.74 of the horizontal travel there (probably a wire at an angle); calibrated against the inclinometer, use `--stroke-scale -1.342` in `transfer_function`. The gain table was replaced with values measured with the new geometry (item38 run 3, `transfer_function`); see `wavemakers.yaml` and *Check on the data* in `git show c6bba03:transfer_function_sketch.md`. Checked the same day with regular runs at 1.0 m: within 2 % at 1.0–1.6 s, 8 % low at 0.8 s, once WP1 was corrected: its catman **channel** had been scaled with 0.03981 m/V, while the sensor database has 0.0229 (as on 2026-11-05). Check WP1's channel scaling before each session; to be confirmed by a static probe calibration (2026-11-16/17).
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
