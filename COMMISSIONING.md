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
  Confirm on the drive how the position tag is scaled (it is read as degrees and used as metres unchanged) and the velocity unit. The code change that depends on it is item 19 in [TODO.md](TODO.md).

- [ ] **33. Calibrate the wave-height transfer gain.** (rig; wave probes, flap connected)
  `wavemaker_transfer_gain` (default 1.0 = linear theory, in `wavemakers.yaml`) divides the stroke of regular waves to correct the generated height; `wavemaker_transfer_gain_periods` / `wavemaker_transfer_gain_values` replace it with a gain per period. Follow [Transfer gain calibration](#transfer-gain-calibration).
  - The gain holds for one water depth; recalibrate after changing it, or after the calibration from item 14.
  - Open (2026-10-21): 0.7 s and 1.05–1.15 s came out 11–46 % too low with the table (see *Verified so far*). Jumps that large over 0.05–0.1 s are unlikely to be the flap's transfer function. Before changing the table, check the paddle tracking in the bags (setpoint against position) and repeat 0.063 m / 1.1 s with the probe moved about a quarter wavelength (0.47 m); a large change there points to reflection. Until then, expect lower waves than targeted at those periods.
  - Not applied to pregenerated waves; decide whether it should be.

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
