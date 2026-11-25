# Sketch: an ultrasonic wave probe on a ROS topic (HC-SR04, Arduino, RPi4)

What to build so a wave probe publishes on a ROS topic that the wavemaker's Pi can subscribe
to, and why. This is the probe that stages 1 and 2 of
[gain_estimation_sketch.md](gain_estimation_sketch.md) need.

**Status (2026-11-19): not started.** Nothing below exists yet: no firmware, no node, no rule,
no service.

Line numbers are as of `master` on 2026-11-19 (`bb73ba1`).

## Problem

The probes today (WP1 and the others) only reach catman. A run's gain is known only after the
record is exported and `transfer_function` is run on the lab PC. The gain sketch's live stages
need η on a topic, stamped, at a steady rate.

The chain:

```text
HC-SR04 ── Arduino ── USB ── RPi4: wave_probe_node ── Ethernet ── RPi5: probe_gain (gain sketch)
          echo time         probe_range, and probe_elevation      zeroes first, then reads η
                            (η = zero − range, after a zero)
```

- **Why ultrasonic first:** it is cheap and needs no analogue electronics. It measures a time,
  so there is no m/V scale to calibrate each day. Its scale comes from the speed of sound in air.
- **Fallback if it fails the check against WP1:** the probe amplifier's ±10 V output through an
  ADC (*Open*).

**What changes for the user.**

```bash
# on the RPi4, once: the rule and the service (decisions 4, 5)
ros2 topic echo /wavemakers/ladertanken/probe_range --once
ros2 topic hz /wavemakers/ladertanken/probe_range          # about 20 Hz
```

```text
[wave_probe_node] /dev/wave_probe: first line at seq 3; clock fit 1.000214 (+214 ppm), rms 0.4 ms
[wave_probe_node] seq gap 18512 -> 18515: 2 lines lost (3 in 25 min)
```

### Decisions

1. **The Arduino only measures.** It sends `seq,t_us,echo_us` lines (the firmware below).
   - The conversion to metres, the limits and the timestamps are on the RPi4, so they change
     without reflashing.
   - The speed of sound changes 0.17 % per °C. A scale error goes straight into the gain, so
     the air temperature is a node parameter, not a constant in the firmware.
2. **Timestamps come from the Arduino's clock, mapped onto the RPi4's.**
   - The node fits `t_pi = a + (1 + ε) · t_arduino` to the lower edge of the arrival times. USB
     latency only ever delays a line, so the lower edge is the true time plus a constant.
   - Each message is stamped with the mapped time, never with the arrival time.
   - `micros()` wraps every 71.6 min. Add 2³² when `t_us` goes backwards while `seq` goes
     forwards.
   - An Uno's ceramic resonator can be off by a few tenths of a percent. The fit takes that
     out, as `transfer_function` does for catman's +255 ppm.
3. **`sensor_msgs/Range`, with REP 117's conventions.**
   - `range` in metres; `+inf` for no echo (`echo_us` 0). Readings outside min/max are
     published as they are, and the subscriber treats them as dropouts.
   - `radiation_type` `ULTRASOUND`, `field_of_view` 0.26 rad (15°), `min_range` 0.02 m and
     `max_range` 4.0 m.
   - **QoS reliable, depth 100,** not the best-effort sensor default. The gain analysis needs
     every sample, and a lost sample is a gap in the FRF.
4. **A fixed device name for the port, as on the oscillator Pi.** A udev rule gives
   `/dev/wave_probe` and keeps ModemManager away from the port, as
   [rpi/99-linmot-bridge.rules](rpi/99-linmot-bridge.rules) does for the LinMot bridge. That
   rule matches any Uno R3. If the probe's board is an Uno R3 too, match by its serial number,
   in case both boards ever end up on one Pi.
5. **The RPi4 is a pure publisher.**
   - A systemd unit starts the node at boot. It is like `soft_stop@.service`, with
     `ROS_DOMAIN_ID=1` from an env file.
   - Nothing on the RPi4 talks to the controller. If the RPi4 is off, the waves run as before,
     and `probe_gain` only logs that it has no probe.
6. **Clocks between the Pis: chrony, with the RPi4 following the RPi5.**
   - A constant offset doesn't change |gain| (gain sketch, stage 1, *Clocks*), so a few ms is
     fine for stage 1.
   - Phase (the gain sketch's *Later*) needs better than 1 ms. Check `chronyc tracking` on the
     RPi4.
   - Connect the Pis by wired Ethernet, not WiFi: WiFi gives latency spikes and lost DDS
     packets.
7. **η comes from the node's zero service, called before each run** (2026-11-25; until then η
   was formed in `probe_gain`).
   - Before each run, `probe_gain` calls `zero_measurement` with `duration_s` = `STILL_S`
     (10 s), the window `check_still` already uses. The node takes the mean range over still
     water as the zero, and publishes η = zero − range on `probe_elevation`
     ([wave_probe_node_sketch.md](wave_probe_node_sketch.md), decision 9).
   - One zero, taken one way, whether a person zeroes by hand or `probe_gain` does it before a
     run. That is how the ordinary wave probes are zeroed too.
   - No mount height is needed, and slow changes in water level between runs drop out.
   - `probe_range` stays a plain distance. The zero lives only in the running node: no yaml,
     no parameter.

## Changes

1. **`firmware/hcsr04_probe/hcsr04_probe.ino`: done,** flashed to the UNO R4 WiFi (board
   `arduino:renesas_uno:unor4wifi`, not the AVR core).
   - 20 Hz on a `micros()` schedule; a period skipped in full restarts the schedule instead of
     catching up.
   - A trigger pulse of 10 µs, and `t_us` taken just before it.
   - `pulseIn(ECHO_PIN, HIGH, 30000)`: 0 on timeout, so no 1 s stall.
   - 115200 baud; one line per ping: `seq,t_us,echo_us`.
   - The header comment says opening the port resets the board. On the R4 WiFi, USB serial
     goes through its ESP32-S3, so it may not. The node copes either way (it skips the first
     line); check which `seq` the first line has.

2. **The node: `wave_probe/src/wave_probe_node.py`, in its own package `wave_probe`: done.**
   Worked out in [wave_probe_node_sketch.md](wave_probe_node_sketch.md).
   - The package is separate so the RPi4 doesn't have to build the drivers that `rig_controller`
     depends on.
   - The sketch covers the serial thread, the clock fit to the lower edge of the arrivals with
     its latency floor, the stamp at `t_trigger + echo / 2`, wraps, resets and reconnects, and
     the parameters.
   - **Topic:** `probe_range`, in the rig's namespace (`/wavemakers/<pi>/probe_range`) through
     `__ns`, as `soft_stop.sh` does.

3. **`rpi/99-wave-probe.rules`: done.** It matches the UNO R4 WiFi (`2341:1002`, serial
   `F412FA6BF3BC`), as installed on the RPi4 on 2026-11-25.

4. **`rpi/probe@.service` and `rpi/probe.sh` (new).** As `soft_stop@.service` and
   `soft_stop.sh`:
   - `EnvironmentFile=/etc/default/probe`, with `ROS_DOMAIN_ID`, `RIG_PI` (the RPi5 the probe
     belongs to, for the namespace) and `AIR_TEMP_C`.
   - `ExecStart` runs `ros2 run wave_probe wave_probe_node` with the namespace, the package's
     params file and `air_temp_c:=$AIR_TEMP_C` (the node sketch, change 5).
   - `Restart=on-failure`, because a USB unplug should not need a person to fix it. This
     differs from the rig units, which use `Restart=no`, but the probe moves nothing.

5. **RPi4 setup, in `rpi/README.md`.**
   - A section *The probe Pi* with these steps: Ubuntu Server 24.04 (as the RPi5), ROS 2 Jazzy,
     the workspace with `wave_probe` only (`colcon build --packages-select wave_probe`), the
     rule, the service, and wired Ethernet.
   - **Chrony on the RPi5 first.** It has `systemd-timesyncd`, which can't serve time. Install
     chrony there with `allow 192.168.1.0/24` and `local stratum 10`, then point the RPi4 at
     it. The commands are in the node sketch, *Before the node*.

## Tests and docs

- The node's tests are in [wave_probe_node_sketch.md](wave_probe_node_sketch.md), *Tests*.
- `udevadm test` on the rule with the board plugged in shows the symlink `wave_probe`.
- README: in *Lab computers*, the RPi4 and what it runs. [TODO.md](TODO.md): list this sketch
  with the others, and add it to *Order of the sketches* before gain model stage 1.

## Check on the rig

1. **On the bench.** The HC-SR04 over a still tray of water at 0.3, 0.5 and 1.0 m, measured
   with a ruler:
   - Range within 3 mm of the ruler, with the air temperature set.
   - Noise (std over 60 s) of a few mm.
   - No `+inf` on still water.
   - 20 Hz in `ros2 topic hz`.
2. **Clocks.**
   - `chronyc tracking` on the RPi4 shows below 1 ms.
   - The node's ε is steady over 30 min, and its rms is below 1 ms.
3. **Against WP1.** The HC-SR04 at WP1's distance from the wavemaker, a few cm to the side, at
   least 0.3 m above the water.
   - Record catman and the topic (`ros2 bag record`) on the same file as TODO 43's check: one
     irregular and the regular runs.
   - `transfer_function` on both, the topic's bag exported as CSV.
   - Pass:
     - the gains agree within their ±;
     - coherence at or above `COHERENCE_MIN` in the band;
     - dropouts below 10 %.
4. **Where it stops working.** Increase steepness at one period, for example 1.5 s at 1/60,
   1/40 and 1/28 (the series the gain sketch's *Later* wants). Note the steepness at which
   dropouts pass 10 %. Above that, use WP1 or the ADC.

## Open

- **The board:** an Uno R3, a Nano, or an Adalogger (a crystal clock, so decision 2's ε is
  smaller).
- **The mount:** height above the water and a rigid fixing; at 0.5 m the beam covers about
  13 cm of water.
- **The 20 Hz rate:** if false short readings (wall echoes) show at 20 Hz, use 16.7 Hz
  (`PERIOD_US` 60000).
- **The fallback:** the probe amplifier's ±10 V output through an ADC, if check 3 or 4 fails:
  - an Adalogger's own 12-bit ADC behind a resistor level shift;
  - an ADS1115;
  - on the RPi5 itself, which needs no RPi4.

## Later, not in the first pass

- **Air temperature measured, not set.** A DS18B20 on the same Arduino, sent as a fourth field.
- **Several probes.** One Arduino triggers them in turn; `frame_id` per probe. Needed for
  incident and reflected waves (gain sketch, *Later*).
