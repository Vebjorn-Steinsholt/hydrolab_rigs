# Sketch: the RPi4 node that reads the probe's Arduino and publishes Range

What to code so the RPi4 reads the HC-SR04 Arduino's serial lines and publishes them as
`sensor_msgs/Range` for the RPi5, and why. This is change 2 of
[ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md), worked out in detail. The firmware,
the udev rule and the service stay there.

**Status (2026-11-26).** Done and run on the bench: the node, `probe_serial.py`, the yaml, the
firmware and the udev rule (UNO R4 WiFi). The full text of the done parts is in
`git show 255d566:wave_probe_node_sketch.md`. Left:

- the stamp creep (*Results*, last point);
- the README, `probe@.service` and `probe.sh`;
- checks 3–6.

## Problem

The Arduino sends one line per ping, `seq,t_us,echo_us`, at 20 Hz (the firmware in the probe
sketch, change 1). Something on the RPi4 has to:

- read the lines without losing any, and see when some are lost anyway;
- turn the Arduino's `micros()` into ROS time, so the RPi5 can line each sample up with its
  setpoint;
- turn the echo time into metres;
- survive an unplugged cable or a reset board without a person;
- on request, take the still-water range as the zero, and publish the surface elevation
  against it.

**What changes for the user.**

```bash
ros2 run wave_probe wave_probe_node --ros-args -r __ns:=/wavemakers/ladertanken \
    -p port:=/dev/wave_probe -p air_temp_c:=21.5
```

```text
[wave_probe_node] /dev/wave_probe open; first line at seq 1
[wave_probe_node] clock: first fit, +430 ppm
[wave_probe_node] clock: +368 ppm, rms 0.24 ms over 30 blocks (300 s), floor 2.0 ms (set); 0 lost, 1 bad
[wave_probe_node] seq 18512 -> 18515: 2 lines lost (3 in 25 min)
[wave_probe_node] 14 % no echo in the last 60 s
[wave_probe_node] restart at seq 0 (board reset, or 3 lines that did not fit); clock fit restarted
[wave_probe_node] /dev/wave_probe: device disconnected; retrying every 2 s
```

```bash
ros2 service call /wavemakers/ladertanken/zero_measurement \
    wave_probe_interfaces/srv/ZeroMeasurement "{duration: 10.0}"
```

```text
[wave_probe_node] zero: 0.2874 m, std 0.5 mm, 200 used, 0 rejected (10 s)
[wave_probe_node] zero refused: std 6.1 mm > 2.0 mm; water not still? zero stays 0.2874 m
```

### Decisions

Decisions 1–8 are in the code. In short:

1. **A package of its own, `wave_probe`,** so the RPi4 doesn't build the rig's drivers.
2. **The serial port is read in a thread of its own,** so the arrival time is taken as soon as
   the line is complete.
3. **The stamp is when the sound reached the water:** `t_trigger + echo_us / 2`, on the RPi4's
   clock. No echo: `t_trigger` and `+inf`.
4. **The clock map is fitted to the lower edge of `d = arrival − t_send`,** with
   `t_send = t_us + echo_us`: the minimum per 10 s block, a line over the last 300 s, minus
   `latency_floor_s` (2 ms). Lines with no echo are left out of the fit.
5. **Wraps, resets, gaps and corrupt lines are told apart by `seq`:** each `t_us` step must
   match `Δseq × 50 000` within 1 ms. The first line after an open and a line cut by a read
   timeout are dropped.
6. **An unplugged port is retried every `reopen_s` (2 s),** not fatal.
7. **Range follows REP 117,** with c = 331.3 + 0.606 · `air_temp_c`. QoS reliable, depth 100.
8. **The node knows nothing about the rig.** The namespace comes from `-r __ns:=`.
9. **A zero service: the mean range over still water is the zero.**
   - **The interfaces get a package of their own, `wave_probe_interfaces`,** next to
     `wave_probe`, so the laptop and `rig_web` can call the service without building the node.
   - **The zero is a range in metres:** `zero`, the mean of the valid ranges over
     `duration`. The elevation is then `elevation = zero − range`, positive upwards. That
     is the probe sketch's `η = −(range − still-water mean)`.
   - **It is kept in metres, not as an echo time.** The distance to the water is what's fixed,
     and metres is right as long as `air_temp_c` was correct during the zero. **Set the
     temperature first, then zero.** A later change of `air_temp_c` changes the ranges but not
     `zero`, so it moves the elevation by `zero · Δc/c`: 0.5 mm per °C at 0.29 m.
   - **Only real echoes go into the mean.** Lines with `echo_us == 0`, or outside
     `min_range_m`..`max_range_m`, are left out and counted as rejected. If more than
     `dropout_warn` (10 %) of the lines are rejected, the zero fails.
   - **A noisy zero is refused.** If the std is above `zero_max_std` (default 0.002, against
     0.5 mm noise on still water), the water wasn't still. The zero fails and the old one stays.
     The name carries no unit, and the srv neither, so the service fits a probe in other units.
   - **Only the service sets the zero.** It is not a parameter and not in the yaml. Zero before
     each set of waves, as with the ordinary wave probes. A zero carried over from an earlier
     day, or from a yaml, would hide a change in water level. The node starts with no zero
     (`zero` 0.0) and publishes no elevation until the first successful call. A restart
     loses the zero. The value is in the service's response and in the log line.
   - **The elevation goes on a topic of its own, `surface_elevation`, as
     `wave_probe_interfaces/msg/SurfaceElevation`.** Not `probe_elevation`: that reads as the
     probe's position. `probe_range` stays a plain `Range`, which can't hold a negative value.
     The elevation has the same stamp as its `probe_range` sample, and is published only while
     `zero` is set. A line with no echo, or out of range, gives `nan`, so the topic keeps its
     20 Hz and the subscriber sees the dropout.
   - **The service waits out `duration` in its callback.** The reader thread keeps filling
     the samples meanwhile. The service has a reentrant callback group of its own, and `main`
     spins a `MultiThreadedExecutor`, so the status timer keeps running during a zero. A second
     request while one runs is refused, not queued. `duration` must be 1–120 s. At 20 Hz, 10 s
     is 200 samples.

## Before the node

Done on 2026-11-23: the RPi5 serves time with chrony, the RPi4 (now 192.168.1.169) follows it,
and it has ROS 2 Jazzy on domain 1. Left, with `probe@.service`:
`sudo systemctl enable chrony-wait.service` on the RPi4, and `Wants=`/`After=time-sync.target`
in the unit, so the node starts only after the clock's first step.

## Changes

Changes 1–4 are done (`package.xml` and `CMakeLists.txt`, `probe_serial.py`,
`wave_probe_node.py`, `config/wave_probe.yaml`). Why the code is as it is, beyond the decisions:
`git show 255d566:wave_probe_node_sketch.md`, *Changes*.

5. **The probe sketch's change 4 (`rpi/probe.sh`)** runs
   `ros2 run wave_probe wave_probe_node --ros-args -r __ns:=/wavemakers/$RIG_PI --params-file <share>/config/wave_probe.yaml -p air_temp_c:=$AIR_TEMP_C`.

6. **`wave_probe_interfaces/`** (decision 9). Done, and `wave_probe/package.xml` depends on
   it. The RPi4 builds both: `colcon build --packages-select wave_probe_interfaces wave_probe`,
   and the `rsync` copies the new folder too.

7. **`wave_probe/src/wave_probe_node.py`: the zero and the elevation** (decision 9). Done
   (2026-11-26), with `zero_max_std` in the yaml. Two things the tests found: `zero_max_std`
   must be declared before `add_on_set_parameters_callback`, or `on_set` refuses it and the
   node doesn't start; and the service needs a `ReentrantCallbackGroup`, or a second request
   is queued instead of refused.

## Tests

Written (2026-11-26):

- `wave_probe/test/test_probe_serial.py` (no ROS, 22 tests): `parse_line`, `range_m`,
  `Sequence` (wrap, gap, reset, bad line, corrupt reference) and `ClockFit` on simulated
  arrivals (300 ppm, 2 ms floor, exponential latency with 1 % spikes), at 1000 s and 1.8·10⁹ s.
  One change from the plan: a **5 ms** step in the arrivals returns no jump, since each refit
  moves the stamps by only about 13 % of a step (0.6 ms, under `jump_warn_s`). The test steps
  20 ms, which gives 2.7 ms.
- `wave_probe/test/test_wave_probe_node.py` (rclpy, a pty behind a symlink as the port, 21
  tests, about 40 s): the Range messages and stamps, the dropped first and cut lines, echo-0
  lines out of the fit, gap and reset, reopening, `air_temp_c`, SIGINT, and the zero (all the
  cases in decision 9). Each test runs on a domain of its own (100–199), so it can't reach a
  rig. It needs `source ~/ros_ws/install/setup.bash` for `wave_probe_interfaces`. All 43
  pass, also on one loaded core, and `ament_flake8` and `ament_pep257` are clean. The tests
  wait on events, not fixed times (a zero is fed until it answers), and read the node's log
  in-process: on the RPi4 the fixed waits failed, and `/rosout` and `capfd` both lost lines.
  The node and the test's helper spin in executors of their own: sharing one
  `MultiThreadedExecutor`, the RPi4's newer rclpy busy-loops while a zero sleeps, and the
  reader gets 5 lines/s. The node alone, as in `main`, uses 6 % of a core on the RPi4 and gets
  200 of 200 samples in a 10 s zero.
  SIGINT is allowed 2 s, not 1: it took 1.3 s on the RPi4, since the reader join alone can
  take 1 s.

- Docs: `wave_probe/README.md`, a short one: what the node does, its parameters and the line
  format. Link it from the probe sketch, and from `rpi/README.md`'s *The probe Pi*.

## Check on the bench and the rig

1. **The Arduino alone:** passed 2026-11-23.
2. **The node:**
   - `ros2 topic hz /wavemakers/ladertanken/probe_range` shows 20 Hz, on the RPi5.
   - `ros2 topic echo … --once` shows a range within 3 mm of a ruler over still water, with the
     air temperature set.
3. **The latency floor.** Wire the Arduino's trigger pin also to a GPIO on the RPi4, and stamp
   its rising edge with `gpiomon` (libgpiod). The median of (edge time − mapped `t_trigger`)
   over 10 min is the error left in `latency_floor_s`. Correct the default by that amount, and
   note it in COMMISSIONING.md.
4. **The clocks.**
   - Over 1 h, ε is steady within 20 ppm, and the fit's rms stays below 1 ms.
   - `chronyc tracking` on the RPi4 shows below 1 ms against the RPi5.
5. **Unplug and reset:**
   - Pull the USB cable for 10 s, then plug it in again: one warning, then data again within
     5 s.
   - Press the Arduino's reset button: one `restart at seq 0`, no stamp jump over 5 ms.
6. **An hour's soak:** no lost lines, and the memory use of the node (`ps -o rss`) is steady.
   This can run alongside the probe sketch's check 3 against WP1.
7. **The zero, over still water:**
   - a 10 s zero succeeds with a std below 1 mm, and `surface_elevation` then reads 0 ± 1 mm;
   - float an object with a flat top, 10 cm across or more, under the probe (the mount is
     fixed, so the target moves, not the probe): the elevation is positive and equals the
     freeboard;
   - block the beam so there is no echo: the elevation is `nan`, still at 20 Hz;
   - zero again during a small regular wave (H 0.02 m, T 1.0 s, std H/2√2 = 7 mm): refused as
     noisy, and the old zero stays.

**Results** (checks 2, 4 and 6 on 2026-11-23 on an Uno R3, before the swap to the R4; full text
in git. Check 2's ruler and check 7 on 2026-11-26, UNO R4 WiFi, Ladertanken, air 19.5 °C):

- **Check 2: passed.** 19.99 Hz on both Pis (19.98 Hz, std 0.7 ms on 2026-11-26); range noise
  0.5 mm std. Over water, 0.3716 m agrees with a ruler.
- **Check 4: partly.** ε +365…+370 ppm; stamps scatter 77 µs on the RPi4 alone;
  arrival − stamp at least 2.2 ms, so `latency_floor_s` 2 ms is plausible until check 3.
- **Check 6: partly.** 30 min, 0 lines lost, 1 bad (the line cut by the open's reset).
- **Stamps creep: open.** The floor of `receive − stamp` per 30 s, which should stay at
  2.2–2.8 ms, rises about 0.1 ms/min: on the RPi5 (2.2 → 3.5 ms over 10 min on 2026-11-24, with
  chrony settled: offset 14 µs, skew 0.036 ppm), and in the RPi4-only bag too, on one clock.
  So the cause is in the node's stamping or `ClockFit`, not chrony, and fitting against
  `CLOCK_MONOTONIC_RAW` is not the fix. Next: log the raw (`t_send`, arrival) pairs on the RPi4
  and replay `ClockFit` offline.
- **Check 7: passed,** with two changes. The mount is fixed, so a floating object replaced the
  lifted probe. Step 6 (the beam blocked) was skipped; the `nan` is covered by
  `test_no_echo_after_zero_is_nan`.
  - **The zero:** 0.3713 m, std 0.3 mm, 200 used, 0 rejected. No elevation before it.
  - **Still water after it:** −0.17 mm, std 0.31 mm.
  - **A floating object:** +141.4 mm, std 0.39 mm. The sign is right. Its freeboard was read
    by eye as about 150 mm, too rough to check the scale; the scale is the ruler's in check 2.
  - **A regular wave, H 0.02 m, T 1.0 s:** refused, std 6.9 mm (7.07 expected), 200 used,
    0 rejected; the zero stayed. The mean over the wave was 0.3 mm off the zero.
- **The tests on the RPi4:** 43 pass. The node alone, at 20 Hz on a pty, uses 6 % of a core,
  and a 10 s zero gets 200 of 200 samples.
- Checks 3 and 5 not done.

## Open

- **`latency_floor_s`:** 2 ms is an estimate until Check 3.
- **Where the air temperature comes from:** a parameter in `/etc/default/probe` for now (the
  probe sketch's *Later* has a DS18B20).
- **Mean or median for the zero:** the mean, as decided. With the std check, a few spikes can't
  move it far, but a median would ignore them altogether.

## Later, not in the first pass

- **More probes on one Arduino:** a fourth field, `probe`, and one topic per probe.
- **`diagnostic_msgs`:** publish the clock fit, the gaps and the dropout share on `/diagnostics`
  instead of only logging them, so the RPi5 or the web page can show the probe's state.
