# Sketch: the RPi4 node that reads the probe's Arduino and publishes Range

What to code so the RPi4 reads the HC-SR04 Arduino's serial lines and publishes them as
`sensor_msgs/Range` for the RPi5, and why. This is change 2 of
[ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md), worked out in detail. The firmware,
the udev rule and the service stay there.

**Status (2026-11-23): written and run on the bench.** `wave_probe` (the node and
`probe_serial.py`) and `rpi/99-wave-probe.rules` exist; the tests, the README, `probe@.service`
and `probe.sh` do not, so it builds only with `-DBUILD_TESTING=OFF`. The RPi4 (Ubuntu Server
24.04, 192.168.1.170) has ROS 2 Jazzy and runs the node. Both Pis are on ROS domain 1, and the RPi4 follows the RPi5 with chrony
(2026-11-23). Results so far are under *Check on the bench and the rig*.

**2026-11-25: a zero service is planned, not written** (decision 9, changes 6 and 7, check 7).

Line numbers are as of `master` on 2026-11-23 (`0b8919e`).

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
    wave_probe_interfaces/srv/ZeroMeasurement "{duration_s: 10.0}"
```

```text
[wave_probe_node] zero: 0.2874 m, std 0.5 mm, 200 used, 0 rejected (10 s)
[wave_probe_node] zero refused: std 6.1 mm > 2.0 mm; water not still? zero stays 0.2874 m
```

### Decisions

1. **A package of its own, `wave_probe`, not a node in `rig_controller`.** `rig_controller`
   depends on `mgate5101_driver` and `linmot_serial_driver` (`rig_controller/package.xml`, lines
   18-19). The RPi4 would have to build both drivers for a node that needs only `rclpy` and
   `sensor_msgs`. `wave_probe` builds in seconds, and the RPi4 checks out the repository but
   builds only it: `colcon build --packages-select wave_probe`.
   - `ament_cmake`, with the programs installed by `install(PROGRAMS …)` without `.py`, as
     `rig_controller` does (`rig_controller/CMakeLists.txt`, lines 58-89).
   - The tests registered with `ament_add_pytest_test`, so `colcon test` runs them (TODO 58).
2. **The serial port is read in a thread of its own.** A blocking `readline()` with a 0.5 s
   timeout in a plain thread, not in a ROS timer. The arrival time is then taken as soon as the
   line is complete, and not when the executor next gets to it. The thread publishes directly
   (`publish` is thread-safe in rclpy), and the executor serves only parameters and logging.
3. **Each sample's stamp is the moment the sound reached the water, on the RPi4's clock.**
   - The Arduino's clock is mapped onto ROS time (decision 4).
   - The pulse reaches the surface half an echo after the trigger, so the stamp is
     `t_trigger + echo_us / 2`. At 1 m that is 2.9 ms. It is small, but it would otherwise show
     up as a phase error that depends on the mount height.
   - No echo: the stamp is `t_trigger` and the range `+inf`.
4. **The clock map is fitted to the lower edge of the arrivals.**
   - Each line's arrival is the Arduino time plus a transfer delay that is never negative: the
     line on the wire (about 1.7 ms for 20 bytes at 115200), USB polling (up to 1 ms), and the
     RPi4's scheduling (mostly under 1 ms, sometimes tens).
   - **The line leaves after the echo, not at `t_us`.** The firmware prints after `pulseIn`
     returns, at `t_us + echo_us` plus a fixed start delay (the HC-SR04 raises its echo pin after
     its 40 kHz burst, a few hundred µs). The fit is fed `t_send = t_us + echo_us`, not `t_us`.
     Fed `t_us`, the lower edge would follow the shortest echo in each block, and the stamps
     would move with the water level.
   - **Lines with no echo are left out of the fit.** `pulseIn` gave up after 30 ms at some
     point we don't know exactly. They are still published, as `+inf`.
   - Model: `arrival = a + (1 + ε) · t_send + delay`, delay ≥ d₀.
   - In blocks of 10 s, keep the line with the smallest `d = arrival − t_send`. Fit a line to
     those minima over the last 300 s (30 blocks). Its slope is ε. Its intercept is `a + d₀`.
   - Subtract `latency_floor_s`, a parameter (default 0.002 s), for d₀. That is the part no fit
     can see, so it stays a parameter, measured once on the bench (Check 3). The echo pin's
     start delay is in it too, since Check 3 compares against the trigger edge.
   - Until two blocks exist, use the smallest `d` so far, with ε = 0. The error then grows as
     ε · t, 6 ms after 20 s at 300 ppm, plus a few ms in the first seconds while the minimum
     is over few lines. The node logs when the first fit is in.
   - The fit is redone at each new block. A jump of more than 1 ms in the mapped time between
     two refits is logged. The jump from the warm-up guess to the first fit is not.
   - Buffered lines read late after an open only have larger `d`, so they can't pull the lower
     edge down. Nothing can arrive before it was sent, except after a step of the RPi4's
     clock. That's why the unit waits for chrony (*Before the node*, step 2).
5. **`micros()` wraps and board resets are told apart by `seq`.**
   - **Each step's `t_us` must match its `seq` step:** `(t_us − t_us_prev) mod 2³²` equal to
     `Δseq × 50 000` within 1 ms. Both are counted in the Arduino's own `micros()`, so the
     resonator's error cancels. `t_us` is taken on the `next_us` schedule, a few µs late at
     most, so 1 ms is generous.
   - A step that matches and has `t_us` below the previous is a wrap (every 71.6 min): add 2³².
   - A step that doesn't match is a corrupt line: it is not published, and the previous line
     stays the reference. If 3 lines in a row don't match, the reference was the bad one, or
     the firmware restarted its schedule after a stall. The next line then starts again as after
     a reset.
   - `seq` going backwards, or standing still, is a board reset: start the unwrap and the clock
     fit again, and log it.
   - A forward `seq` gap that matches is lost lines: count it and log it. The fit is
     unaffected.
   - `seq` wraps too, after 6.8 years at 20 Hz. Not handled.
   - **The first line after an open, and the line after a read that timed out half way, are
     dropped unparsed.** Either can start mid-line, and a tail like `56,3456,5824` parses as a
     good line with the wrong `seq`. An Uno resets on open and starts clean, but a board with
     native USB (the Adalogger) doesn't.
6. **An unplugged port is retried, not fatal.** On `SerialException`, close the port, log it
   once, and try to open it again every `reopen_s` (2 s). When it is back, it starts as from
   cold (decision 5). The service's `Restart=on-failure` is only for crashes of the node itself.
7. **Range follows REP 117** (as the probe sketch, decision 3):
   - `range = echo_us · 1e-6 · c / 2`, c = 331.3 + 0.606 · `air_temp_c` m/s;
   - 0 → `+inf`;
   - outside `min_range_m`..`max_range_m`: published as it is; the subscriber treats it as a
     dropout.

   The QoS is reliable with depth 100 (`KEEP_LAST`, `VOLATILE`).
8. **The node knows nothing about the rig.** No setpoints. The namespace puts it under
   `/wavemakers/<pi>` through `-r __ns:=`, as `soft_stop.sh` does. Until 2026-11-25 this also
   said "no η, no zero", with η formed on the RPi5. Decision 9 replaces that part, and
   `probe_gain` calls the zero service before each run (the probe sketch's decision 7, changed
   to match on 2026-11-25).
9. **A zero service: the mean range over still water is the zero.**
   - **The interfaces get a package of their own, `wave_probe_interfaces`.** ROS package names
     can't contain spaces, so the folder `wave_probe/wave_probe interfaces/` has to be renamed
     before colcon will build it. As its own package, the laptop and `rig_web` can call the
     service without building the node.
   - **The zero is a range in metres:** `zero_m`, the mean of the valid ranges over
     `duration_s`. The elevation is then `elevation = zero_m − range`, positive upwards. That
     is the probe sketch's `η = −(range − still-water mean)`.
   - **It is kept in metres, not as an echo time.** The distance to the water is what's fixed,
     and metres is right as long as `air_temp_c` was correct during the zero. **Set the
     temperature first, then zero.** A later change of `air_temp_c` changes the ranges but not
     `zero_m`, so it moves the elevation by `zero_m · Δc/c`: 0.5 mm per °C at 0.29 m.
   - **Only real echoes go into the mean.** Lines with `echo_us == 0`, or outside
     `min_range_m`..`max_range_m`, are left out and counted as rejected. If more than
     `dropout_warn` (10 %) of the lines are rejected, the zero fails.
   - **A noisy zero is refused.** If the std is above `zero_max_std_m` (default 2 mm, against
     0.5 mm noise on still water), the water wasn't still. The zero fails and the old one stays.
   - **Only the service sets the zero.** It is not a parameter and not in the yaml. Zero before
     each set of waves, as with the ordinary wave probes. A zero carried over from an earlier
     day, or from a yaml, would hide a change in water level. The node starts with no zero
     (`zero_m` 0.0) and publishes no elevation until the first successful call. A restart
     loses the zero. The value is in the service's response and in the log line.
   - **The elevation goes on a topic of its own, `probe_elevation`, as
     `wave_probe_interfaces/msg/Elevation`.** `probe_range` stays a plain `Range`, which can't
     hold a negative value. The elevation has the same stamp as its `probe_range` sample, and is
     published only while `zero_m` is set. A line with no echo, or out of range, gives `nan`,
     so the topic keeps its 20 Hz and the subscriber sees the dropout.
   - **The service waits out `duration_s` in its callback.** The reader thread keeps filling
     the samples meanwhile. The service has a callback group of its own, and `main` spins a
     `MultiThreadedExecutor`, so the status timer keeps running during a zero. A second request
     while one runs is refused, not queued. `duration_s` must be 1–120 s. At 20 Hz, 10 s is
     200 samples.

## Before the node

These are setup steps, not code, but the node's stamps need them:

1. **The RPi5 serves time.** It already had chrony (2026-11-23), as a client only. On the RPi5,
   with no rig running:

   ```bash
   sudo apt install -y chrony                      # already there on ladertanken-rpi
   printf 'allow 192.168.1.0/24\nlocal stratum 10\n' | sudo tee /etc/chrony/conf.d/lab-server.conf
   sudo systemctl restart chrony
   ```

   `local stratum 10` keeps it serving when the internet is down, so the two Pis still agree
   with each other. `play_dat` and the controller use the monotonic clock, so a clock step does
   not move a running file.
2. **The RPi4 follows it:** `server 192.168.1.98 iburst prefer` (the IP: the RPi4 has no avahi) in
   `/etc/chrony/conf.d/rpi5.conf`. `chronyc tracking` should show an offset below 1 ms.
   - `sudo systemctl enable chrony-wait.service`, and `probe@.service` gets
     `Wants=time-sync.target` and `After=time-sync.target`. The node then starts after the
     clock's first step, and chrony only slews it from then on (its default `makestep 1 3`
     steps only in the first three updates). A step while the node runs would shift every
     `d` and drag the fit for up to 300 s.
3. **The RPi4 has:**
   - ROS 2 Jazzy `ros-base`, `ROS_DOMAIN_ID=1`;
   - `avahi-daemon`, `python3-serial`;
   - its user in `dialout`;
   - `brltty` removed if installed (it takes CH340 ports);
   - wired Ethernet;
   - the repository, with `wave_probe` built.

   Check that `ros2 topic list` shows the RPi5's topics.

## Changes

**Changes 1–4 are written** (`wave_probe/`, branch `wave-probe`, 5bdbe2b). Their code is in the
files, not here. What's below is only what the code doesn't say: why it is that way.

1. **`wave_probe/package.xml` and `wave_probe/CMakeLists.txt`: done.**

2. **`wave_probe/src/probe_serial.py`: done.** The pure part: no ROS, so the tests need nothing
   else. It has the line format, the range, the sequence check (decision 5) and the clock fit
   (decision 4).
   - **`parse_line`** matches on bytes, so line noise that isn't UTF-8 gives `None`, not an
     exception. `fullmatch` rejects a minus sign, a fourth field, leading text and an empty
     line. The digit limits and the `WRAP` check reject numbers an `unsigned long` can't hold.
   - **`Sequence`:** only lines that fit become the reference, so one corrupt line costs one
     line, not two. `t_s` restarts near 0 at each `'first'` and `'reset'`, and the clock fit
     restarts with it, so the jump doesn't matter.
   - **`ClockFit`** works on `d = arrival − t_send`, minus the first `d` seen (`d_ref`).
     `arrival` is ROS time, about 1.8·10⁹ s. Float64 resolves that to 0.24 µs, which is enough,
     but subtracting `d_ref` keeps the numbers in `polyfit` in ms.
     - `to_ros` takes any Arduino time, not only a send time. The node passes the time the
       sound reached the water (decision 3).
     - No outlier rejection on the minima: a block holds 200 lines, and its lowest delay is
       close to d₀ unless all 200 were delayed. With 3 ms mean exponential latency the expected
       minimum is 15 µs above d₀. The `rms` in the status shows it if that ever fails.
     - A gap in the data (a block with no lines) only skips a block index. The fit is over the
       blocks there are.
     - `t_first` sets where blocks start, so the blocks are aligned the same way for a whole
       run.

3. **`wave_probe/src/wave_probe_node.py`: done.**
   - **No locks on the counters.** They are written only by the reader thread and read by the
     status timer, and they are plain ints. The no-echo share keeps a running count next to
     its deque, because summing a deque while the other thread appends to it can raise
     `RuntimeError: deque mutated during iteration`.
   - **A line without `\n` is a read timeout.** pyserial's `readline` returns what it has when
     the 0.5 s timeout runs out. If it had bytes, the rest of that line comes in the next read,
     and that one is dropped too.
   - **Ctrl-C:** rclpy's SIGINT handler shuts the context down before `main` gets to `stop()`,
     and the thread can publish once more in between (`RCLError: publisher's context is
     invalid`, seen on the RPi4 on 2026-11-23). `publish` stops the thread quietly then, and
     still raises while the context is fine. `RCLError` is in a private module, hence
     `except Exception`.
   - **A restart right after opening is logged as info.** The Uno's USB chip still holds lines
     from before the open's reset, so the first lines can be old ones (seq 2854 on the RPi4),
     followed by seq 0 after the bootloader.
   - **No stamps before the first echo.** Until the first line with an echo after an open or a
     reset, there is no clock to stamp with (`d_ref` is `None`), so those lines are dropped. On
     still water that is one line.
   - **The status timer** (1 s) logs the first fit once, the clock every 5 min, and the no-echo
     share above `dropout_warn` at most once a minute. Gaps are logged as they come, by the
     thread.
   - **`air_temp_c` while running:** the callback checks all parameters first and changes
     nothing unless all pass. The others are read at start only, so setting them is refused,
     not silently ignored. A float assignment is atomic, so the next line uses the new value.

4. **`wave_probe/config/wave_probe.yaml`: done.**

5. **The probe sketch's change 4 (`rpi/probe.sh`)** runs
   `ros2 run wave_probe wave_probe_node --ros-args -r __ns:=/wavemakers/$RIG_PI --params-file <share>/config/wave_probe.yaml -p air_temp_c:=$AIR_TEMP_C`.

6. **`wave_probe_interfaces/` (new, renamed from `wave_probe/wave_probe interfaces/`)**
   (decision 9).

   `srv/ZeroMeasurement.srv`:

   ```text
   # Average the range over duration_s; the mean becomes the zero (still-water range).
   float64 duration_s        # 1-120 s
   ---
   bool success
   string message            # why it failed, or a one-line summary
   float64 zero_m            # mean range of the valid echoes
   float64 std_m
   uint32 n_used
   uint32 n_rejected         # no echo, or out of range
   ```

   `msg/Elevation.msg`:

   ```text
   std_msgs/Header header    # the stamp of the probe_range sample it comes from
   float64 elevation_m       # zero_m - range, positive up; nan for no echo or out of range
   ```

   - `CMakeLists.txt`: `find_package(rosidl_default_generators REQUIRED)`,
     `find_package(std_msgs REQUIRED)`, and
     `rosidl_generate_interfaces(${PROJECT_NAME} "srv/ZeroMeasurement.srv" "msg/Elevation.msg" DEPENDENCIES std_msgs)`.
   - `package.xml`: `<buildtool_depend>rosidl_default_generators</buildtool_depend>`,
     `<depend>std_msgs</depend>`, `<exec_depend>rosidl_default_runtime</exec_depend>`, and
     `<member_of_group>rosidl_interface_packages</member_of_group>`.
   - `wave_probe/package.xml` gets `<exec_depend>wave_probe_interfaces</exec_depend>`. The RPi4
     builds both: `colcon build --packages-select wave_probe_interfaces wave_probe`, and the
     `rsync` copies the new folder too.

7. **`wave_probe/src/wave_probe_node.py`: the zero and the elevation** (decision 9).

   **Setup,** in `__init__`:

   ```python
   from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
   from rclpy.executors import MultiThreadedExecutor
   from wave_probe_interfaces.msg import Elevation
   from wave_probe_interfaces.srv import ZeroMeasurement

   ZERO_DURATION_S = (1.0, 120.0)

           self.zero_m = 0.0          # not set; only on_zero sets it, never a parameter
           self.zero_max_std = p('zero_max_std_m', 0.002)
           self.zero_lock = threading.Lock()
           self.collect = None        # list of ranges (None = rejected) while a zero runs
           self.elev_pub = self.create_publisher(Elevation, 'probe_elevation', qos)
           self.create_service(ZeroMeasurement, 'zero_measurement', self.on_zero,
                               callback_group=MutuallyExclusiveCallbackGroup())
   ```

   **Each line,** at the end of `publish`, with `r = msg.range`:

   ```python
           valid = line.echo_us > 0 and self.min_range <= r <= self.max_range
           with self.zero_lock:
               if self.collect is not None:
                   self.collect.append(r if valid else None)
           if self.zero_m > 0.0:
               elev = Elevation(header=msg.header)
               elev.elevation_m = self.zero_m - r if valid else math.nan
               self.elev_pub.publish(elev)
   ```

   **The service.** It fills `res` and returns it on every path.

   ```python
       def on_zero(self, req, res):
           if not ZERO_DURATION_S[0] <= req.duration_s <= ZERO_DURATION_S[1]:
               return self.zero_failed(res, f'duration_s must be {ZERO_DURATION_S} s')
           with self.zero_lock:
               if self.collect is not None:
                   return self.zero_failed(res, 'a zero is already running')
               self.collect = []
           time.sleep(req.duration_s)                    # the reader thread fills collect
           with self.zero_lock:
               samples, self.collect = self.collect, None
           good = [s for s in samples if s is not None]
           res.n_used, res.n_rejected = len(good), len(samples) - len(good)
           if not samples:
               return self.zero_failed(res, 'no samples: is the port open?')
           if res.n_rejected / len(samples) > self.dropout_warn:
               return self.zero_failed(res, f'{res.n_rejected} of {len(samples)} rejected')
           res.zero_m, res.std_m = float(np.mean(good)), float(np.std(good))
           if res.std_m > self.zero_max_std:
               return self.zero_failed(res, f'std {res.std_m * 1e3:.1f} mm > '
                                            f'{self.zero_max_std * 1e3:.1f} mm; water not still?')
           self.zero_m = res.zero_m        # a float assignment: the reader thread sees it whole
           res.success = True
           res.message = (f'zero: {res.zero_m:.4f} m, std {res.std_m * 1e3:.1f} mm, '
                          f'{res.n_used} used, {res.n_rejected} rejected ({req.duration_s:g} s)')
           self.get_logger().info(res.message)
           return res

       def zero_failed(self, res, why):
           res.success = False
           res.message = f'zero refused: {why}; zero stays {self.zero_m:.4f} m'
           self.get_logger().warning(res.message)
           return res
   ```

   - **`on_set` is unchanged.** `zero_max_std_m` is start-only like the others.
   - **`main`** spins a `MultiThreadedExecutor` instead of `rclpy.spin(node)`, so the status
     timer runs while `on_zero` sleeps:

     ```python
         executor = MultiThreadedExecutor()
         executor.add_node(node)
         executor.spin()
     ```

   - **`config/wave_probe.yaml`** gets `zero_max_std_m: 0.002`, and no zero.

## Tests

- `wave_probe/test/test_probe_serial.py` (no ROS):
  - **`parse_line`:**
    - `b'12,3456,5824\r\n'` gives the three numbers;
    - `b'3456,5824\r\n'` (a partial first line), `b'Distance: 12\r\n'`, `b''`, `b'1,-2,3\r\n'`
      and `b'1,2,3,4\r\n'` give `None`.
  - **`range_m`:**
    - 5824 µs at 20 °C gives 1.000 m ± 0.1 mm;
    - 0 gives `+inf`;
    - the same echo at 30 °C is 1.8 % longer.
  - **`Sequence`:**
    - `t_us` across 2³² stays monotonic, and `wraps` counts 1;
    - a `seq` jump of 3 with `t_us` 150 ms on is a gap of 2;
    - `seq` back to 0 is a `reset`;
    - one line with `t_us` 2 ms off is `bad`, and the next good line is a `gap` of 1 against
      the line before it (the bad line isn't published, so it counts as lost);
    - a backwards `t_us` with a `seq` step that doesn't match is `bad`, not a wrap;
    - a reference that was itself corrupt: the 3rd line after it is a `reset`, and the 4th is
      `ok`.
  - **`ClockFit`,** with `t_send` every 50 ms plus an echo of 1–6 ms, and arrivals =
    true × (1 + 300 ppm) + 2 ms + exponential latency (mean 3 ms, 1 % spikes of 30 ms):
    - after 300 s, `to_ros` is within 0.5 ms of the truth, and ε within 20 ppm;
    - before 20 s, within 300 ppm × t + 5 ms (no slope yet, and few lines in the first minimum);
    - a refit in a steady run returns no jump; the arrivals stepped by 5 ms return one;
    - with `d_ref` near 1.8·10⁹ s (ROS time today), the same bounds hold;
    - after `reset`, it starts from cold.
- `wave_probe/test/test_wave_probe_node.py` (rclpy, a pseudo-terminal from `os.openpty()` as
  the port; `exclusive=True` works on a pty):
  - Lines written to the pty come out as Range messages, one each, with the stamps from the fit
    and `+inf` for echo 0.
  - The first line after the open is dropped. A line written in two halves with 0.6 s between
    (past the read timeout) is dropped, and its tail too.
  - Echo-0 lines are published but don't move the fit: 50 % dropouts give the same stamps.
  - A gap and a reset are logged, and the clock restarts at the reset.
  - Closing the pty is logged once. A new pty at the same path (a symlink) is picked up within
    `reopen_s`.
  - `ros2 param set air_temp_c 30` changes the next ranges by 1.8 %; `air_temp_c 80` and
    `port` are refused. `-p air_temp_c:=21` (an int) starts.
  - SIGINT stops the node within 1 s.
  - **The zero** (decision 9, change 7):
    - a constant range: `zero_m` equals it, `std_m` ≈ 0, and the elevation is 0 afterwards;
    - before any zero, nothing is published on `probe_elevation`;
    - 20 % `echo_us == 0`: fails, and `zero_m` keeps its old value;
    - ranges with a 6 mm std: fails as noisy, and `zero_m` keeps its old value;
    - a second call while one runs: refused;
    - `duration_s` 0 or 500: refused;
    - no lines arriving (pty closed): fails with "no samples";
    - a line with no echo after a zero gives `nan` on `probe_elevation`;
    - `zero_m` is not a parameter: `ros2 param set zero_m 0.3` is refused;
    - the status timer keeps firing during a 3 s zero.
- Docs: `wave_probe/README.md`, a short one: what the node does, its parameters and the line
  format. Link it from the probe sketch, and from `rpi/README.md`'s *The probe Pi*.

## Check on the bench and the rig

1. **The Arduino alone, on the RPi4:** `python3 -m serial.tools.miniterm /dev/wave_probe 115200`
   shows the lines, and `seq` counts up at 20 Hz.
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
   - a 10 s zero succeeds with a std below 1 mm, and `probe_elevation` then reads 0 ± 1 mm;
   - lift the probe 10 mm on its mount: the elevation reads −10 mm ± 1 mm;
   - stir the water and zero again: refused as noisy, and the old zero stays.

**Results on 2026-11-23,** on the bench: an Uno R3 (serial `55730303137351D031E2`) on the RPi4,
the HC-SR04 over a still object at about 23 cm, 20 °C set. The bag analysis is from a 10 min
`ros2 bag record` (11 940 messages).

- **Check 1: passed.** `seq` counts without gaps; `t_us` steps 50 000 ± 4 µs (the Uno's
  `micros()` resolution), well inside `Sequence`'s 1 ms. At 28 cm by ruler, `echo_us` is
  1663–1673 µs: 28.7 cm, +7 mm (a fixed offset, which drops out of η) and ±0.5 mm. The
  1667/1673 pattern repeats every ~6 lines: the timer0 interrupt (every 1.024 ms) landing in
  `pulseIn`'s counting loop.
- **Check 2: partly.** 19.99 Hz in `ros2 topic hz`, on the RPi4 and on the RPi5 (domain 1,
  default Fast DDS on both). On the RPi5 the intervals scatter 2.3 ms std (37–66 ms), against
  1.2 ms on the RPi4: Ethernet and DDS. Range noise 0.5 mm std, per-minute means within 0.05 mm
  over 10 min. Not yet over water against a ruler.
- **Check 4: partly.** ε +365…+370 ppm over 30 min (the 5-min log lines), so the resonator runs
  slow. The first fit, from 2 blocks, was 60 ppm off; the stamps settle within 90 s. Block rms
  0.21–0.38 ms (USB batching in the Uno's 16U2). The stamps scatter 77 µs about a straight line,
  the arrivals 771 µs. Arrival − stamp is at least 2.2 ms, as half the echo plus the line on the
  wire predicts, so `latency_floor_s` 2 ms is plausible until check 3. No chrony yet.
- **Check 6: partly.** 30 min, 0 lines lost, 1 bad (the line cut by the open's reset).
- **Stamps against the RPi5's clock: open.** A 10 min bag on the RPi5, recorded right after the
  RPi4 had switched chrony to the RPi5 (skew 11 ppm, RMS offset 0.42 ms). The lowest
  `receive − stamp` per 30 s, which can't be below about 2.2 ms (half the echo plus the line on
  the wire), was 2.1–2.8 ms for 4 min, then rose to 4.6–5.2 ms. The stamps' rate swung
  ±10–16 ppm from minute to minute, and they scattered 488 µs about a line, against 77 µs on the
  RPi4 alone. Most likely chrony was still slewing the RPi4's clock, and the 300 s fit lags a
  change in its rate. To do on 2026-11-24: check `chronyc tracking` on the RPi4 (skew below
  1 ppm, RMS offset below 0.1 ms), then record 10 min on the RPi5 again; the floor should stay
  at 2.2–2.8 ms. If it doesn't, or to be independent of chrony anyway: fit against
  `CLOCK_MONOTONIC_RAW` and add the current `time.time_ns() − CLOCK_MONOTONIC_RAW` only at
  stamping (decision 4 would change; `ClockFit` would not).
- Checks 3 and 5 not done.

## Open

- **The board:** an Uno R3, a Nano clone (CH340) or an Adalogger. This changes the udev rule
  and whether `brltty` matters, not the node.
- **`latency_floor_s`:** 2 ms is an estimate until Check 3.
- **Where the air temperature comes from:** a parameter in `/etc/default/probe` for now (the
  probe sketch's *Later* has a DS18B20).
- **Mean or median for the zero:** the mean, as decided. With the std check, a few spikes can't
  move it far, but a median would ignore them altogether.

## Later, not in the first pass

- **More probes on one Arduino:** a fourth field, `probe`, and one topic per probe.
- **`diagnostic_msgs`:** publish the clock fit, the gaps and the dropout share on `/diagnostics`
  instead of only logging them, so the RPi5 or the web page can show the probe's state.
