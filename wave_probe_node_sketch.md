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

Line numbers are as of `master` on 2026-11-23 (`0b8919e`).

## Problem

The Arduino sends one line per ping, `seq,t_us,echo_us`, at 20 Hz (the firmware in the probe
sketch, change 1). Something on the RPi4 has to:

- read the lines without losing any, and see when some are lost anyway;
- turn the Arduino's `micros()` into ROS time, so the RPi5 can line each sample up with its
  setpoint;
- turn the echo time into metres;
- survive an unplugged cable or a reset board without a person.

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
8. **The node knows nothing about the rig.** No setpoints, no η, no zero. The namespace puts it
   under `/wavemakers/<pi>` through `-r __ns:=`, as `soft_stop.sh` does. η is formed on the RPi5
   (the probe sketch, decision 7).

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

1. **`wave_probe/package.xml` and `wave_probe/CMakeLists.txt` (new).**
   - Depends on `rclpy`, `sensor_msgs`, `python3-serial`, `python3-numpy`.
   - Installs `src/wave_probe_node.py` as `wave_probe_node`, and `src/probe_serial.py` next to
     it (imported).
   - Installs `config/wave_probe.yaml` to `share/wave_probe/config`.

2. **`wave_probe/src/probe_serial.py` (new): the pure part.** No ROS, so the tests need nothing
   else. Four pieces: the line format, the range, the sequence check, and the clock fit.

   **The line and the range.** The regex works on bytes, so line noise that isn't UTF-8 gives
   `None` and not an exception. `fullmatch` rejects a minus sign, a fourth field, leading text
   and an empty line. The digit limits and the `WRAP` check reject numbers an `unsigned long`
   can't hold.

   ```python
   import math
   import re
   from collections import deque
   from dataclasses import dataclass

   import numpy as np

   WRAP = 2**32              # micros() and seq are unsigned long on the Arduino
   PERIOD_US = 50_000        # the firmware's PERIOD_US (20 Hz)
   _LINE = re.compile(rb'(\d{1,10}),(\d{1,10}),(\d{1,6})')


   @dataclass(frozen=True)
   class Line:
       seq: int
       t_us: int             # micros() just before the trigger
       echo_us: int          # echo pin high time; 0 = no echo within 30 ms


   def parse_line(raw: bytes) -> Line | None:
       m = _LINE.fullmatch(raw.rstrip(b'\r\n'))
       if m is None:
           return None
       seq, t_us, echo_us = (int(g) for g in m.groups())
       if seq >= WRAP or t_us >= WRAP:
           return None
       return Line(seq, t_us, echo_us)


   def speed_of_sound(air_temp_c: float) -> float:
       return 331.3 + 0.606 * air_temp_c          # m/s, dry air


   def range_m(echo_us: int, air_temp_c: float) -> float:
       if echo_us == 0:
           return math.inf                        # REP 117: no return
       return echo_us * 1e-6 * speed_of_sound(air_temp_c) / 2
   ```

   **The sequence check** (decision 5). `push` gives a `Step` per line, and the node acts on
   its `kind`. `t_s` is the unwrapped `t_us` in seconds, the Arduino's time of the trigger.

   ```python
   @dataclass(frozen=True)
   class Step:
       kind: str             # 'first' | 'ok' | 'gap' | 'reset' | 'bad'
       t_s: float            # unwrapped t_us in s; nan for 'bad'
       lost: int = 0         # lines lost just before this one ('gap' only)


   class Sequence:
       def __init__(self, period_us=PERIOD_US, tol_us=1_000, max_bad=3):
           self.period_us, self.tol_us, self.max_bad = period_us, tol_us, max_bad
           self.reset()

       def reset(self):
           self.prev: Line | None = None    # the last line that fitted: the reference
           self.wraps = 0                   # times 2**32 has been added to t_us
           self.bad_run = 0                 # lines in a row that did not fit the reference

       def push(self, line: Line) -> Step:
           if self.prev is None:
               return self._start(line, 'first')
           dseq = line.seq - self.prev.seq
           if dseq <= 0:
               return self._start(line, 'reset')           # the board restarted from seq 0
           dt_us = (line.t_us - self.prev.t_us) % WRAP    # the forward step, across a wrap too
           if abs(dt_us - dseq * self.period_us) > self.tol_us:
               self.bad_run += 1
               if self.bad_run >= self.max_bad:            # the reference was the bad one,
                   return self._start(line, 'reset')       # or the schedule restarted
               return Step('bad', math.nan)
           if line.t_us < self.prev.t_us:
               self.wraps += 1
           self.prev, self.bad_run = line, 0
           t_s = (self.wraps * WRAP + line.t_us) * 1e-6
           return Step('gap', t_s, dseq - 1) if dseq > 1 else Step('ok', t_s)

       def _start(self, line: Line, kind: str) -> Step:
           self.reset()
           self.prev = line
           return Step(kind, line.t_us * 1e-6)
   ```

   - Only lines that fit become the reference, so one corrupt line costs one line, not two.
   - `t_s` restarts near 0 at each `'first'` and `'reset'`. The clock fit restarts with it,
     so the jump doesn't matter.

   **The clock fit** (decision 4). It works on `d = arrival − t_send`, minus the first `d` seen
   (`d_ref`). `arrival` is ROS time, about 1.8·10⁹ s. Float64 resolves that to 0.24 µs, which is
   enough, but subtracting `d_ref` keeps the numbers in `polyfit` in ms.

   ```python
   class ClockFit:
       def __init__(self, window_s=300.0, block_s=10.0, latency_floor_s=0.002,
                    jump_warn_s=0.001):
           self.block_s = block_s
           self.latency_floor_s = latency_floor_s
           self.jump_warn_s = jump_warn_s
           self.minima = deque(maxlen=max(2, round(window_s / block_s)))
           self.reset()

       def reset(self):
           self.minima.clear()       # (t_send, d − d_ref) of each closed block's lowest line
           self.block = None         # index of the open block, counted from t_first
           self.low = None           # (t_send, d − d_ref) of the open block's lowest line so far
           self.t_first = None
           self.d_ref = None
           self.eps, self.c = 0.0, 0.0   # lower edge: d − d_ref = c + eps · t_send
           self.rms = math.nan           # of the block minima about the fitted line

       @property
       def ready(self) -> bool:
           return len(self.minima) >= 2

       def add(self, t_send: float, arrival: float) -> float | None:
           # Returns the shift of the mapped time at t_send if a refit moved it > jump_warn_s.
           if self.d_ref is None:
               self.t_first, self.d_ref = t_send, arrival - t_send
           d = arrival - t_send - self.d_ref
           k = int((t_send - self.t_first) // self.block_s)
           jump = None
           if self.block is not None and k != self.block:   # the open block has closed
               was_ready = self.ready
               before = self.to_ros(t_send)
               self.minima.append(self.low)
               self.low = None
               if self.ready:
                   self._fit()
                   shift = self.to_ros(t_send) - before
                   if was_ready and abs(shift) > self.jump_warn_s:
                       jump = shift
           self.block = k
           if self.low is None or d < self.low[1]:
               self.low = (t_send, d)
           if not self.ready:                               # warm-up: lowest so far, no slope
               self.c = min([m[1] for m in self.minima] + [self.low[1]])
           return jump

       def _fit(self):
           t, d = np.array(self.minima).T
           eps, c = np.polyfit(t, d, 1)
           self.eps, self.c = float(eps), float(c)
           self.rms = float(np.sqrt(np.mean((d - (c + eps * t)) ** 2)))

       def to_ros(self, t_arduino: float) -> float:
           return t_arduino + self.d_ref + self.c + self.eps * t_arduino - self.latency_floor_s

       def status(self) -> dict:
           return dict(ready=self.ready, blocks=len(self.minima),
                       span_s=len(self.minima) * self.block_s,
                       eps_ppm=self.eps * 1e6, rms_s=self.rms,
                       floor_s=self.latency_floor_s)
   ```

   - `to_ros` takes any Arduino time, not only a send time. The node passes the time the sound
     reached the water (change 3, `publish`).
   - Why no outlier rejection on the minima: a block holds 200 lines, and its lowest delay is
     close to d₀ unless all 200 were delayed. With 3 ms mean exponential latency the expected
     minimum is 15 µs above d₀. The `rms` in the status shows it if that ever fails.
   - A gap in the data (a block with no lines) only skips a block index. The fit is over the
     blocks there are.
   - `t_first` sets where blocks start, so the blocks are aligned the same way for a whole run.

3. **`wave_probe/src/wave_probe_node.py` (new): the node.**

   **Setup.** The parameters are read once at start, except `air_temp_c` (the callback
   below). The counters are written only by the reader thread and read by the status timer.
   They are plain ints, so no lock is needed. The no-echo share keeps a running count next to
   its deque, because summing a deque while the other thread appends to it can raise
   `RuntimeError: deque mutated during iteration`.

   ```python
   import threading
   import time
   from collections import deque

   import rclpy
   import serial
   from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
   from rclpy.executors import ExternalShutdownException
   from rclpy.node import Node
   from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
   from rclpy.time import Time
   from sensor_msgs.msg import Range

   from probe_serial import ClockFit, Sequence, parse_line, range_m

   READ_TIMEOUT_S = 0.5
   CLOCK_LOG_S = 300.0
   DROPOUT_LINES = 1200          # 60 s at 20 Hz
   OPEN_RESET_S = 3.0            # a restart this soon after opening is the Uno's reset
   AIR_TEMP_RANGE = (-10.0, 50.0)


   class WaveProbeNode(Node):
       def __init__(self):
           super().__init__('wave_probe_node')
           p = lambda name, default: self.declare_parameter(name, default).value
           self.port_name = p('port', '/dev/wave_probe')
           self.baud = p('baud', 115200)
           self.frame_id = p('frame_id', 'wp_us1')
           self.min_range = p('min_range_m', 0.02)
           self.max_range = p('max_range_m', 4.0)
           self.fov = p('field_of_view_rad', 0.26)
           self.reopen_s = p('reopen_s', 2.0)
           self.dropout_warn = p('dropout_warn', 0.10)
           # dynamic_typing: `-p air_temp_c:=21` arrives as an int and would otherwise refuse
           # to start against the float default
           self.air_temp_c = float(self.declare_parameter(
               'air_temp_c', 20.0, ParameterDescriptor(dynamic_typing=True)).value)
           self.clock = ClockFit(p('fit_window_s', 300.0), p('fit_block_s', 10.0),
                                 p('latency_floor_s', 0.002))
           self.sequence = Sequence()
           self.add_on_set_parameters_callback(self.on_set)

           qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=100,
                            durability=DurabilityPolicy.VOLATILE)
           self.pub = self.create_publisher(Range, 'probe_range', qos)

           self.started = time.monotonic()
           self.opened = time.monotonic()  # when the port was last opened
           self.lost_total = 0       # lines lost to seq gaps, since start
           self.bad_total = 0        # lines that didn't parse or didn't fit
           self.inf = deque(maxlen=DROPOUT_LINES)
           self.n_inf = 0            # True values in self.inf
           self.fit_logged = False
           self.last_clock_log = time.monotonic()
           self.create_timer(1.0, self.log_status)

           self.stopping = threading.Event()
           self.reader = threading.Thread(target=self.read_loop, daemon=True)
           self.reader.start()
   ```

   **The reader thread** (decisions 2, 5, 6). The arrival time is taken right after
   `readline` returns, before anything else. pyserial's `readline` returns what it has when the
   0.5 s timeout runs out, so a line without `\n` is a timeout. If it had bytes, the rest of
   that line comes in the next read, and that one is dropped too.

   ```python
       def read_loop(self):
           announce = True                   # log the first failed open, not every retry
           while not self.stopping.is_set():
               port = self.open_or_wait(announce)
               if port is None:
                   return                    # stopping
               self.opened = time.monotonic()
               self.sequence.reset()
               self.clock.reset()
               self.fit_logged = False
               skip = True                   # the first line may start mid-line (decision 5)
               try:
                   port.reset_input_buffer()
                   while not self.stopping.is_set():
                       raw = port.readline()
                       arrival = self.get_clock().now().nanoseconds * 1e-9
                       if not raw.endswith(b'\n'):
                           skip = skip or bool(raw)     # half a line: drop its tail too
                           continue
                       if skip:
                           skip = False
                           continue
                       line = parse_line(raw)
                       if line is None:
                           self.bad_total += 1
                           continue
                       self.on_line(line, arrival)
               except (serial.SerialException, OSError) as e:
                   self.get_logger().warning(
                       f'{self.port_name}: {e}; retrying every {self.reopen_s:g} s')
                   announce = False
               finally:
                   port.close()

       def open_or_wait(self, announce: bool) -> serial.Serial | None:
           while not self.stopping.is_set():
               try:
                   # exclusive: a miniterm left open can't steal lines from the node
                   return serial.Serial(self.port_name, self.baud,
                                        timeout=READ_TIMEOUT_S, exclusive=True)
               except serial.SerialException as e:
                   if announce:
                       self.get_logger().warning(
                           f'{self.port_name}: {e}; retrying every {self.reopen_s:g} s')
                       announce = False
               self.stopping.wait(self.reopen_s)
           return None
   ```

   **Each line.** The clock fit gets the send time, and only for lines with an echo
   (decision 4). Every line that fitted the sequence is published, `+inf` included.

   ```python
       def on_line(self, line, arrival):
           step = self.sequence.push(line)
           log = self.get_logger()
           if step.kind == 'bad':
               self.bad_total += 1
               return
           if step.kind == 'first':
               log.info(f'{self.port_name} open; first line at seq {line.seq}')
           elif step.kind == 'reset':
               self.clock.reset()
               self.fit_logged = False
               if time.monotonic() - self.opened < OPEN_RESET_S:
                   # opening the port resets an Uno; lines from before it can still arrive first
                   log.info(f'board restarted on open, at seq {line.seq}; clock fit restarted')
               else:
                   log.warning(f'restart at seq {line.seq} (board reset, or '
                               f'{self.sequence.max_bad} lines that did not fit); '
                               f'clock fit restarted')
           elif step.kind == 'gap':
               self.lost_total += step.lost
               minutes = (time.monotonic() - self.started) / 60
               log.warning(f'seq {line.seq - step.lost - 1} -> {line.seq}: {step.lost} lines '
                           f'lost ({self.lost_total} in {minutes:.0f} min)')
           if line.echo_us > 0:
               jump = self.clock.add(step.t_s + line.echo_us * 1e-6, arrival)
               if jump is not None:
                   log.warning(f'clock refit moved the stamps by {jump * 1e3:+.1f} ms')
           if self.clock.d_ref is None:        # no echo yet since the (re)start: no clock
               return
           is_inf = line.echo_us == 0
           if len(self.inf) == self.inf.maxlen:
               self.n_inf -= self.inf[0]
           self.inf.append(is_inf)
           self.n_inf += is_inf
           self.publish(line, step.t_s)

       def publish(self, line, t_trigger):
           msg = Range()
           t_water = t_trigger + line.echo_us * 1e-6 / 2          # decision 3
           msg.header.stamp = Time(nanoseconds=round(self.clock.to_ros(t_water) * 1e9)).to_msg()
           msg.header.frame_id = self.frame_id
           msg.radiation_type = Range.ULTRASOUND
           msg.field_of_view = self.fov
           msg.min_range, msg.max_range = self.min_range, self.max_range
           msg.range = range_m(line.echo_us, self.air_temp_c)
           try:
               self.pub.publish(msg)
           except Exception:
               if self.context.ok():
                   raise
               self.stopping.set()  # Ctrl-C shut rclpy down under the thread: stop quietly
   ```

   - **Ctrl-C:** rclpy's SIGINT handler shuts the context down before `main` gets to
     `stop()`, and the thread can publish once more in between (`RCLError: publisher's context
     is invalid`, seen on the RPi4 on 2026-11-23). `publish` stops the thread quietly then, and
     still raises while the context is fine. `RCLError` is in a private module, hence
     `except Exception`.
   - **A restart right after opening is logged as info.** The Uno's USB chip still holds lines
     from before the open's reset, so the first lines can be old ones (seq 2854 on the RPi4),
     followed by seq 0 after the bootloader.
   - Until the first line with an echo after an open or a reset, there is no clock to stamp
     with (`d_ref` is `None`), so those lines are dropped. On still water that is one line.

   **The status timer** (1 s, in the executor). The gaps are logged as they come, by the thread.
   The timer logs the first fit once, the clock every 5 min, and the no-echo share when it is
   above `dropout_warn`, at most once a minute.

   ```python
       def log_status(self):
           log = self.get_logger()
           n = len(self.inf)
           if n == DROPOUT_LINES and self.n_inf / n > self.dropout_warn:
               log.warning(f'{100 * self.n_inf / n:.0f} % no echo in the last 60 s',
                           throttle_duration_sec=60.0)
           s = self.clock.status()
           if s['ready'] and not self.fit_logged:
               self.fit_logged = True
               log.info(f"clock: first fit, {s['eps_ppm']:+.0f} ppm")
           if time.monotonic() - self.last_clock_log >= CLOCK_LOG_S:
               self.last_clock_log = time.monotonic()
               log.info(f"clock: {s['eps_ppm']:+.0f} ppm, rms {s['rms_s'] * 1e3:.2f} ms over "
                        f"{s['blocks']} blocks ({s['span_s']:.0f} s), "
                        f"floor {s['floor_s'] * 1e3:.1f} ms (set); "
                        f"{self.lost_total} lost, {self.bad_total} bad")
   ```

   **`air_temp_c` while running.** The callback checks all parameters first and changes
   nothing unless all pass. The others are read at start only, so setting them is refused,
   not silently ignored. The thread reads `self.air_temp_c` once per line; a float assignment
   is atomic, so the next line uses the new value.

   ```python
       def on_set(self, params):
           for prm in params:
               if prm.name != 'air_temp_c':
                   return SetParametersResult(successful=False,
                                              reason=f'{prm.name} is read at start only')
               if not isinstance(prm.value, (int, float)) or not (
                       AIR_TEMP_RANGE[0] <= prm.value <= AIR_TEMP_RANGE[1]):
                   return SetParametersResult(successful=False,
                                              reason=f'air_temp_c must be {AIR_TEMP_RANGE} °C')
           for prm in params:
               old, self.air_temp_c = self.air_temp_c, float(prm.value)
               change = range_m(1, self.air_temp_c) / range_m(1, old) - 1
               self.get_logger().info(f'air_temp_c {old:g} -> {self.air_temp_c:g} °C '
                                      f'(ranges {100 * change:+.2f} %)')
           return SetParametersResult(successful=True)
   ```

   **Stopping.** The thread sees `stopping` within one read timeout and closes the port in its
   `finally`. `join` waits a little longer than that.

   ```python
       def stop(self):
           self.stopping.set()
           self.reader.join(timeout=2 * READ_TIMEOUT_S)


   def main():
       rclpy.init()
       node = WaveProbeNode()
       try:
           rclpy.spin(node)
       except (KeyboardInterrupt, ExternalShutdownException):
           pass
       finally:
           node.stop()
           node.destroy_node()
           rclpy.try_shutdown()


   if __name__ == '__main__':
       main()
   ```

4. **`wave_probe/config/wave_probe.yaml` (new).**

   ```yaml
   /**/wave_probe_node:
     ros__parameters:
       port: /dev/wave_probe
       baud: 115200
       frame_id: wp_us1
       air_temp_c: 20.0
       min_range_m: 0.02
       max_range_m: 4.0
       field_of_view_rad: 0.26
       fit_window_s: 300.0
       fit_block_s: 10.0
       latency_floor_s: 0.002
       reopen_s: 2.0
       dropout_warn: 0.10
   ```

5. **The probe sketch's change 4 (`rpi/probe.sh`)** runs
   `ros2 run wave_probe wave_probe_node --ros-args -r __ns:=/wavemakers/$RIG_PI --params-file <share>/config/wave_probe.yaml -p air_temp_c:=$AIR_TEMP_C`.

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

## Later, not in the first pass

- **More probes on one Arduino:** a fourth field, `probe`, and one topic per probe.
- **`diagnostic_msgs`:** publish the clock fit, the gaps and the dropout share on `/diagnostics`
  instead of only logging them, so the RPi5 or the web page can show the probe's state.
