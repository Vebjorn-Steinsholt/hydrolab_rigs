# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

What to code for the vertical oscillator, and why. Everything on the rig (the drive's
parameters, switching drives, test procedures, measured results, Test 1 and Test 2, the PROFIBUS
diagnosis log) is in
[COMMISSIONING.md, *Vertical oscillator*](COMMISSIONING.md#vertical-oscillator-ladertanken).

Line numbers are as of `feat-vertical-oscillator` on 2026-11-10 where checked; older ones may
be a few lines off.

## Now

1. **T11-T15 done in the repo 2026-11-10** ([Done](#done-code)); on the Pi with T16.
2. **Test 2 failed 2026-11-11** (both rigs: gateway stalls up to 401 ms; alone: none).
3. **T17: log slow gateway cycles**, then Test 2 again
   ([T17](#t17-log-slow-gateway-cycles)). Which request stalls picks the remedy.
4. **The remedy** ([If Test 2 fails](#if-test-2-fails)), then Test 2 must pass.
5. **T16 on the Pi**, after that: `/etc/default/oscillator`, installing the units, the rig checks
   ([T16](#t16-install-on-the-pi)).
6. **T5's last check** when the sloshing rig is connected.
7. T10 is optional and not needed now.

| # | File | Where | What | Done when | Left (2026-11-10) |
|---|---|---|---|---|---|
| T17 | `mgate_driver.h/.cpp`, `indradrive_actuator.hpp/.cpp` | *T17: log slow gateway cycles* | Count and time slow gateway cycles; log each | `MGate cycle ... ms` lines in the journal during Test 2 | Next |
| T16 | the ladertanken Pi: `/etc/default/oscillator`, `/etc/systemd/system/` | *T16: install on the Pi* | Port line, copy the units, build, the rig checks | The rig checks listed there | After the remedy and a passed Test 2 |
| T5 | the ladertanken Pi | — | Decision 6: the last check | The sloshing rig still starts | When the sloshing rig is connected (the refusal passed 2026-11-10) |
| T10 (optional, Claude's: `mgate_example` handed over 2026-11-10) | `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt` | `main()` (87-116), `test_move()` (58-83); `install(TARGETS ...)` (49) | `--step`, `--speed`, `--min`/`--max` (tag units) and installing `mgate_example` | `ros2 run mgate5101_driver mgate_example --move --profibus-address 3 --input-base 7 --output-base 6 --step 50` moves 50 mm and back | Not needed now |

## Scope (2026-11-06)

- **Phase 2, since 2026-11-10: daisy chain.** Both drives stay on the line; the oscillator is
  PROFIBUS address 3 (MGate slave 3, registers 7 / 6), the wavemaker keeps 2 (0 / 0). One rig
  still runs at a time; running the wavemaker and the vertical oscillator together is TODO 47
  (T11-T15). Phase 1 (2026-11-06 – 2026-11-10) replugged one cable between the drives at
  address 2.
- **The oscillator is developed by playing motion files** (`play_dat --rig oscillator`).
- **You write the code.** Each task says what to build, where, and how to check it. Claude
  writes tests and docs, and maintains `mgate_example` (handed over 2026-11-10).

## Goal

A second Rexroth IndraDrive C (rotary motor, linear drive scaling) runs a vertical oscillator.
It is daisy-chained behind the wavemaker drive on the ladertanken MGate (`192.168.1.70`) and
runs from the ladertanken Pi, playing motion files, with the same safety behaviour as the wavemaker.

## The drive, as the code sees it

The full parameter read-out and the comparison with the wavemaker are in
[COMMISSIONING.md, *The drive*](COMMISSIONING.md#the-oscillator-drive-read-2026-11-06).

| | Value | Meaning for the code |
|---|---|---|
| PROFIBUS address | 3 | MGate slave 3, inputs at register 7, outputs at 6; the wavemaker is slave 2 at 0 / 0 |
| Scaling type | linear (the drive converts motor turns to mm) | `drive_motion: linear` |
| Position resolution | 0.0001 mm | Tag factor 0.0001 → the "deg" tag is **mm** |
| Velocity resolution | 0.001 mm/min | Tag factor 0.0001 → the "rpm" tag is **10 mm/min** per unit (1 mm/s = 6 units) |
| Acceleration, jerk | 20 m/s², 3 m/s³ (S-0-0260, S-0-0193) | Not in the cyclic data; they limit every move (decision 5) |
| Position limits | −211 … +211 mm | The drive's backstop; the controller's travel goes inside it |
| Direction | **+ command = down** | The rig uses + = up: `drive_direction: -1` (decision 4) |
| Power section | Needs mains switched on at the cabinet | Without it `enable()` times out waiting for `Ab` |

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
- **Linear IndraDrive scaling (TODO item 19):** `drive_m_per_unit`,
  `drive_velocity_per_unit_per_s`, `drive_direction`, limits from travel (T2, 2026-11-09).
- **Only one rig runs on the Pi at a time (TODO 47 will allow wavemaker + vertical).**
  `wavemaker@` and `oscillator@` refuse to start while the other runs (`rpi/refuse_if_busy.sh`,
  which also catches a launch by hand through the web port), and `windows/_rig.bat` stops the
  other before starting one. The oscillator launch takes `oscillator:=sloshing|vertical`, set
  through `OSCILLATOR=` in `/etc/default/oscillator` (T5).
- **`play_dat` already plays to an oscillator** (`--rig oscillator`, action `move_oscillator`).
  With a rig name it reads limits and namespace from `oscillator.yaml` (`rig_limits.py:20`).
  For a linear IndraDrive, `rig_limits` derives the speed limit from `max_velocity_rpm` and the
  drive's units (T8), so `--max-velocity-mps` is optional. `generate_waves` is wavemakers only;
  the oscillators have `generate_oscillations` (T7).
- **`mgate_example` reaches the oscillator with options:** `--input-base 7 --output-base 6
  --profibus-address 3` (`drivers/mgate5101_driver/examples/example_main.cpp`); without them it is
  the wavemaker. `--move` refuses without `--profibus-address` and steps 5 mm at 1.7 mm/s on this
  drive. Its limits default to −550…500 (`IndraDriveConfig`), so the drive's ±211 mm are the
  only backstop. Built but not installed (`~/ros_ws/build/mgate5101_driver/mgate_example`).

## Decisions

All implemented on 2026-11-09 except where noted.

1. **Phase 1 replugging ended 2026-11-10 with phase 2.** Both drives are on the line at their
   own addresses and blocks, so each yaml can only reach its own drive.
2. **The oscillator drive speaks the layout `IndraDrive` expects** (checked 2026-11-06, read
   only). No configurable tag table.
3. **Use the existing `rig: oscillator` path with `drive_name: indradrive`.** No new actuator
   class. Nothing may assume `rig: oscillator` means LinMot.
4. **Linear drive scaling (TODO item 19),** with parameters whose defaults keep today's angular
   behaviour:
   - `drive_m_per_unit`: metres per position tag unit. **0.001** here (0.0001 mm resolution ×
     tag factor 10 000 = 1 mm). It goes into `DriveScale::m_per_unit` in the linear branch.
   - `drive_velocity_per_unit_per_s`: velocity tag units per (position tag unit per second).
     **6.0** here (1 mm/s = 60 mm/min = 6 × 10 mm/min). Angular default **1/6** (deg/s → rpm).
     It replaces the `/ 6` in `write_setpoint()` and the `* 6.0` in `max_velocity_mps()`.
   - `drive_direction`: +1 or −1, the sign between rig coordinates (+ = up) and drive
     coordinates. **−1 here**; default +1 keeps ladertanken unchanged. It applies to setpoints,
     position feedback and the drive limits (min and max swap on the drive side). The velocity
     cap and `max_velocity_mps()` use magnitudes only, so the cap stays positive. The drive's
     S-0-0055 stays as it is.
   - The drive limits come from `travel_minimum_m/maximum_m` (one set of limits, not two).
     `max_velocity_rpm` / `hold_velocity_rpm` stay in velocity tag units (here 10 mm/min each;
     0.1 m/s = 600). The ±211 mm in the drive stay as the backstop.
5. **Vertical axis: gravity, brake, acceleration and jerk.** Control word 0 (our fault path,
   and the gateway fault value) removes torque; the drive's own 5 Nm brake holds the slide.
   - **The drive limits every move.** The controller sends no acceleration
     (`max_acceleration_mps2()` is infinite), so the drive's positioning acceleration (20 m/s²)
     and jerk (3 m/s³) do.
   - A sine of stroke `s` and period `T` needs acceleration `(s/2)(2π/T)²` and jerk
     `(s/2)(2π/T)³`; for example 20 mm at 1 s: 0.39 m/s², 2.5 m/s³. Above 3 m/s³ the drive rounds
     the motion off. `generate_oscillations` warns above it. Below it, the measured stroke loss
     depends on the period, not on the jerk ([Open questions](#open-questions)).
   - `max_goal_acceleration_mps2` and `min_period_s` in the vertical block stay 0 (no limit)
     until the loaded tests show where the lag or the mechanics become a problem; then the
     controller rejects a goal that is too steep instead of the drive rounding it off.
6. **Which rigs run together.** Sloshing never runs with anything; the wavemaker and the
   vertical oscillator may run together (decided 2026-11-10, T11-T15). Until T11-T15 the Pi
   services refuse both directions. Sloshing and vertical are one unit,
   `oscillator@ladertanken`, switched by editing `OSCILLATOR=` in `/etc/default/oscillator`
   (decided 2026-11-10), so systemd itself keeps them apart. No second unit.
7. **Files are played with `play_dat` as they are,** made by `generate_oscillations` (T7, built
   like `generate_waves`); the speed limit comes from the yaml (T8). Single waves also run from
   the web page (height = stroke, period).

## T16: install on the Pi

After T11-T15 are merged into the branch and Test 2 has passed. Everything here needs the Pi
(`ssh ladertanken@ladertanken-rpi.local`); both rigs stopped first.

1. **Pull and build:**

   ```bash
   sudo systemctl stop wavemaker@ladertanken oscillator@ladertanken
   cd ~/ros_ws/src/hydrolab_rigs && git pull
   cd ~/ros_ws && colcon build
   ```

2. **`/etc/default/oscillator`** (`sudoedit /etc/default/oscillator`):

   ```bash
   ROS_DOMAIN_ID=1
   PORT=8081
   OSCILLATOR=vertical
   ```

   `/etc/default/wavemaker` is unchanged (8080 by default). `ROS_DOMAIN_ID` must be the same in
   both files: `soft_stop@` reads the wavemaker's, so an oscillator on another domain never hears
   the button and stays stopped.
3. **Install the units:**

   ```bash
   sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/{wavemaker,oscillator,soft_stop}@.service /etc/systemd/system/
   sudo systemctl daemon-reload
   ```

   The sudoers rule is unchanged: systemd starts `soft_stop@` through `Requires=`.
4. **The rig checks:**
   - wavemaker and vertical start in either order;
   - sloshing is refused in both directions;
   - one press stops both;
   - stopping one rig leaves the other and the button working;
   - stopping the soft stop unit stops both.
5. **Windows:** `start_oscillator.bat` with the wavemaker running leaves the wavemaker running
   and opens port 8081.

Then Claude marks TODO 47 done and rewrites COMMISSIONING.md, *Switching drives* (no stop needed
for the vertical oscillator any more).

## T5: the Pi

Left: one check. The refusal passed 2026-11-10: with `wavemaker@ladertanken` running,
`sudo systemctl start oscillator@ladertanken` failed in `ExecStartPre` (`refuse_if_busy.sh`).

1. **The sloshing rig, when it is connected.** Remove `OSCILLATOR` from `/etc/default/oscillator`
   (or set `sloshing`) and start; the journal shows `LinMot: homed` and the nodes are in
   `/oscillators/sloshing`.
2. **Afterwards.** The Pi stays on `feat-vertical-oscillator`, so the wavemaker runs this branch's
   code too. T2 leaves ladertanken's numbers unchanged (the tests check it), but the first wavemaker
   run after the switch should be a small wave. Back to `master` once the branch is merged:
   `git checkout master && git pull`, `colcon build`.

## If Test 2 fails

**Test 2 failed 2026-11-11** (COMMISSIONING.md, *Test 2*): with both rigs, the wavemaker's
gateway cycle stalled about 200 ms (once 401 ms) eleven times in 10 min; alone, never in 16 min.
T16 waits. T17 first shows which request stalls; that picks the remedy.

### T17: log slow gateway cycles

Today a stalled cycle leaves no trace: the controller keeps publishing setpoints, and only the
bag shows the position standing still. The driver has no logger (it is not ROS code), so it
counts slow cycles in `DriverStats` and times each of the three requests; the actuator logs
them. Line numbers as of 2026-11-11.

1. **`drivers/mgate5101_driver/include/mgate/mgate_driver.h`**
   - `GatewayConfig`, after line 65 (`reconnect_ms`):

     ```cpp
         int slow_cycle_ms    = 100;   // a longer cycle counts in DriverStats::slow_cycles
     ```

   - `DriverStats`, after line 77 (`last_cycle_us`):

     ```cpp
         long long     max_cycle_us    = 0;
         std::uint64_t slow_cycles     = 0;  // cycles longer than GatewayConfig::slow_cycle_ms
         // The last slow cycle, and its three requests, in microseconds.
         long long     slow_cycle_us   = 0;
         long long     slow_write_us   = 0;
         long long     slow_inputs_us  = 0;
         long long     slow_status_us  = 0;
     ```

   - Private members, after line 167 (`wake_cv_`):

     ```cpp
         std::array<long long, 3> phase_us_{};  // write, inputs, status of this cycle (poll thread)
     ```

     and `#include <array>` with the other includes.

2. **`drivers/mgate5101_driver/src/mgate_driver.cpp`**
   - Replace `exchange()` (lines 364-369) with:

     ```cpp
     Result MGateDriver::exchange() {
         // Times each request, so a slow cycle shows which one stalled (DriverStats::slow_*_us).
         phase_us_ = {};
         const auto timed = [](long long& us, auto&& request) {
             const auto start = std::chrono::steady_clock::now();
             Result r = request();
             us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - start)
                      .count();
             return r;
         };
         if (Result r = timed(phase_us_[0], [this] { return flush_outputs(); }); !r) return r;
         if (Result r = timed(phase_us_[1], [this] { return read_inputs(); }); !r) return r;
         if (Result r = timed(phase_us_[2], [this] { return read_status(); }); !r) return r;
         return {};
     }
     ```

   - `poll_loop()`, in the stats block after `stats_.last_cycle_us = ...;` (lines 394-396):

     ```cpp
                 stats_.max_cycle_us = std::max(stats_.max_cycle_us, stats_.last_cycle_us);
                 if (stats_.last_cycle_us > 1000LL * cfg_.slow_cycle_ms) {
                     ++stats_.slow_cycles;
                     stats_.slow_cycle_us  = stats_.last_cycle_us;
                     stats_.slow_write_us  = phase_us_[0];
                     stats_.slow_inputs_us = phase_us_[1];
                     stats_.slow_status_us = phase_us_[2];
                 }
     ```

3. **`rig_controller/src/indradrive_actuator.hpp`**, private, after line 50 (`has_target_`):

   ```cpp
     // Set in start(); report_slow_cycles() logs each new slow gateway cycle.
     rclcpp::Logger logger_{rclcpp::get_logger("indradrive_actuator")};
     std::uint64_t slow_cycles_seen_{0};
     void report_slow_cycles();
   ```

4. **`rig_controller/src/indradrive_actuator.cpp`**
   - `start()`, after `mgate_driver_->start();` (line 157):

     ```cpp
       logger_ = logger;
       slow_cycles_seen_ = 0;
     ```

   - `write_setpoint()` (line 235), after its `if (!indradrive_) { return false; }` (lines
     237-239), add one line:

     ```cpp
       report_slow_cycles();   // log any slow gateway cycle since the last call
     ```

     The controller calls `write_setpoint()` every 20 ms while a wave or a return runs, which is
     when a stall delays the drive's targets. When no goal runs, slow cycles are not logged.
   - New, after `write_setpoint()`:

     ```cpp
     // A slow gateway cycle delays the targets and the feedback; over 500 ms the MGate's fault
     // timeout switches the drive off (Test 2, 2026-11-11: up to 401 ms with both rigs).
     void IndraDriveActuator::report_slow_cycles()
     {
       const auto stats = mgate_driver_->stats();
       if (stats.slow_cycles == slow_cycles_seen_) {
         return;
       }
       RCLCPP_WARN(
         logger_,
         "MGate cycle %lld ms (write %lld, inputs %lld, status %lld ms); %llu slow so far, "
         "longest %lld ms",
         stats.slow_cycle_us / 1000, stats.slow_write_us / 1000, stats.slow_inputs_us / 1000,
         stats.slow_status_us / 1000, static_cast<unsigned long long>(stats.slow_cycles),
         stats.max_cycle_us / 1000);
       slow_cycles_seen_ = stats.slow_cycles;
     }
     ```

5. **Tests (Claude, done 2026-11-11):** `FakeMGate::delay_next()` holds one request's reply;
   `NoSlowCyclesWithoutAStall`, `SlowCycleShowsWhichRequestStalled` (write, inputs, status in
   turn), `WriteSetpointLogsEachSlowCycleOnce` (`test_indradrive_actuator.cpp`).

**On the Pi** (the branch has T11-T15, so no `git pull` before T16; take only these files):

```bash
sudo systemctl stop wavemaker@ladertanken
cd ~/ros_ws/src/hydrolab_rigs && git fetch
git checkout origin/feat-vertical-oscillator -- drivers/mgate5101_driver/include/mgate/mgate_driver.h \
  drivers/mgate5101_driver/src/mgate_driver.cpp rig_controller/src/indradrive_actuator.hpp \
  rig_controller/src/indradrive_actuator.cpp
cd ~/ros_ws && colcon build --packages-select mgate5101_driver rig_controller
```

Then Test 2 again (COMMISSIONING.md, *Test 2*, 10 min, both rigs) and:

```bash
journalctl -u wavemaker@ladertanken --since "15 min ago" -o cat --no-pager | grep "MGate cycle"
```

The oscillator's lines are in terminal 2 (its hand launch).

**What the result decides:**

| The slow part | Remedy |
|---|---|
| `status` only | Read the status area every Nth cycle (remedy 2 below). Small. |
| `inputs` or `write`, or all | One shared connection (remedy 1). Its own sketch. |

### Background and remedies

Test 2 asks whether two controllers on one MGate (300 requests/s) cause the ~50-210 ms gateway
stalls. The fault timeout is no reason to merge the processes: Test 1 showed the MGate's timer
runs per slave.

- **Count requests per second, not bandwidth.** Each node makes 3 requests per cycle
  (`MGateDriver::exchange()`, `mgate_driver.cpp:365`). Two nodes at 20 ms make 300 requests/s,
  about 75 kB/s: under 1 % of the Ethernet, and PROFIBUS takes about 1 ms per bus cycle for both
  drives. The limit is the gateway: one client at 20 ms (150 requests/s) was clean, at 10 ms
  (300 requests/s) there were stalls with TCP retransmissions (COMMISSIONING.md, *Verified so
  far*).
- **Remedies, in this order** (each a code change in the controller, next to T11-T15; needed
  before both rigs run together):
  1. one shared connection for both drives (3 requests per cycle for both rigs), so one process
     owns the gateway, both drives on one `MGateDriver`. `set_cycle_callback` then needs more
     than one subscriber (`IndraDrive` takes the only slot today);
  2. the status area only every Nth cycle;
  3. FC23 (probably not usable, since the inputs are FC4);
  4. a slower oscillator poll.
- **Tests that the shared connection needs:** two `IndraDriveActuator`s on one fake gateway
  (`FakeMGate` plays only one drive and counts writes to the other slave today).
- Still to check either way: the MGate's maximum number of Modbus TCP clients (3 worked: two
  controllers and the PC reader, 2026-11-10).

## Open questions

- **Tracking under changing loads (2026-11-09).** Unloaded, the slide follows with a lag of
  about 77 ms (first order) plus 100 ms delay: 97 % of the stroke at 2 s, 90 % at 1 s
  (COMMISSIONING.md, *Waves from the page*). The load is not fixed: different models, profiles,
  water resistance and depths, so one fixed correction cannot hold. Options: (1) a stiffer drive
  (Kv S-0-0104 = 1.0, acceleration feedforward S-0-0348 / P-0-1126 = 0; with the drive's owner);
  (2) always record and report the measured motion per run (`oscillator_position`); (3) per-setup
  calibration with a T7 sweep and a gain table per period (the controller's table is wavemaker
  only today); (4) amplitude correction during the run (closed loop, steady sines only).
  Recommended: 1 and 2 first. **To decide with the experimenters:** must the slide reach the
  requested stroke exactly, or is it enough to know exactly what it did?
  **Tuning (option 1) waits for Robert, back about 2026-11-23.** Until then: code.
- Travel, stroke, max speed and rest position wanted for the vertical oscillator (they set the
  yaml).

The drive and gateway questions (GSD, P-0-4088, jerk, brake, counterbalance, the button) are in
COMMISSIONING.md, *Vertical oscillator*, *Open*.

## Done (code)

All in git; the commit history has the details.

- **T11-T15** (2026-11-10): oscillator port 8081; `run.sh` writes `/run/oscillator-<user>/oscillator`
  and `stop.sh` reads it; `refuse_if_busy.sh <rig> <user>` refuses only sloshing next to the
  wavemaker (starting and stopping units count as running); one `soft_stop@.service` publishing to
  every controller on the Pi (`topics` parameter, `soft_stop:=false` in both launches,
  `Requires=` in both units); `_rig.bat` starts only the requested unit. Tests:
  `test_soft_stop.py`, `test_launch_soft_stop.py`, `test_refuse_if_busy.py`.
- **Bridge** (2026-11-11, `c4c1a4a`): logs `Stop requested: <reason>` for every stop.
- **T1** (2026-11-10): `IndraDrive` refuses blocks other than 7 in / 6 out (failed configure).
- **Daisy chain:** no code change needed (per-controller blocks and PROFIBUS address).
- **`mgate_example`** (2026-11-10): `--input-base`, `--output-base`, `--profibus-address`.
- **Tests** (2026-11-10): `FakeMGate` with a second slave; three daisy-chain tests in
  `test_indradrive_actuator.cpp`.
- **`wait_for_data()` race** fixed (2026-11-09).
- **T2** linear scaling, **T5** `oscillator:=` and the refusal, **T7** `generate_oscillations`,
  **T8** speed limit from the yaml, **T9** (2026-11-09 – 2026-11-10). T6 not needed.
