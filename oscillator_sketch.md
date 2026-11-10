# Sketch: IndraDrive C vertical oscillator on the ladertanken MGate

What to code for the vertical oscillator, and why. Everything on the rig (the drive's
parameters, switching drives, test procedures, measured results, Test 1 and Test 2, the PROFIBUS
diagnosis log) is in
[COMMISSIONING.md, *Vertical oscillator*](COMMISSIONING.md#vertical-oscillator-ladertanken).

Line numbers are as of `feat-vertical-oscillator` on 2026-11-10 where checked; older ones may
be a few lines off.

## Now

1. **T11-T15, done in the repo 2026-11-10**
   ([Code: wavemaker + vertical together](#code-wavemaker--vertical-together-todo-47-2-5-proposed-2026-11-10)).
   They are needed whatever Test 2 shows.
2. **Test 2 on the rig, 2026-11-11** (gateway load with both rigs,
   [COMMISSIONING.md](COMMISSIONING.md#test-2-gateway-load-with-both-rigs-next)), with the Pi as
   it is today (hand launch). If it fails, [If Test 2 fails](#if-test-2-fails) is added to the
   controller before both rigs run together.
3. **T16 on the Pi**, after T11-T15 and a passed Test 2: `/etc/default/oscillator`, installing
   the units, the rig checks ([T16](#t16-install-on-the-pi)).
4. **T5's last check** when the sloshing rig is connected.
5. T10 is optional and not needed now.

| # | File | Where | What | Done when | Left (2026-11-10) |
|---|---|---|---|---|---|
| T11-T15 | `rpi/` (units, `run.sh`, `stop.sh`, `refuse_if_busy.sh`, new `soft_stop@.service` + `soft_stop.sh`), `soft_stop.py`, both launches, `windows/_rig.bat` | *Code: wavemaker + vertical together* | Port 8081 by default, oscillator marker, refuse only sloshing, one shared soft stop, Windows start without stopping the other | Tests pass on the PC; `systemd-analyze verify` on the units | Done 2026-11-10 (tests: `test_soft_stop.py`, `test_launch_soft_stop.py`, `test_refuse_if_busy.py`; docs updated) |
| T16 | the ladertanken Pi: `/etc/default/oscillator`, `/etc/systemd/system/` | *T16: install on the Pi* | Port line, copy the units, build, the rig checks | The rig checks listed there | After T11-T15 and Test 2 |
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

## Code: wavemaker + vertical together (TODO 47 (2)-(5), proposed 2026-11-10)

Coded now in the repo; installed on the Pi by T16 after Test 2 passes. **Rules (decided 2026-11-10):** sloshing never runs with anything;
wavemaker + vertical may run together; one soft stop button stops both. Test 1 showed separate
controller processes are safe. Everything below is your code unless handed over; Claude writes
the docs (`rpi/README.md`, TODO 47) and the tests.

**T11: own web port for the oscillator, by default.** The repo side only; the line in
`/etc/default/oscillator` is T16. The oscillator's port defaults to 8081 wherever it is read, so
a Pi whose env file has no `PORT` still gets it right:
- `run.sh`: `port:="${PORT:-8081}"` in the oscillator branch (T12's snippet);
- `refuse_if_busy.sh`: the default depends on the rig (in T13's script);
- `windows/_rig.bat`: opens the page on 8081 (T15).

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

`run.sh`, after the `ROS_DOMAIN_ID` check (line 5) and **before** the two `source` lines (about
0.5 s): the unit is `active` as soon as `run.sh` starts (`Type=exec`), and a wavemaker start in
that gap would find no marker and refuse ("unknown").

```bash
# Which oscillator runs, for refuse_if_busy.sh and stop.sh (the env file can change meanwhile).
# Before sourcing ROS (about 0.5 s), so the wavemaker's unit finds it as soon as this unit is active.
if [[ ${RIG:-wavemaker} == oscillator && -n "$RUNTIME_DIRECTORY" ]]; then
  echo "${OSCILLATOR:-sloshing}" > "$RUNTIME_DIRECTORY/oscillator"
fi
```

and the oscillator branch keeps only:

```bash
    args=(port:="${PORT:-8081}" oscillator:="${OSCILLATOR:-sloshing}" soft_stop:=false)
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

```ini
ExecStartPre=/home/%i/ros_ws/src/hydrolab_rigs/rpi/refuse_if_busy.sh wavemaker %i    # wavemaker@
ExecStartPre=/home/%i/ros_ws/src/hydrolab_rigs/rpi/refuse_if_busy.sh oscillator %i   # oscillator@
```

`systemctl is-active --quiet` succeeds only for `active`, so a unit in its own `ExecStartPre`
(`activating`) or still stopping its drive (`deactivating`, up to 45 s) would count as not
running; two starts within about 100 ms could both pass. `running()` counts those states too:

| Situation | Result |
|---|---|
| Oscillator in `ExecStartPre` (no marker yet), wavemaker starts | Refused ("unknown"); start again a second later |
| Both start at the same moment | Both may refuse; nothing runs; start one after the other |
| Sloshing stopping (marker still there), wavemaker starts | Refused until the stop is done (`systemctl stop` blocks, so `stop_rigs.bat` waits anyway) |
| Vertical running, wavemaker starts | Allowed |

```bash
#!/usr/bin/env bash
# ExecStartPre: refuse combinations that cannot run together on this Pi. The wavemaker and the
# vertical oscillator may run together (TODO 47); the sloshing oscillator only alone.
# $1: this unit's rig (wavemaker | oscillator), $2: the Pi user (%i).
set -eo pipefail
rig="$1" pi="$2"
marker="/run/oscillator-$pi/oscillator"   # written by run.sh, removed when oscillator@ stops

# Starting and stopping count as running: the other unit may be in its own ExecStartPre, or
# still shutting its drive down. (is-active exits non-zero for these; ignored inside [[ ]].)
running() { [[ $(systemctl is-active "$1") =~ ^(active|activating|deactivating|reloading)$ ]]; }

if [[ $rig == wavemaker ]]; then
  if running "oscillator@$pi"; then
    running_osc="$(cat "$marker" 2>/dev/null || echo unknown)"
    if [[ $running_osc != vertical ]]; then
      echo "oscillator@$pi runs '$running_osc'; only the vertical oscillator can run with the wavemaker" >&2
      exit 1
    fi
  fi
else
  if [[ ${OSCILLATOR:-sloshing} != vertical ]] && running "wavemaker@$pi"; then
    echo "wavemaker@$pi is running; only the vertical oscillator can run with it" >&2
    exit 1
  fi
fi

# Catches a launch by hand on this unit's own port (the same defaults as run.sh).
if [[ $rig == wavemaker ]]; then default_port=8080; else default_port=8081; fi
port="${PORT:-$default_port}"
if ss -Hltn "sport = :$port" | grep -q .; then
  echo "port $port is in use, probably a launch by hand; stop it first" >&2
  exit 1
fi
```

**T14: one soft stop for both (TODO 47 (4)).** GPIO17 can be claimed by one process only. The
node leaves the launches and gets a unit of its own, which publishes to every controller that can
run on the Pi; no controller is remapped. Line numbers as of 2026-11-10.

1. **`rig_controller/src/soft_stop.py`: a `topics` parameter.** The default `['soft_stop']`
   keeps a launch by hand as today.
   - After line 19 (`internal_pull_up`), add:

     ```python
             self.topics = self.declare_parameter('topics', ['soft_stop']).value
     ```

   - Replace line 41 (`self.publisher = ...`) with:

     ```python
             # Not self.publishers: rclpy's Node already has that name.
             self.stop_publishers = [self.create_publisher(Bool, t, qos) for t in self.topics]
     ```

   - Replace line 43 (the `info` log) with:

     ```python
             self.get_logger().info(
                 f"Soft stop on GPIO{self.pin} to {', '.join(self.topics)}, starting stopped")
     ```

   - Replace line 75 (`self.publisher.publish(...)`) with:

     ```python
             msg = Bool(data=self.stopped)
             for publisher in self.stop_publishers:
                 publisher.publish(msg)
     ```

2. **`wavemaker_bringup/launch/lab.launch.py`: leave the node out on request.**
   - Line 15: add `from launch.conditions import IfCondition` below it.
   - After the `soft_stop_pin` argument (lines 44-48), add:

     ```python
             DeclareLaunchArgument(
                 'soft_stop',
                 default_value='true',
                 description='Start a soft stop node; false when soft_stop@.service owns the button',
             ),
     ```

   - In the soft stop `Node(...)` (lines 70-79), after `output='screen',` (line 78), add:

     ```python
                 condition=IfCondition(LaunchConfiguration('soft_stop')),
     ```

3. **`oscillator_bringup/launch/oscillator.launch.py`: the same.**
   - Line 20: add `from launch.conditions import IfCondition` below it.
   - After the `soft_stop_pin` argument (lines 41-42), add:

     ```python
             DeclareLaunchArgument(
                 'soft_stop', default_value='true',
                 description='Start a soft stop node; false when soft_stop@.service owns the button'),
     ```

   - In the soft stop `Node(...)` (lines 85-93), after `output='screen',`, add the same
     `condition=IfCondition(LaunchConfiguration('soft_stop')),`.
   - Line 36 (T11): `port` default `'8080'` → `'8081'`, so a launch by hand does not take the
     wavemaker's port.

4. **New `rpi/soft_stop@.service`:**

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

5. **New `rpi/soft_stop.sh`** (`chmod +x`):

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

6. **`rpi/wavemaker@.service` and `rpi/oscillator@.service`:** replace line 4
   (`After=network-online.target`) with:

   ```ini
   Requires=soft_stop@%i.service
   After=network-online.target soft_stop@%i.service
   ```

7. **`rpi/run.sh`:** `soft_stop:=false` on both launches. Done.

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

**Order:** T11-T15 and Test 2 (either order), then T16.

**Claude, after T11-T15:**
- the docs: `rpi/README.md` (the new unit, `PORT=8081`, start and stop), `windows/README.md`;
- tests: `soft_stop.py`'s `topics`, and the launch argument.

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

- **T1** (2026-11-10): `MGateDriver::config()` and a check at the top of
  `IndraDrive::IndraDrive` (`indradrive.cpp`): `output_word_count` must be 6, `input_word_count`
  at least 7, else `invalid_argument`, which `IndraDriveActuator` turns into a failed configure.
  It does not catch a wrong `output_base_reg`: any base is valid on its own.
- **Daisy chain without code changes:** the tag offsets in `IndraDrive::IndraDrive` are bytes 0,
  2, 6, 10 *inside the block*, and each controller has its own `MGateDriver` with its own
  `input_base_reg` / `output_base_reg` (both starts must be even: 14 input bytes and 12 output
  bytes give exactly registers 7 and 6). `slave_live()` (`indradrive.cpp:50-63`) takes the bit
  for `profibus_address` (address 3 is byte 2, bit 3). The drive that is not running is switched
  off by the MGate's fault timeout on its slave, a gateway setting.
- **`mgate_example`** (2026-11-10): `--input-base`, `--output-base`, `--profibus-address`;
  defaults 0 / 0 / 2; `--move` refuses without `--profibus-address`.
- **Tests** (2026-11-10): `FakeMGate(input_base, output_base, live_list)` plays a second slave in
  front of the drive and counts writes to it; `RejectsBlockSizesOtherThanTheDriveLayout`,
  `DaisyChainedDriveLeavesTheOtherSlaveAlone` (slave 3 at 7 / 6 enables and moves, no write to
  slave 2), `DaisyChainedDriveNotLiveRefusesToStart` (`test_indradrive_actuator.cpp`).
- **`MGateDriver::wait_for_data()` race** (2026-11-09): it returned after the first input read,
  before the status area and `online_` were set. `have_data_` is now set only after a whole
  successful cycle (`mgate_driver.cpp`, `poll_loop()`).
- **T2** linear scaling (2026-11-09), **T5** `oscillator:=` argument and the refusal, **T7**
  `generate_oscillations` with tests and README section, **T8** speed limit from the yaml, **T9**
  (2026-11-10). T6 (an address-check tool) was not needed: a read-only FC4 read of the live list
  and both blocks did the job.
