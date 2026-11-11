# TODO

Open software work, by priority, and planned features for all rigs at the end
([Planned features](#planned-features-all-rigs)). Physical checks on the wavemaker are in
[COMMISSIONING.md](COMMISSIONING.md); the checks before each lab session are in
[STARTUP.md](STARTUP.md).

Item numbers are shared with [COMMISSIONING.md](COMMISSIONING.md) (numbering restarted on
2026-10-06), so references such as "item 33" stay valid in both files; gaps in the numbering are
the items kept there, and finished items, which are removed (see the git history).

- **(rig)** needs the real wavemaker; the unit tests use fakes for the drive and the controller.
- Paths are relative to the repository root.

## High

- [ ] **43. Measure the generated waves with probes. (rig)** (`rig_controller/src/generate_waves.py`, the gain table in `wavemakers.yaml`)
  Item 38 (2026-11-05, `rpi/README.md`, *`generate_waves` on the rig*) played all four runs of `item38.dat` on ladertanken without faults, but no wave probes were available, so the wave heights were not measured. When probes are in the tank: measure the depth, generate the same matrix again (`matrix.csv` is in `~/wave_sessions/2026-11-05_item38` on the Pi, and in the copy taken off it), and play the whole file in one go: `play_dat item38.dat ladertanken`, without `--runs`, which plays runs 0-3 with the 60 s pauses between them, about 8 min in all. `play_dat` logs `run N (...)` with a timestamp as each run starts; use those times to match the probe data. Check on the probes that the tank has calmed before the next run starts; if not, raise `pause_s` and generate again. To repeat a single run, use `--runs N`. It first waits for the rest before run N in the file, about 60 s, and logs nothing while it waits. `item38.dat` has no `.txt` (item 39 came after it); generating it again makes one, and it goes with the probe data.
  Pass: the measured H of regular runs 0-2 (1/60 at 1.0 s, 1/40 at 1.5 s, 0.171 m at 1.3 s) is within the gain table's scatter (~6 %) of the target. Note the measured Hs of irregular run 3 (target 0.03 m, Tp 1.2 s); there is no pass limit yet. The gain table is for 1.0 m: note the depth with the result, which on 2026-11-05 was 0.98 m.
  Measured 2026-11-05, WP1 3 m from the flap as in October (`rpi/README.md`, *`generate_waves` on the rig*; `wave_analysis/sessions/2026-11-05_item38.toml`): run 0 1.03 and run 2 1.05 pass, **run 1 (1.5 s) 1.12 fails** (before the beach reflection; 1.16 with it); run 3 Hm0 0.0347 m, Tp 1.18 s. Against `Calib20oct.BIN` the tank and drive behave as in October (paddle 0.572 of the command, H per paddle amplitude within 3 %). Run 1's excess is the command plus a reflection: 8 % more stroke per metre of target than October's 1.5 s run (gain table 0.75 against the 0.78 that run measured, and the 0.98 m depth), then +6 % from the beach reflection about 21 s into the run, which October's 15 s run never reached.
  Open: (1) the 1.5 s table value: 2026-10-20 measured 0.78, the table has 0.75, and the 2026-10-21 check (0 % with 0.75) is not on the stick to recheck. (2) Runs 0 and 2 also came out 3-5 % high, which fits the depth correction overcorrecting, or a depth other than 0.98 m. (3) Long runs at 1.5 s and above see the beach reflection after about 21 s; shorten the measuring window or separate incident and reflected waves (COMMISSIONING.md, 3 probes). Next: one session at a measured depth with 1.5 s at 0.074, 0.0835 and 0.112 m and 1.0 s / 1/60 before and after, measured before the reflection arrives. Leave the gain table alone until then.
## Medium

- [ ] **19. Linear drives (lilletanken, mclab): set their units.** Deferred: neither is deployed yet (they need gateways). (`wavemaker_bringup/config/wavemakers.yaml`)
  The code is done (2026-11-09, for the vertical oscillator): `drive_m_per_unit`, `drive_velocity_per_unit_per_s` and `drive_direction` in `DriveScale`, one set of limits (`travel_*`, the drive's derived from them), and `rig_limits` derives the speed from `max_velocity_rpm`. lilletanken and mclab still carry placeholders (1.0, 1/6, 1, and `max_velocity_rpm: 100`, which `rig_limits` turns into 600 m/s). Set them from each drive's scaling (item 19 in [COMMISSIONING.md](COMMISSIONING.md)); their `min_position_m` / `max_position_m` are no longer read and can go.

- [ ] **46. Vertical oscillator: tracking under changing loads.** (rig; drive tuning with the drive's owner, from about 2026-11-23)
  Unloaded (2026-11-09) the slide follows with about 180 ms lag; the stroke reaches 97 % at a 2 s period and 90 % at 1 s, for any stroke: a first-order lag of about 77 ms plus about 100 ms delay ([COMMISSIONING.md](COMMISSIONING.md#vertical-oscillator-ladertanken)). The load changes between experiments (models, profiles, water resistance, depth), so one fixed correction cannot hold. Options: (1) a stiffer drive (Kv S-0-0104 = 1.0, acceleration feedforward S-0-0348 / P-0-1126 = 0); (2) record and report the measured motion with every run; (3) a calibration sweep per setup and a gain per period (the controller's gain table is wavemaker-only); (4) amplitude correction during the run. 1 and 2 first. To decide with the experimenters: must the slide reach the requested stroke, or is knowing what it did enough?

- [ ] **47. Run the wavemaker and the vertical oscillator at the same time.** (rig; `rpi/refuse_if_busy.sh`, both units, a new `soft_stop@.service`, `lab.launch.py`, `oscillator.launch.py`, `soft_stop.py`, `windows/`, the MGate)
  Decided 2026-11-10: the sloshing and the vertical oscillator never run together (one unit, `OSCILLATOR=` in `/etc/default/oscillator`), but the wavemaker and the vertical oscillator should. Today `refuse_if_busy.sh` refuses either rig while the other runs, because they share the web port, the soft stop on GPIO17 (to be shared on purpose, see 4) and the MGate. What has to change:
  (1) **The drive:** done 2026-11-10: the oscillator is PROFIBUS address 3 (MGate slave 3, registers 7 / 6) next to the wavemaker's 2, fault value timeout on both output modules, single-rig kill tests passed. Kill test with both rigs running passed 2026-11-10 ([COMMISSIONING.md](COMMISSIONING.md#test-1-kill-test-with-both-rigs-running-passed-2026-11-10)): the MGate's fault timer runs per slave, so one process per rig stays. Left: the gateway load test with both rigs (*Test 2* in COMMISSIONING.md), then the remedy for the gateway stalls (Test 2 failed 2026-11-11; sketch, *If Test 2 fails*), then T16 on the Pi; T11-T15 for (2)-(4) are done in the repo (2026-11-10), and check that two Modbus clients on one MGate each write only their own slave, and repeat the kill test per drive with the other one running (any Modbus access resets the MGate timer: make sure the other rig's traffic does not keep a dead rig's outputs alive).
  (2) **The refusal:** refuse only when the oscillator is `sloshing`. The wavemaker's unit has to know which oscillator runs; reading `/etc/default/oscillator` is wrong if it was edited after the start, so let the running oscillator leave a marker (e.g. `run.sh` writes the oscillator to `$RUNTIME_DIRECTORY`) or check the ROS namespace.
  (3) **The web page:** a second port for the vertical oscillator (`PORT=` in `/etc/default/oscillator`; the port check in `refuse_if_busy.sh` then checks its own port), or one page for both rigs. `windows/start_oscillator.bat` opens the right address.
  (4) **The soft stop: one button stops both rigs** (decided 2026-11-10). GPIO17 can be claimed by one process only, and today each launch starts its own `soft_stop` node publishing the relative `soft_stop` (`/wavemakers/ladertanken/soft_stop`, `/oscillators/vertical/soft_stop`). Proposed: the node leaves both launches and gets a unit of its own, `soft_stop@.service`, which both rig units pull in (`Requires=` and `After=`; `StopWhenUnneeded=yes` so it stops with the last rig). It publishes one shared topic, e.g. `/ladertanken/soft_stop`, and both controllers' `soft_stop` is remapped to it in the launches (a launch argument, so a launch by hand on the PC keeps its own node). Stopping one rig then leaves the button working for the other. Check: a press stops both, releasing lets both start again, and killing the soft stop unit stops both through `soft_stop_timeout_s`. `refuse_if_busy.sh` no longer needs GPIO17 as a reason.
  (5) **ROS:** the namespaces differ (`/wavemakers/...`, `/oscillators/...`); check that the lifecycle managers, the bridges' `<rig>_state` / `<rig>_message` topics and the web nodes have no names in common.

- [ ] **48. Wavemaker start fails with F2074 after the flap drifted.** (rig; IndraWorks, `drivers/mgate5101_driver/src/indradrive.cpp`)
  2026-11-11 08:49: the first start after a stop failed in `enable()` with `drive error F2074` (IndraWorks: actual position outside the absolute encoder window); cleared in IndraWorks, the next start worked, with the flap at +0.103 m from upright. Likely cause, not confirmed: a service stop leaves the drive in parameter mode (`0x0888`), the motor has no brake (`-UG0`), so the water moves the flap, and switching back to operating mode checks the position against the stored one. F2074 also in `journalctl -b` in October. Check P-0-0097 (the window) in IndraWorks; decide between a larger window, letting the tank calm before a start, or the controller clearing F2074 (`clear_errors()` exists, nothing calls it), which needs care: it is a position-plausibility error.

## Low

- [ ] **29. `IndraDrive::disable()` can block a lifecycle callback for up to 20 s.** (`drivers/mgate5101_driver/src/indradrive.cpp`)
  Accept, or use a shorter timeout for `disable()`.

- [ ] **30. `travel_minimum_m` / `travel_maximum_m` are declared only for linear drives at construction.** (`controller_node.cpp`)
  Always declare both. `IndraDriveActuator` now declares them whenever missing (2026-11-09); the controller still declares them only when `drive_motion` is linear.

- [ ] **31. Bridge interface details.** (`bridge_node.cpp`)
  `height`, `period` and `stop` use depth 10 instead of `command_qos`; the `stop` topic and `stop` service share a name (consider renaming the service, e.g. `stop_wave`).

- [ ] **32. Clean-up.**
  Remove the unused `driver_address_` member and the commented-out members at the end of `ControllerNode`; set the license in `rig_controller/package.xml`. (Formatting and CMake lint fixed 2026-10-07.)

- [ ] **36. Soft stop timeout logged once at startup.** (`controller_node.cpp`, `soft_stop.py`; rig)
  On the Pi, about 1 s after `Managed nodes are active`, the controller logs `No Soft stop message received for over 200 ms; treating as stopped` once (2026-10-16), then works normally. Most likely load while all nodes start; harmless before the first wave, but it is logged as an error on every normal start. If it ever appears mid-session, `soft_stop_timeout_s` (0.2 s) is too tight for the Pi. Decide: raise the timeout, or skip the check until the soft stop has published steadily for a moment. 2026-10-22, on the PC with the LinMot actuator: logged 13 ms after an activation that blocked for about 0.8 s, with `ros2 topic pub` publishing at 20 Hz throughout. Both times it follows a blocking `on_activate`, so the soft stop subscription is probably not served during activation (the liveness check runs in `fault_timer_`, in the default callback group with the lifecycle services); check that before changing the timeout. 2026-10-23, LinMot: logged 2 ms after an activation that blocked for 21 s while homing, `ros2 topic pub` at 20 Hz throughout; goals worked right after. The blocking length makes no difference, which fits the callback-group explanation, not the timeout. Confirmed 2026-10-23 on the Pi with `oscillator.launch.py`: `soft_stop` published at 13:14:40.70 (starting stopped) and 13:14:47.77 (released), but the controller logged `Soft stop connected` only at 13:15:01.60, when the 21 s homing activation ended, together with `has not published within 5 s of configure`. The controller does not serve the soft stop subscription while `on_activate` blocks, although it already has its own callback group (`soft_stop_callback_group_`) and `main()` uses a `MultiThreadedExecutor`. Why is still open; a lead: the group and subscription are created in `on_configure`, after `spin()` started. Fix the cause (or do not block `on_activate`), not the timeout. 2026-10-26: also logged while the controller was `inactive` after a failed activation (the drive did not answer on CAN), minutes after the fault; harmless (goals are refused), probably the same cause. 2026-11-05 at 09:32:07, mid-session on ladertanken with no motion running: logged once, then 5 s later `soft_stop` logged `Soft stop triggered` and was released 4.5 s after that. It is not yet known whether anyone pressed the button.

- [ ] **45. A drive fault logs a burst of lifecycle errors.** (`controller_node.cpp`, the lifecycle manager in `wavemakers.launch.py` / `oscillator.launch.py`; rig)
  Seen 2026-11-06 on ladertanken (drive power off during a return, F2026; `rpi/README.md`, *Rig test* → *Controller and driver fixes*). The controller handles the fault itself: `handle_driver_fault()` deactivates, `on_deactivate` returns `ERROR`, and `on_error` releases everything and leaves it `unconfigured`. `stop_activity()` breaks the bond on the way, so the lifecycle manager logs `CRITICAL FAILURE: SERVER controller IS DOWN after not receiving a heartbeat`, resets all nodes, and its deactivate and cleanup of the controller fail with `No transition matching 4 (2) found for current state unconfigured` and `Failed to change state for node: controller`. Harmless (the drive is already released, the bridge is reset, a service restart recovers), but one fault gives about ten `ERROR` lines, and the real cause (`Driver fault: drive error F2026`) is easy to miss among them.
  Options: (a) on a driver fault, do not transition the controller; only halt and disable the drive, break the bond, and let the manager's reset do deactivate and cleanup (one path for every failure, but the controller then depends on the manager to release the drive); (b) keep today's path and document the burst in `rpi/README.md`, *Troubleshooting*, with "look for `Driver fault:` first"; (c) both: keep `on_error` as the safety net if the manager is gone. Check first whether the manager's `attempt_respawn_reconnection: False` affects which path it takes. The unit test `DriverFaultReleasesResourcesAndAllowsReconfigure` covers today's path.

- [ ] **49. A wave stopped by itself.** (rig; `bridge_node.cpp`, `rig_web`)
  2026-11-11 08:41:33, during the Test 2 baseline: `Received request to cancel goal` and `bridge: Stop: 1 goal(s) canceling` with nobody pressing Stop. The reason only reached the page then; since commit `c4c1a4a` the bridge logs `Stop requested: <reason>` (an operator Stop, a lost client heartbeat, the stop service). If it happens again: `journalctl -u wavemaker@ladertanken -o cat --no-pager | grep "Stop requested"`.

## Planned features (all rigs)

Decided but not started. Each feature says what and why, the decisions taken, and the steps.

### Go to position

An action that moves a rig to a position with a given maximum velocity and acceleration, for
CLI users. First wanted for the LinMot sloshing oscillator and a future carriage; available on
every rig (for the wavemakers it is not needed, but costs nothing).

#### Decisions (2026-10-27)

- **For CLI users.** LabVIEW is no longer a target client, so the bridge gets nothing new. The
  existing web page stays as simple as it is; a separate page may come later and can use the
  same action.
- **An action, not a service.** Ctrl-C on `ros2 action send_goal` cancels the goal, and the
  controller halts the drive; Ctrl-C on `ros2 service call` only stops waiting while the move
  goes on. The action also gives feedback during long moves (a carriage), reports a rejection at
  once, and avoids the blocking service thread that `return_to_upright` needs.
- **The drive makes the profile where it can, the controller streams one otherwise.** A drive
  that can do a positioning move with position, velocity and acceleration gets one command and
  runs it itself (on the LinMot: smooth, and no 50 Hz stepping, so none of the hum found in
  OSCILLATOR.md, step G). Other drives get a trapezoidal profile streamed at the control rate,
  as waves are.
- **On all rigs.** No parameter to switch it off.
- **Absolute position** in metres in the rig's own frame, as `upright_position_m` and the travel
  limits.

#### Behaviour

```bash
ros2 action send_goal /oscillators/sloshing/go_to_position rig_interfaces/action/GoToPosition \
  "{position: 0.15, max_velocity: 0.1, max_acceleration: 0.5}" --feedback
```

Expected: `Goal accepted`, feedback lines with the measured position and the remaining distance,
then a result with `status` SUCCESS and `final_position` within the tolerance of 0.15. Ctrl-C
before the end prints `Canceling goal...`; the slide stops and holds.

- **Rejected at once**, with the reason in the result's `message`, for:
  - a position outside the travel (`travel_minimum_m` / `travel_maximum_m`, or the limits from
    `min_position_deg` / `max_position_deg` for an angular drive);
  - `max_velocity` or `max_acceleration` ≤ 0;
  - `max_velocity` above the drive's maximum (`Actuator::max_velocity_mps()`);
  - `max_acceleration` above `max_goal_acceleration_mps2` (when set) or the drive's maximum;
  - the soft stop engaged, a fault, or another goal or a return running.
- **During the move:** cancel, the soft stop or a fault halts the drive; the result says which.
  Feedback: `position` (measured), `remaining` (distance to the target).
- **Done** when the position is within `tolerance` of the target and the motion has ended
  (native: the drive's own status; streamed: the profile has finished). Timeout: the profile's
  duration plus `return_to_upright_timeout_margin_s`.
- `tolerance` 0 means `return_to_upright_default_tolerance_m`.
- Mutual exclusion with `move_<rig>` and `return_to_upright` uses the controller's existing
  busy logic.

#### Interface

```
# rig_interfaces/action/GoToPosition.action
float64 position          # m, absolute in the rig's frame
float64 max_velocity      # m/s, > 0
float64 max_acceleration  # m/s^2, > 0
float64 tolerance         # m; 0 = return_to_upright_default_tolerance_m
---
uint8 SUCCESS=0
uint8 CANCELED=1
uint8 REJECTED=2
uint8 FAULT=3
uint8 STOPPED=4           # soft stop
uint8 TIMEOUT=5
uint8 status
float64 final_position    # m, measured
string message
---
float64 position          # m, measured
float64 remaining         # m, to the target
```

#### Native moves in the actuator

A new optional capability on `Actuator` (`rig_controller/src/actuator.hpp`); drives without it
keep the defaults and get the streamed profile:

```cpp
// A single positioning move run by the drive's own profile generator. Only drives that honour
// position, velocity and acceleration together return true; the others get a streamed profile.
virtual bool supports_native_move() const {return false;}
// Starts the move; false if the drive refused it. The target is in actuator units
// (to_actuator_setpoint()).
virtual bool move_absolute(const ActuatorSetpoint & target) {(void)target; return false;}
// True once the drive reports the move finished (in target, no motion active).
virtual bool native_move_done() const {return false;}
```

| Drive | Path | Why |
| --- | --- | --- |
| LinMot (`LinMotActuator`) | native | VAI is exactly position + velocity + acceleration; done = status bits `kInTargetPosition` set and `kMotionActive` clear (`linmot/linmot_drive.h`). |
| IndraDrive (`IndraDriveActuator`) | streamed | Positions with a velocity limit, but takes no acceleration. |
| Future carriage | depends on its drive | Behind the same interface. |

To check first, on the LinMot: the bridge keeps sending the latest target every cycle. The
command-count firmware (OSCILLATOR.md, 2026-10-22) should make the drive treat a repeated
command with the same count as the same command, so one `move_absolute()` is one move and is not
restarted every 20 ms. Confirm with the simulator and on the rig.

#### Streamed profile

A trapezoid: accelerate at `max_acceleration` to `max_velocity`, cruise, brake at
`max_acceleration`; a triangle when the distance is too short to reach `max_velocity`. A pure
function next to `wave_math` (`rig_controller/src/`), sampled at the control rate and written
through `publish_and_write_setpoint()` like waves and returns, with the velocity and
acceleration caps from the goal.

#### Steps

- [ ] **1. Interface:** `GoToPosition.action` in `rig_interfaces` (`CMakeLists.txt`).
- [ ] **2. Trapezoidal profile** with unit tests: duration, peak velocity and acceleration, the
  triangle case, ends exactly at the target, zero distance.
- [ ] **3. Actuator:** the three methods in `actuator.hpp`; the LinMot implementation, tested in
  `test_linmot_actuator.cpp` against the simulated bridge (one command is one move, done is
  reported, halt during a move); the IndraDrive keeps the defaults.
- [ ] **4. Controller:** the action server `go_to_position` in `controller_node.cpp`, native and
  streamed paths, sharing the busy logic, soft stop and fault handling; tests in
  `test_controller_node.cpp` with `FakeActuator` for both paths, cancel, soft stop, fault and
  every rejection.
- [ ] **5. Docs:** a short CLI section in README.md.
- [ ] **6. Rig test, oscillator:** moves of a few sizes and speeds; Ctrl-C mid-move; the soft
  stop mid-move; a rejection (outside the travel). Listen: a native move should be quiet. Then
  one streamed move on the ladertanken wavemaker.

Unit tests (no hardware):

```bash
cd ~/ros_ws
colcon build --packages-up-to rig_controller
colcon test --packages-select rig_controller linmot_serial_driver
colcon test-result --verbose
```

Expected: `Summary: N tests, 0 errors, 0 failures`.

#### Open

- Whether `return_to_upright` should become a go-to-position to the upright with default
  limits (the service stays for compatibility). Decide after step 4.

### Rig-agnostic configuration

Make adding a rig (a future carriage, or others) a matter of one yaml file, plus an `Actuator`
class only when the rig uses a new drive family. The drive is already chosen by config
(`drive_name`); the rig name and the way a goal becomes a trajectory are not.

**When:** after `feat/oscillator` is merged into `master`, which waits until the current lab use
ends (end of the week of 2026-10-26). On its own branch.

#### Decisions (2026-10-28)

- Three separate settings, none inferred from another:
  - **drive** (`drive_name`): how setpoints reach the hardware. Already done.
  - **rig** (`rig`): a name only. It gives the action and topic names (`move_<rig>`,
    `<rig>_setpoint`, ...) and nothing else.
  - **motion model** (`motion_model`): how a goal becomes a trajectory.
- No list of allowed rig names anywhere in the code.
- Rig-neutral renames of `upright`, `return_to_upright`, `wavemaker_transfer_gain` and
  `wavemaker_position_offset_` are left out for now: they break configs, the service and the web
  page. Revisit if a third rig makes the names confusing.

#### Today

The rig name is checked against `wavemaker` / `oscillator` in `controller_node.cpp` (`on_configure`),
`bridge_node.cpp`, `rig_web/launch/web.launch.py`, `rig_web/launch/demo.launch.py`, `RIG_TEXTS` in
`rig_web/scripts/web_server.py` and the `case` in `rpi/run.sh` / `rpi/stop.sh`. The motion is
picked from `rig_ == "wavemaker"` plus `wavemaker_type` in `controller_node.cpp` (configure
checks, the gain table, `trajectory_.configure()`) and `geometry.type == "oscillator"` in
`wave_trajectory.cpp`.

#### Steps

- [ ] **1. Motion model.** Replace `wavemaker_type` with `motion_model`: `piston`, `flap` or
  `stroke` (H is the peak-to-peak travel, no wave theory; today's oscillator). Add
  `{"wavemaker_type", "motion_model"}` to the rename table so old configs fail with a clear
  message. Each model's parameter checks (depth, hinge, attachment height, gain table) run only
  for the models that use them; the gain table and `water_depth` only for `piston` and `flap`.
  `WavemakerGeometry::type` takes the same three values. Update `wavemakers.yaml` and
  `oscillator.yaml`. Tests: `test_wave_trajectory.cpp` (stroke ignores depth and gain),
  `test_controller_node.cpp` (each model's rejections; an unknown model fails configure; the old
  name is rejected).
- [ ] **2. Free rig name.** `rig` must be a valid ROS name token (letters, digits, `_`, not
  starting with a digit); drop the enum checks in the controller and the bridge. Move the web
  texts into the rig's yaml under the web server (`machine`, `motion`, `a_motion`, `title`,
  `height_label`, ...), with generic defaults ("rig", "motion") so a new rig works without
  touching `web_server.py`. `web.launch.py` and `demo.launch.py` take the namespace directly
  instead of deriving it from the rig. Tests: a controller and bridge configured with
  `rig: carriage` publish `carriage_setpoint` and serve `move_carriage`.
- [ ] **3. One launch file.** A generic `rig.launch.py` (controller, bridge, lifecycle manager
  with `attempt_respawn_reconnection: False`, soft stop, web) taking `config` (yaml path) and
  `namespace`. `lab.launch.py` and `oscillator.launch.py` become thin wrappers, or go. On the
  Pi, `run.sh` / `stop.sh` read `LAUNCH_CONFIG` and `NAMESPACE` from the unit's
  `/etc/default/<unit>` file instead of a `case` per rig; the units stay one per rig, so
  `refuse_if_busy.sh` and the sudoers lines do not change.
- [ ] **4. Docs:** a short "Adding a rig" section in README.md: the yaml keys every rig needs,
  the three settings above, and when a new `Actuator` is needed.
- [ ] **5. Rig test** on the ladertanken Pi, both rigs through the services: start, a wave from
  the page, `systemctl stop`, and the mutual exclusion (`rpi/README.md`, *Rig test* and
  *Oscillator → Rig test*).

Later, with [Go to position](#go-to-position): oscillate around the position the rig is at when
the goal arrives, not only around `upright_position_m` (a carriage moves somewhere first, then
oscillates). Today a goal must start from upright.

Unit tests (no hardware):

```bash
cd ~/ros_ws
colcon build --packages-up-to rig_controller rig_web
colcon test --packages-select rig_controller
colcon test-result --verbose
```

Expected: `Summary: N tests, 0 errors, 0 failures`.

### Oscillator motion generator

`generate_oscillations`: motion files for the oscillators (`sloshing`, `vertical`), built like
the wavemakers' `rig_controller/src/generate_waves.py`. Files must play with
`play_dat --rig oscillator` and be accepted by the controller. `play_dat` takes the oscillator's
limits, upright and namespace from `oscillator.yaml` (`play_dat <file> vertical --rig oscillator`).

**Status (2026-11-10): done.** `rig_controller/src/generate_oscillations.py`, installed as
`ros2 run rig_controller generate_oscillations`, with the shared code in `motion_files.py`, tests in
`test_generate_oscillations.py` and the README section *Generating oscillator motion files*.
Played on the rig 2026-11-10 (ladertanken, vertical, no load): three runs of 10 mm (regular 2 s and 1 s,
irregular Tp 1.5 s for 60 s) from the PC with `play_dat`; the controller's setpoints equal the file
within 0.003 mm, the slide reached 97 % / 90 % / 93 % (4σ) with 160-190 ms lag, as item 46.

#### Decided (2026-11-09)

- A test matrix as the wavemaker's: one row per run, regular (`period_s`, `stroke_m`,
  `n_periods`) and irregular (`sig_stroke_m`, 4σ of the position, `tp_s`, `gamma`, `duration_s`,
  `seed`), `n_ramp` and `pause_s`.
- The same four files (`.dat`, run table, record, plot), from `motion_files`.
- A cosine ramp (no acceleration step where the ramp ends); the peak jerk in the run table and a
  warning above the vertical oscillator's 3 m/s³ (S-0-0193).

#### Decided (2026-11-03)

- Its own script, not a branch in `generate_waves.py`: the input is a stroke, not a wave height,
  so the test matrices differ.
- Same file contract as the wavemakers: `play_dat`'s `.dat` layout and run table, checked by
  reading the file back with `play_dat`'s own functions.

#### Differences from the wavemaker generator

- Config `oscillator_bringup/config/oscillator.yaml`, key `/oscillators/<name>/controller`.
- No wave physics: amplitude = H / 2 (H is the stroke; `WaveTrajectory::configure`, type
  `oscillator`). No transfer gain.
- Limits from `rig_limits`: sloshing within 0.015..0.185 m around upright 0.1 m, 2.0 m/s (the
  LinMot's default), acceleration the lower of `max_goal_acceleration_mps2` (9.81) and the drive's
  20; vertical within ±0.2 m around 0, 0.1 m/s (`max_velocity_rpm` 600 in 10 mm/min), no
  acceleration limit.

### Trigger output

A hardware signal for external equipment (data acquisition, cameras) that follows the runs of a
motion file. The generators already write a `trigger` channel (1 during each run, 0 at rest, as
`make_test_dat.py`); nothing reads it: `play_dat` sends only positions, and the controller and
the Pi have no trigger output.

**When:** not started. First find out what the lab's acquisition needs.

#### Open

- **Who uses it, and what does it expect?** A level (high while recording) or an edge (a pulse
  at start); voltage and connector.
- **When is it on?** During the run (today's channel); from when the waves reach the model until
  they have passed (needs the paddle-to-model distance and the group velocity; irregular runs need
  a rule, e.g. at Tp); or a pulse at the start of each run.
- **How does it leave the Pi, in step with the paddle?**
  - The controller switches a GPIO output at goal start and end, or plays a trigger array with
    the positions: accurate to the control cycle; changes the action, the controller and wiring.
  - `play_dat` switches a GPIO output when a goal is accepted and finished: simpler, accurate to
    tens of milliseconds.
  - No hardware trigger: record continuously and align afterwards from the `<rig>_setpoint` and
    position topics or the run table's times.

Until then the generator keeps writing 1 during each run; changing the channel has no effect on
the rig.
