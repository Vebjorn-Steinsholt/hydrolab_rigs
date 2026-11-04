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

- [ ] **38. Rig test of `generate_waves`. (rig)** (`rig_controller/src/generate_waves.py`, README *Generating motion files*)
  Unit tests and `play_dat --dry-run` pass (2026-11-03); not yet played on the rig. After item 37, on ladertanken.
  1. Pi on the branch with the generator (`git pull`, `colcon build`), the same commit as the PC that generates. Measure the water depth; `WATER_DEPTH` in `/etc/default/wavemaker` set to it, service restarted.
  2. On the PC, a small test matrix: two small regular runs (1.0 s 1/60, 1.5 s 1/40), one near the limits (1.3 s, H 0.171 m), one irregular run (Hs 0.03 m, Tp 1.2 s, 120 s, seed 1), 60 s pauses. Generate with `--water-depth` equal to the Pi's, check the `.png`, copy the `.dat` and `.csv` together to the Pi.
  3. On the Pi, in `tmux`: `play_dat --dry-run` with the printed limits, then play run by run (`--runs 0`, ...) with the service running and the page open. Stop on the page during one run, and the soft stop during another: both must stop the paddle.
  4. Pass: every run accepted, no faults; the page's plot of each run matches the `.png`; measured H of the regular runs within the gain table's scatter (~6 %) of the target at calibrated periods; measured Hs of the irregular run noted (no pass limit yet).
## Medium

- [ ] **16. Test and document the bridge.** (`rig_controller/test/`, `README.md`)
  - A gtest with a fake `move_wavemaker` server: start and refused start, stop (topic and service), stop during start, lost heartbeat, controller rejection and abort, deactivation (`inactive`), and a stop the controller never confirms (no result within 2 s, or the action unavailable): the bridge must leave `stopping` and publish idle with a warning. `test/fake_controller.py` shows the fake's behaviour.
  - Document the bridge interface in the README: topics, types, QoS, the start sequence (height, period, then `start`), heartbeat, `wavemaker_state` values.

- [ ] **19. Linear drives (lilletanken, mclab).** Deferred: neither is deployed yet (they need gateways). (`rig_controller/src/indradrive_actuator.cpp`)
  The drive's position tag (scaled as degrees) is used as metres unchanged, and velocity is converted with `/ 6` as if it were deg/s → rpm; the limits exist twice (`travel_minimum_m/maximum` and `min_position_m/max_position_m`). Once the drive scaling is confirmed on the rig (item 19 in [COMMISSIONING.md](COMMISSIONING.md)), add a proper linear conversion and derive the drive limits from the node limits. The unit conversion now goes through `DriveScale` (`rig_controller/src/drive_scale.hpp`, 2026-10-23); the duplicate limits remain.

- [ ] **20. Data race on `IndraDrive::last_error_`.** (`drivers/mgate5101_driver/src/indradrive.cpp`)
  Written by `fail()` from the worker thread, read by `fault_reason()` / `last_error()` from executor threads. Guard it with a mutex and return a copy.

- [ ] **21. Unit-test the hold behaviour of `IndraDriveActuator::halt()`.**
  Check that `halt()` writes the last commanded target (clamped) with `hold_velocity_rpm` and leaves Drive Start set, e.g. by reading the MGate driver's staged outputs.

- [ ] **22. `cancel_all_goals` can report `SERVER_UNAVAILABLE` right after configure.** (`rig_controller/src/controller_node.cpp`)
  `action_server_is_ready()` is false until discovery completes; wait up to about 1 s before replying.

- [ ] **39. `generate_waves` records how a file was made.** (`rig_controller/src/generate_waves.py`)
  A file is only right for the depth, yaml and generator it was made with, and a pregenerated goal carries positions only, so neither `play_dat` nor the controller can tell. Write `<output>.txt` next to the `.dat`: wavemaker, water depth (and whether it was overridden), dt, the git commit of the repository (with `-dirty` if it has local changes), the date and the command line. Needed to interpret probe data and to regenerate a run.

- [ ] **40. `play_dat` takes its limits from `wavemakers.yaml`.** (`rig_controller/src/play_dat.py`)
  `--min-m`, `--max-m` and `--max-velocity-mps` are typed by hand, separately from the limits `generate_waves` used; a typo gives a misleading dry-run OK (the controller still enforces its own limits). Give `play_dat` the wavemaker's name and read the limits as `generate_waves.load_wavemaker` does (share that code rather than copy it); keep the three options as overrides. The oscillator needs the same from `oscillator.yaml`.

## Low

- [ ] **24. Return-to-upright replies with the wrong reason in two cases.** (`controller_node.cpp`)
  An exception gives `ACTUATOR_FAULT` with an empty message (use the exception text); a fault that deactivates the node first gives `CONTROLLER_INACTIVE` (check `actuator_->faulted()` in the `Stopped` case). Also (2026-10-27): a soft stop during a return ends as `Stopped` too and is reported as `CONTROLLER_INACTIVE`, "Interrupted by lifecycle transition"; give the soft stop its own status or message.

- [ ] **25. The start blend always lasts one wave period.** (`rig_controller/src/wave_trajectory.cpp`)
  Far from the first wave position with a short period, the start is too fast and the goal is rejected. Use enough whole periods to stay under `return_to_upright_max_velocity_mps`.

- [x] **26. Goal type is chosen by a parameter.** (`controller_node.cpp`, `play_dat.py`)
  `mode_pregenerated` fixes at launch whether the controller takes regular waves or position lists, so playing a `.dat` file needs a relaunch, and a regular wave sent in pregenerated mode is rejected with "goal requires positions", which does not name the mode. Choose per goal instead (non-empty `positions` → pregenerated), remove `mode_pregenerated` (add it to the rename table as removed) and the relaunch from `play_dat.py`'s usage text, the launch files and `fake_controller.py --pregenerated`.
  Decided 2026-10-29: the web page and the bridge start regular waves only; pregenerated motion is for `play_dat.py` (and other CLI clients). The bridge needs no change.
  Done 2026-10-30: each goal picks its type (non-empty `positions` → pregenerated; positions together with height/period, or no sample_interval, are rejected); `mode_pregenerated` and `wavemaker_mode_pregenerated` are in the rename table as removed, so a leftover setting fails configure. Removed from `wavemakers.yaml`, `wavemakers.launch.py`, `bridge.launch.py`; `fake_controller.py` lost `--pregenerated`; `MoveRig.action`, README, OSCILLATOR.md and `play_dat.py` usage updated. `play_dat` now runs under `lab.launch.py` with the page open: the plot, Stop on the page and the soft stop act on its goals, and the web server's idle auto-stop counts every controller setpoint as motion. Tests: new `EachGoalPicksItsType`, `GoalWithPositionsAndWaveIsRejected` and removed-name cases in `test_controller_node`; all gtests pass; checked against `fake_controller` (play_dat, a regular goal and a both-types goal on one node) and the idle timer (`idle_timeout_s:=12`, 20 s run: shutdown 12 s after the run, not during). Rig test passed on ladertanken 2026-11-02 (commit `e096801`): checklist and results in `rpi/README.md`, *Rig test* → *Goal type per goal and `play_dat`*; next: merge `feat/oscillator` into `master`.

- [ ] **27. A regular wave from the terminal runs until it is cancelled.**
  In bridge mode the heartbeat covers a lost client; from the terminal nothing does. Consider an optional `duration` in `MoveRig.action`.

- [ ] **29. `IndraDrive::disable()` can block a lifecycle callback for up to 20 s.** (`drivers/mgate5101_driver/src/indradrive.cpp`)
  Accept, or use a shorter timeout for `disable()`.

- [ ] **30. `travel_minimum_m` / `travel_maximum_m` are declared only for linear drives at construction.** (`controller_node.cpp`)
  Always declare both.

- [ ] **31. Bridge interface details.** (`bridge_node.cpp`)
  `height`, `period` and `stop` use depth 10 instead of `command_qos`; the `stop` topic and `stop` service share a name (consider renaming the service, e.g. `stop_wave`).

- [ ] **32. Clean-up.**
  Remove the unused `driver_address_` member and the commented-out members at the end of `ControllerNode`; set the license in `rig_controller/package.xml`. (Formatting and CMake lint fixed 2026-10-07.)

- [ ] **36. Soft stop timeout logged once at startup.** (`controller_node.cpp`, `soft_stop.py`; rig)
  On the Pi, about 1 s after `Managed nodes are active`, the controller logs `No Soft stop message received for over 200 ms; treating as stopped` once (2026-10-16), then works normally. Most likely load while all nodes start; harmless before the first wave, but it is logged as an error on every normal start. If it ever appears mid-session, `soft_stop_timeout_s` (0.2 s) is too tight for the Pi. Decide: raise the timeout, or skip the check until the soft stop has published steadily for a moment. 2026-10-22, on the PC with the LinMot actuator: logged 13 ms after an activation that blocked for about 0.8 s, with `ros2 topic pub` publishing at 20 Hz throughout. Both times it follows a blocking `on_activate`, so the soft stop subscription is probably not served during activation (the liveness check runs in `fault_timer_`, in the default callback group with the lifecycle services); check that before changing the timeout. 2026-10-23, LinMot: logged 2 ms after an activation that blocked for 21 s while homing, `ros2 topic pub` at 20 Hz throughout; goals worked right after. The blocking length makes no difference, which fits the callback-group explanation, not the timeout. Confirmed 2026-10-23 on the Pi with `oscillator.launch.py`: `soft_stop` published at 13:14:40.70 (starting stopped) and 13:14:47.77 (released), but the controller logged `Soft stop connected` only at 13:15:01.60, when the 21 s homing activation ended, together with `has not published within 5 s of configure`. The controller does not serve the soft stop subscription while `on_activate` blocks, although it already has its own callback group (`soft_stop_callback_group_`) and `main()` uses a `MultiThreadedExecutor`. Why is still open; a lead: the group and subscription are created in `on_configure`, after `spin()` started. Fix the cause (or do not block `on_activate`), not the timeout. 2026-10-26: also logged while the controller was `inactive` after a failed activation (the drive did not answer on CAN), minutes after the fault; harmless (goals are refused), probably the same cause.

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

A generator of motion files for the sloshing oscillator, separate from the wavemakers'
`rig_controller/src/generate_waves.py`, which reads only `wavemakers.yaml` and refuses other rigs.
Files must play with `play_dat --rig oscillator` and be accepted by the controller.

**When:** after the wavemaker generator works. Not started.

#### Decided (2026-11-03)

- Its own script, not a branch in `generate_waves.py`: the input is a stroke, not a wave height,
  so the test matrices differ.
- Same file contract as the wavemakers: `play_dat`'s `.dat` layout and run table, checked by
  reading the file back with `play_dat`'s own functions.

#### Differences from the wavemaker generator

- Config `oscillator_bringup/config/oscillator.yaml`, key `/oscillators/<name>/controller`.
- No wave physics: amplitude = H / 2 (H is the stroke; `WaveTrajectory::configure`, type
  `oscillator`). No transfer gain.
- Limits: positions absolute within `travel_minimum_m`..`travel_maximum_m` (0.015..0.185 m,
  upright 0.1 m); speed the LinMot's `max_velocity_mps` (default 2.0, not in the yaml);
  acceleration the lower of `max_goal_acceleration_mps2` (9.81) and the drive's
  `max_acceleration_mps2` (20).

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
