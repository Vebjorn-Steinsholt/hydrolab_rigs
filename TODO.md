# TODO

Open software work, by priority, and planned features for all rigs at the end
([Planned features](#planned-features-all-rigs)). Physical checks on the wavemaker are in
[COMMISSIONING.md](COMMISSIONING.md); the checks before each lab session are in
[STARTUP.md](STARTUP.md).

Item numbers are shared with [COMMISSIONING.md](COMMISSIONING.md) (numbering restarted on
2026-10-06), so references such as "item 33" stay valid in both files; gaps in the numbering are
the items kept there, and finished items, which are removed (see the git history).

Designs for open items are in sketches, each linked from its items:

- [controller_fixes_sketch.md](controller_fixes_sketch.md): 29, 30, 31, 32, 36.
- [remaining_items_sketch.md](remaining_items_sketch.md): 19, 45, 49, 52, 55, 56, 58–65, with
  the order to do them in, and why 43, 46, 50, 51 and 53 have no sketch.
- [oscillator_centre_sketch.md](oscillator_centre_sketch.md): 54.
- [gain_estimation_sketch.md](gain_estimation_sketch.md): the long-term follow-up to 43
  (*Transfer function from every run*).
- [drive_correction_sketch.md](drive_correction_sketch.md): correcting one irregular run's drive
  signal from its recording, by iteration (no item yet).
- [ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md): an HC-SR04 wave probe on a ROS
  topic, through an Arduino and an RPi4, for gain model stage 1 (no item yet). Its node, in
  detail: [wave_probe_node_sketch.md](wave_probe_node_sketch.md).

**Order of the sketches:**

1. **36**, then **59** and **58** (before the next rig session).
2. The rest of [remaining_items_sketch.md](remaining_items_sketch.md) in its own *Order*, with
   the other controller_fixes sections (29, 30, 31, 32) before 61.
3. **54**'s *Check on the rig*, when the rig is free.
4. After **43**: [gain_estimation_sketch.md](gain_estimation_sketch.md) stage 0. Begin with
   [drive_correction_sketch.md](drive_correction_sketch.md) change 1 (`analyse` split into
   `measured_run`), because gain-model change 1 edits `analyse` too.
5. The rest of [drive_correction_sketch.md](drive_correction_sketch.md). It uses the gain model
   as Ĥ once stage 0 exists.
6. [ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md): the probe on a topic, and its
   check against WP1. Can start any time; it needs only the RPi4 and the bench.
7. Gain model stages 1 and 2, once the probe passes that check.

- **(rig)** needs the real wavemaker; the unit tests use fakes for the drive and the controller.
- Paths are relative to the repository root.

## High

- [ ] **36. The soft stop is not heard while `on_activate` blocks, homing included.** (`controller_node.cpp`, `soft_stop.py`; rig)
  Moved to High 2026-11-18: homing moves the drive, and on 2026-10-23 soft stop messages sent during the 21 s homing activation were received only after it ended. **Cause, found 2026-11-05:** `on_configure()` builds `options` with `soft_stop_callback_group_` but never passes it to `create_subscription`, so the subscription is in the default group, which every lifecycle transition blocks. Fix: pass `options` as the last argument (`}, options);`); keep the 200 ms timeout. Still not in the code on 2026-11-18. Sketch: [controller_fixes_sketch.md](controller_fixes_sketch.md), section 1 (the gtest that fails before the fix, and the rig checks). History:
  On the Pi, about 1 s after `Managed nodes are active`, the controller logs `No Soft stop message received for over 200 ms; treating as stopped` once (2026-10-16), then works normally. Most likely load while all nodes start; harmless before the first wave, but it is logged as an error on every normal start. If it ever appears mid-session, `soft_stop_timeout_s` (0.2 s) is too tight for the Pi. Decide: raise the timeout, or skip the check until the soft stop has published steadily for a moment. 2026-10-22, on the PC with the LinMot actuator: logged 13 ms after an activation that blocked for about 0.8 s, with `ros2 topic pub` publishing at 20 Hz throughout. Both times it follows a blocking `on_activate`, so the soft stop subscription is probably not served during activation (the liveness check runs in `fault_timer_`, in the default callback group with the lifecycle services); check that before changing the timeout. 2026-10-23, LinMot: logged 2 ms after an activation that blocked for 21 s while homing, `ros2 topic pub` at 20 Hz throughout; goals worked right after. The blocking length makes no difference, which fits the callback-group explanation, not the timeout. Confirmed 2026-10-23 on the Pi with `oscillator.launch.py`: `soft_stop` published at 13:14:40.70 (starting stopped) and 13:14:47.77 (released), but the controller logged `Soft stop connected` only at 13:15:01.60, when the 21 s homing activation ended, together with `has not published within 5 s of configure`. The controller does not serve the soft stop subscription while `on_activate` blocks, although it already has its own callback group (`soft_stop_callback_group_`) and `main()` uses a `MultiThreadedExecutor`. (Cause found 2026-11-05, see above.) 2026-10-26: also logged while the controller was `inactive` after a failed activation (the drive did not answer on CAN), minutes after the fault; harmless (goals are refused), probably the same cause. 2026-11-05 at 09:32:07, mid-session on ladertanken with no motion running: logged once, then 5 s later `soft_stop` logged `Soft stop triggered` and was released 4.5 s after that. It is not yet known whether anyone pressed the button.

- [ ] **57. `rig_web`: refuse cross-origin POSTs.** (`rig_web/scripts/web_server.py` `do_POST()`; Claude's, handed over)
  `do_POST()` checks neither `Origin` nor `Content-Type`. Any web page open in a lab browser can send a `text/plain` POST (no CORS preflight) to `/api/start` on `<pi>.local:8080` and start a wave. Reject a POST whose `Origin` is set and does not match `Host`, and require `Content-Type: application/json`. The page already sends it (`app.js` `post()`); `api/ack` must then send it too. Test in `rig_web/test/test_web_http.py`: a foreign `Origin` and a `text/plain` body are refused with 403 and start nothing. 2026-11-18: code and tests done (`_refuse_reason()`; the page's `api/ack` now sends JSON). Left: after the Pi's `git pull` and build, open the page in a browser, start and stop a wave, and check the journal shows no `No page acknowledged` gaps; then remove this item.

- [ ] **58. Run all tests automatically.** (`rig_controller/CMakeLists.txt`, `rig_web/CMakeLists.txt`, a hook or CI)
  `colcon test` runs only the gtests. The Python tests (318 on 2026-11-18) run only by hand, and there is no CI, while the Pis deploy `master` with `git pull`. Register them with `ament_add_pytest_test` and run `pytest` and `colcon test` from a pre-push hook or a GitHub Action. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **59. Record which code played each run.** (`rig_controller/src/play_dat.py`, `rpi/run.sh`)
  `generate_waves` and `transfer_function` write `source_commit()`, but `play_dat` and the controller don't, so a session can't show which code ran on the Pi (the ~49 ms late-start `play_dat` of 2026-11-17 is an example). `play_dat` writes the Pi's `git describe --dirty` into its session record, and `run.sh` logs it at startup. Decide whether `run.sh` refuses a `-dirty` tree. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **43. Confirm the gain table with probes, and the per-run timeline. (rig)** (the gain table in `wavemakers.yaml`, `transfer_function`, `play_dat`)
  The yaml has the 2026-11-13 geometry (attachment 1.39 m, lead 0.000413 m/deg) and gain table, with 0.8 s at 0.99 since 2026-11-17. The 2026-11-17 check (`~/wave_sessions/2026-11-17_check`, `check2.dat`, 1.0 m) reproduced 2026-11-13 within 2 %. It also found that catman's clock runs +255 ppm against the Pi's, and that `play_dat` started each goal ~49 ms late, adding up. Both are handled since `88363ba` (design and checks: `git show 88363ba:run_timeline_sketch.md`): `transfer_function` finds each run by itself and fits the drift, and `play_dat` sends each goal at its time in the file. On 2026-11-17 that gave late starts of 0 62 102 142 202 243 ms; irregular gain 1.019 (0.8 s) and 1.106 (1.0 s); regular 0.986 (0.8 s, a 7.2 s window only), then 1.107, 1.108, 1.126, 1.128 (1.0–1.6 s). Left: the old records without `--offset-s`, the new `play_dat` on the Pi, and a 0.8 s run long enough to trust. Pooling this session's runs with earlier ones into one gain model, later: [gain_estimation_sketch.md](gain_estimation_sketch.md).

  **2026-11-18, with the stick and the rig, in this order:**

  1. **The 2026-11-13 check without `--offset-s` (lab PC, stick in).** Rebuild first if the laptop install is older than `88363ba` (`cd ~/ros_ws && colcon build --packages-select rig_controller wavemaker_bringup`). The loop reruns each run's original command, from its `.txt`, without `--offset-s`:

     ```bash
     cd ~/wave_sessions/2026-11-13_check
     cp /media/hydrolab-linux/LADERSCHTIC/check.ASC .
     source ~/ros_ws/install/setup.bash
     for run in 0 1 2 3 4; do
       cmd=$(sed -n 's/^command      transfer_function //p' run$run.txt \
             | sed 's/ --offset-s [-0-9.]*//; s/ -o run\([0-9]\)/ -o new_run\1/; s|/media/hydrolab-linux/LADERSCHTIC/check.ASC|check.ASC|')
       eval ros2 run rig_controller transfer_function $cmd | grep -E '^(offset|runs|late|drift|error)'
     done
     for run in 0 1 2 3 4; do
       python3 -c "import csv; a, b = (next(csv.DictReader(open(f))) for f in ('run$run.csv', 'new_run$run.csv')); print($run, a['period_s'], f\"{float(b['gain']) / float(a['gain']) - 1:+.2%}\")"
     done
     ```

     Pass:
     - every run analyses;
     - `offset` about 16.0–16.3 s;
     - `late` rises ~50 ms a run (the old `play_dat`);
     - `drift` is *not measured* (regular runs only, one piece each);
     - each gain is within 0.5 % of before.

     The WP1 scaling is wrong in these records (×1.74), but that cancels in the comparison. If run 0 is refused *upside down*, that's step 7 below: note it and go on. If item38's `.dat` and `.ASC` are on the stick, also run README's item38 run 3 command and compare with `item38_run3_tf.csv` the same way.

  2. **The Pi: the new `play_dat` and its clock.**

     ```bash
     ssh ladertanken@ladertanken-rpi.local
     cd ~/ros_ws/src/hydrolab_rigs && git pull && git log --oneline -1    # 88363ba or later
     cd ~/ros_ws && colcon build --packages-select rig_controller wavemaker_bringup
     sudo systemctl restart wavemaker@ladertanken
     chronyc tracking; timedatectl
     ```

     From `chronyc tracking`, note `Leap status` (synced or not), `Frequency` (the correction in ppm) and `System time`. On the catman PC, run `w32tm /query /status` in a command prompt. Together they say which clock the +255 ppm belongs to: if the Pi is synced and corrects by much less than 255 ppm, it's catman's.

  3. **WP1 in catman.** Check the channel's scaling reads the sensor database's 0.0229 m/V. Zero it in still water, then move the probe -5, 0, +5 cm on its stand: it must read -50, 0, +50 mm (±1 mm). Check `WM_Position` is recorded. Measure the depth with a ruler. For 1.0 m, `WATER_DEPTH` in `/etc/default/wavemaker` stays empty; any other depth goes there (README, *Water depth*).

  4. **The session file, on the Pi.** As `check2`, but with the 0.8 s run at 30 periods (a 19 s window, not 7 s):

     ```bash
     source ~/ros_ws/install/setup.bash && export ROS_DOMAIN_ID=1
     mkdir -p ~/wave_sessions/2026-11-18_check && cd ~/wave_sessions/2026-11-18_check
     cat > matrix.csv <<'END'
     kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
     irregular,,,,,0.03,1.2,3.3,600,1,5,60
     regular,0.8,,1/60,30,,,,,,3,60
     regular,1.0,,1/60,15,,,,,,3,60
     regular,1.1,,1/60,15,,,,,,3,60
     regular,1.3,,1/60,15,,,,,,3,60
     regular,1.6,,1/60,15,,,,,,3,60
     END
     ros2 run rig_controller generate_waves ladertanken matrix.csv -o check3
     ros2 run rig_controller play_dat check3.dat ladertanken --dry-run
     ```

     Pass: the dry run lists runs 0-5 without errors; the file is about 17 min long.

  5. **Record and play.** Start catman (WP1, `WM_Position`) with at least 30 s of still water before the file. In a second Pi terminal, start the bag:

     ```bash
     source ~/ros_ws/install/setup.bash && export ROS_DOMAIN_ID=1
     cd ~/wave_sessions/2026-11-18_check
     ros2 bag record -o check3_bag /wavemakers/ladertanken/wavemaker_setpoint /wavemakers/ladertanken/wavemaker_position
     ```

     Then, in the first terminal, `ros2 run rig_controller play_dat check3.dat ladertanken`. No `late on the file` warning should show. Stop catman about 30 s after the last run, then the bag (Ctrl-C). Export the record as catman ASCII to the stick as `check3.ASC`.

  6. **Analyse, on the lab PC.**

     ```bash
     mkdir -p ~/wave_sessions/2026-11-18_check && cd ~/wave_sessions/2026-11-18_check
     scp -r ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-18_check/{matrix.csv,check3.dat,check3.csv,check3.txt,check3_bag} .
     cp /media/hydrolab-linux/LADERSCHTIC/check3.ASC .
     source ~/ros_ws/install/setup.bash
     for run in 0 1 2 3 4 5; do
       ros2 run rig_controller transfer_function check3.dat ladertanken --run $run \
           --record check3.ASC --probe WP1 --probe-distance-m 3.0 \
           --stroke WM_Position --stroke-scale -1.342 --stroke-height-m 1.375 \
           --depth-m 1.0 -o run$run --plot
     done
     ```

     Use the measured depth, and `--probe-scale 0.001` if WP1 is exported in mm. Pass:
     - **`late` within ±20 ms of 0** for every run (2026-11-17: up to 243 ms); what's left is the drive's lag changing with period. If it still rises ~50 ms a run, the Pi didn't get the new `play_dat` (step 2).
     - **`drift`** near +255 ppm, with rms below a few ms. A clearly different value means one clock changes between boots. That's fine for `transfer_function`, which fits it every time, but note it with step 2's numbers.
     - **Gains:**
       - run 0 within 0.5 % of 2026-11-17's run 0 (1.106 at 1.0 s);
       - runs 2–5 within 1 % of 2026-11-17 (1.107, 1.108, 1.126, 1.128);
       - `drive` 0.98–1.00 and `chain` a few %.
     - **0.8 s (run 1):** H/target = `gain` / 0.99. If the 19 s window and run 0 agree on a value more than 2 % from 0.99, update the yaml:
       1. Set the 0.8 s entry (the second value of `wavemaker_transfer_gain_values`) to that value.
       2. Run `python3 -m pytest -q rig_controller/test/`, then commit.
       3. On the Pi: `git pull`, build `wavemaker_bringup` and restart the service.

     Note the results and the date in COMMISSIONING.md, *Actuator conversion calibration*: the gains, the drift, the late starts and step 2's clock numbers.

  7. (Claude, if wanted) `anchor_from_stroke` with a regular first run: the sign check refused `check.ASC` (negative peak -0.882 against +0.868). Refuse only when the negative peak is clearly larger (> 1.2 ×). Not needed for a file that starts with an irregular run.

  Later, if a long session shows it: the drift per run instead of per file (the pieces' rms would grow first), and the drive's lag by period from the `runs` line once the late starts are gone.

  Still open from October and 2026-11-05: long runs at 1.5 s and above see the beach reflection after about 21 s; measure before it arrives, or separate incident and reflected waves (3 probes, *Transfer function from every run*).

## Medium

- [ ] **19. Linear drives (lilletanken, mclab): set their units.** Deferred: neither is deployed yet (they need gateways). (`wavemaker_bringup/config/wavemakers.yaml`)
  Lilletanken and mclab still carry placeholders: `drive_m_per_unit` 1.0, `drive_velocity_per_unit_per_s` 1/6, `drive_direction` 1 and `max_velocity_rpm: 100` which `rig_limits` turns into 600 m/s). Set them from each drive's scaling (item 19 in [COMMISSIONING.md](COMMISSIONING.md)); their `min_position_m` / `max_position_m` are no longer read and can go. Do *Rig-agnostic configuration* (Planned features) first: `wavemakers.launch.py` and `oscillator.launch.py` are near-duplicates, and each new rig would copy one again. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **46. Vertical oscillator: tracking under changing loads.** (rig; drive tuning with the drive's owner, from about 2026-11-23)
  Unloaded (2026-11-09) the slide follows with about 180 ms lag; the stroke reaches 97 % at a 2 s period and 90 % at 1 s, for any stroke: a first-order lag of about 77 ms plus about 100 ms delay ([COMMISSIONING.md](COMMISSIONING.md#vertical-oscillator-ladertanken)). The load changes between experiments (models, profiles, water resistance, depth), so one fixed correction cannot hold. Options: (1) a stiffer drive (Kv S-0-0104 = 1.0, acceleration feedforward S-0-0348 / P-0-1126 = 0); (2) record and report the measured motion with every run; (3) a calibration sweep per setup and a gain per period (the controller's gain table is wavemaker-only); (4) amplitude correction during the run. 1 and 2 first. To decide with the experimenters: must the slide reach the requested stroke, or is knowing what it did enough?

- [ ] **51. Vertical oscillator: the working values in the yaml.** (`oscillator_bringup/config/oscillator.yaml`, the `vertical` block; with the experimenters)
  Travel, stroke, maximum speed and rest position are not yet known from the experiments; the yaml has ±0.2 m around 0 and, since 2026-11-12, 0.3 m/s (`max_velocity_rpm` 1800; 0.1 m/s before, a placeholder from `cecf951`). The speed caps a sine's stroke at v·T/π (95 mm at 1 s, 191 mm at 2 s). The drive allows 1.6 m/s (S-0-0091 = 96 000 mm/min, [COMMISSIONING.md](COMMISSIONING.md#the-oscillator-drive-read-2026-11-06)), 4800 rpm at the motor with 20 mm/rev, but the MSK050C-0300's nameplate gives n_max 4700 min⁻¹ (read 2026-11-12), so the ceiling is ≈1.57 m/s (`max_velocity_rpm` 9400). Test 3 ([COMMISSIONING.md](COMMISSIONING.md#done)) passed up to 0.3 m/s (`max_velocity_rpm` 1800) with the naked load on 2026-11-12; the axis allows about 1.0 m/s (ball nut). The yaml now has 1800. Still to do: ask the experimenters for their fastest goal (π × stroke / period); lower `max_velocity_rpm` to about 1.2 × that if it is well below 0.25 m/s, or test further levels if it is above. Short periods stay small anyway: above about 24 mm at 1 s the drive's 3 m/s³ jerk rounds the sine off. `max_goal_acceleration_mps2` and `min_period_s` stay 0 (no limit) until the loaded tests (item 46) show where the lag or the mechanics become a problem; then set them so the controller rejects a goal that is too steep instead of the drive rounding it off (the drive's own 20 m/s² and 3 m/s³ limit every move today; decision 5 in `git show c6bba03:oscillator_sketch.md`).

- [ ] **61. Split `controller_node.cpp`.** (`rig_controller/src/controller_node.cpp`, `rig_controller/test/test_controller_node.cpp`)
  1483 lines, its test 2059. Items 29, 30, 36, 45 and 56 all touch it. Move the lifecycle, goal execution and soft stop / fault handling into their own files, with no change in behaviour; the existing tests must pass unchanged. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

## Low

- [ ] **56. Log when soft stop messages resume.** (`rig_controller/src/controller_node.cpp` `check_soft_stop_liveness()`)
  A gap of more than `soft_stop_timeout_s` (0.2 s) logs `No Soft stop message received ...; treating as stopped` and stops a running motion, but nothing is logged when the messages come back, so the journal can't show how long a gap lasted. Seen 2026-11-12 (Test 3, level 3 restart): a gap 3.3 s after start, likely DDS settling; harmless then. Log once on recovery, e.g. `Soft stop messages resumed after 340 ms`. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **55. Units in the `Drive status` line.** (`drivers/mgate5101_driver/src/indradrive.cpp` `describe()`, `rig_controller/src/indradrive_actuator.cpp`)
  `describe()` hard-codes `deg` and `rpm`; on the vertical oscillator the values are mm and 10 mm/min (drive direction). Decided 2026-11-12: the actuator, which knows `drive_motion`, passes the unit labels to `describe()` (the sketch puts them in `IndraDriveConfig`, so `describe()`'s signature stays). Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **29. `IndraDrive::disable()` can block a lifecycle callback for up to 20 s.** (`drivers/mgate5101_driver/src/indradrive.cpp`)
  Accept, or use a shorter timeout for `disable()`. Decided 2026-11-05: a separate `disable_timeout_ms`, default 3000. Sketch: [controller_fixes_sketch.md](controller_fixes_sketch.md), section 2.

- [ ] **30. `travel_minimum_m` / `travel_maximum_m` are declared only for linear drives at construction.** (`controller_node.cpp`)
  Always declare both. `IndraDriveActuator` now declares them whenever missing (2026-11-09); the controller still declares them only when `drive_motion` is linear. Sketch: [controller_fixes_sketch.md](controller_fixes_sketch.md), section 3.

- [ ] **31. Bridge interface details.** (`bridge_node.cpp`)
  `height`, `period` and `stop` use depth 10 instead of `command_qos`; the `stop` topic and `stop` service share a name (consider renaming the service, e.g. `stop_wave`). Sketch: [controller_fixes_sketch.md](controller_fixes_sketch.md), section 4.

- [ ] **32. Clean-up.**
  Remove the unused `driver_address_` member and the commented-out members at the end of `ControllerNode`; set the license in all seven `package.xml` files. (Formatting and CMake lint fixed 2026-10-07.) Sketch: [controller_fixes_sketch.md](controller_fixes_sketch.md), section 4.

- [ ] **45. A drive fault logs a burst of lifecycle errors.** (`controller_node.cpp`, the lifecycle manager in `wavemakers.launch.py` / `oscillator.launch.py`; rig)
  Seen 2026-11-06 on ladertanken (drive power off during a return, F2026; *Controller and driver fixes* in `git show c6bba03:rpi/README.md`). The controller handles the fault itself: `handle_driver_fault()` deactivates, `on_deactivate` returns `ERROR`, and `on_error` releases everything and leaves it `unconfigured`. `stop_activity()` breaks the bond on the way, so the lifecycle manager logs `CRITICAL FAILURE: SERVER controller IS DOWN after not receiving a heartbeat`, resets all nodes, and its deactivate and cleanup of the controller fail with `No transition matching 4 (2) found for current state unconfigured` and `Failed to change state for node: controller`. Harmless (the drive is already released, the bridge is reset, a service restart recovers), but one fault gives about ten `ERROR` lines, and the real cause (`Driver fault: drive error F2026`) is easy to miss among them. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).
  Options: (a) on a driver fault, do not transition the controller; only halt and disable the drive, break the bond, and let the manager's reset do deactivate and cleanup (one path for every failure, but the controller then depends on the manager to release the drive); (b) keep today's path and document the burst in `rpi/README.md`, *Troubleshooting*, with "look for `Driver fault:` first"; (c) both: keep `on_error` as the safety net if the manager is gone. Proposed in the sketch: (b), plus one line from the controller after `Driver fault:`. Check first whether the manager's `attempt_respawn_reconnection: False` affects which path it takes. The unit test `DriverFaultReleasesResourcesAndAllowsReconfigure` covers today's path.

- [ ] **49. The page's heartbeat margin.** (`rig_web`, `bridge_node.cpp`)
  A wave stops when the page has not acknowledged for 1.5 s (`ACK_TIMEOUT_S` 0.5 s + the bridge's `heartbeat_timeout` 1.0 s). The cause of the waves that stopped by themselves (a name lookup per request) is fixed (2026-11-11). Open: whether 1.5 s is right; it exists to stop a wave when nobody is watching. The server logs each gap (`No page acknowledged for ...`), so a too-tight margin shows in the journal. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **50. Sloshing oscillator on the ladertanken Pi: the last check.** (rig; postponed to 2027, when the sloshing rig is connected; was T5 in the vertical oscillator's sketch)
  The vertical oscillator's branch changed how the sloshing rig starts (`oscillator:=`, the shared `soft_stop@`, `refuse_if_busy.sh`). The refusal of sloshing's unit next to the running wavemaker passed 2026-11-10 (`ExecStartPre` failed), but sloshing has not run since. Remove `OSCILLATOR` from `/etc/default/oscillator` (or set `sloshing`) and start `oscillator@ladertanken`: the journal shows `LinMot: homed` and the nodes are in `/oscillators/sloshing`. Then step 7 of `rpi/README.md`, *Rig test (wavemaker and vertical together)*: sloshing refused next to the running wavemaker, and the wavemaker refused next to it. Set `OSCILLATOR=vertical` again afterwards.

- [ ] **52. `mgate_example`: step size, speed and limits; install it.** (optional; `drivers/mgate5101_driver/examples/example_main.cpp`, `CMakeLists.txt`; Claude's, handed over 2026-11-10; was T10)
  Add `--step`, `--speed` and `--min` / `--max` (tag units) to `main()` and `test_move()`, and install `mgate_example` (`install(TARGETS ...)`), so `ros2 run mgate5101_driver mgate_example --move --profibus-address 3 --input-base 7 --output-base 6 --step 50` moves 50 mm and back. Today `--move` steps 5 mm at 1.7 mm/s on the vertical oscillator, its limits default to −550…500 (`IndraDriveConfig`, so the drive's ±211 mm are the only backstop), and it runs from `~/ros_ws/build/mgate5101_driver/mgate_example`. Not needed now. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **53. One shared gateway connection for both ladertanken drives.** (only if needed; `IndraDriveActuator`, `MGateDriver`, `IndraDrive`)
  Not needed now: Test 2's slow cycles came from the Pi's 200 ms minimum resend time, fixed by `rto_min` 20 ms at every rig start (T18, 2026-11-11; COMMISSIONING.md, *Vertical oscillator*, *Done*). Do it if the gateway's own resends (about 200 ms, not adjustable) become a problem. Remedies, in this order: (1) one process owns the gateway, both drives on one `MGateDriver` (3 requests per cycle for both rigs instead of 6); `set_cycle_callback` then needs more than one subscriber (`IndraDrive` takes the only slot); tests: two `IndraDriveActuator`s on one `FakeMGate` (it plays only one drive today); (2) read the status area only every Nth cycle; (3) FC23 (probably not usable, the inputs are FC4); (4) a slower oscillator poll. Still to check either way: the MGate's maximum number of Modbus TCP clients (3 worked on 2026-11-10). Background: *If Test 2 fails* in `git show c6bba03:oscillator_sketch.md`.

- [ ] **54. Oscillator: the centre of oscillation from `/etc/default/oscillator`.** (`oscillator_bringup/launch/oscillator.launch.py`, `rpi/run.sh`)
  `VERTICAL_UPRIGHT_M` / `SLOSHING_UPRIGHT_M` override `upright_position_m` at launch, as `WATER_DEPTH` does for the wavemaker, so another centre needs no edit of the tracked yaml on the Pi. Sketch: [oscillator_centre_sketch.md](oscillator_centre_sketch.md). 2026-11-18: code, tests (`test_oscillator_centre.py`) and docs done; left: the sketch's *Check on the rig*.

- [ ] **60. `test_driver` for the MGate driver: move its checks, delete the stub.** (`drivers/mgate5101_driver/tests/`, `CMakeLists.txt`)
  `test_driver` is left out of the build because it needs `stub_server.py`. Corrected 2026-11-18: the Modbus layer is still tested end-to-end, through `FakeMGate` (a real Modbus/TCP server) in `test_indradrive_actuator` and `test_controller_node`. Only three checks are `test_driver`'s alone: an out-of-area tag throws, writing an input throws, and straddling tags decode. Move them into `test_mapping`, then delete `test_driver.cpp` and `stub_server.py`. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **62. Sketches name functions, not line numbers.** (the `*_sketch.md` files)
  Line numbers go stale as soon as the code moves (`gain_estimation_sketch.md`, updated 2026-11-18). Name the function or block each change goes in, as `controller_fixes_sketch.md` does, with line numbers at most as a hint and their commit. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **63. One entry point for the docs.** (`README.md`, the root `*.md`)
  The root has seven guides and four sketches, and `LAB_PERSONNEL.md` overlaps README (configuration, motion files). Have README list every doc with one line on who it's for, or move the sketches to `docs/sketches/`. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **64. Remove leftovers.** (`carriage_bringup/`, `.gitignore`)
  The empty `carriage_bringup/launch/` (untracked); add `.pytest_cache/` to `.gitignore`; drop the `OPPSTART.md` and `drivers/mgate5101_driver/compile_commands.json` entries there if those files are gone for good. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

- [ ] **65. Lint the Python.** (`rig_controller/src/*.py`, `rig_web/scripts/`)
  `ruff`, and perhaps `mypy` on `motion_files.py`, which the other tools import, in the same hook or CI as item 58. Sketch: [remaining_items_sketch.md](remaining_items_sketch.md).

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

**When:** not started. On its own branch.

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

### Transfer function from every run (long term)

Pool every run's transfer gain (from `transfer_function` today, and live from a wave probe on a
topic later) into one smooth gain model per wavemaker, with an error bar per period. The model
flags runs that disagree with it and prints a suggested gain table. The design is in
[gain_estimation_sketch.md](gain_estimation_sketch.md).

**When:** not started. Stage 0 (offline pooling) needs nothing new; the live stages need a probe
on a topic. Item 43 first.

Smaller additions to `transfer_function` on the way:

- With three probes, separate the incident and reflected waves (Mansard & Funke 1980) and use
  the incident wave in place of the smoothing over the reflection's ripple.
- The drive's absolute delay, which the record can't give: `--bag <bag>` with
  `wavemaker_setpoint` and `wavemaker_position`, published in the same control cycle. Pair them
  by message order, not receive time (`Float64`, no stamp; bags arrive in bursts). Or a played
  trigger recorded in catman (*Trigger output*).
- Read catman `.bin` directly, which saves the export step.
