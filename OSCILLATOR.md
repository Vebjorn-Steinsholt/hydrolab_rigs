# Sloshing oscillator

Plan and record for running the LinMot sloshing oscillator from the lab Pi with the same
controller, bridge and web page as the wavemakers. Work happens on the branch `feat/oscillator`
and stays off master and the lab Pi until it has run on its rig.

- **(rig)** needs the real oscillator; the unit tests use a simulated Arduino and drive.
- Paths are relative to the repository root.
- Decisions are dated; change them here when they change.

## The rig

A LinMot linear motor moves a sloshing tank back and forth. The physical limits of the tank and
the water are fixed:

| | Limit |
| --- | --- |
| Travel | 5–195 mm, centre 100 mm (hard end stops at 0 and 200 mm) |
| Speed | 2 m/s |
| Acceleration | 9.81 m/s² (1 g, the professor's specification, tested) |
| Frequency | 5 Hz |

Goals give a height H, the peak-to-peak stroke, and a period T. There is no transfer function and
no water depth: the water level changes between experiments but does not affect the control.

## How it fits together

```
 web page / ros2 CLI ──▶ bridge_node ──▶ controller_node ──USB serial──▶ Arduino Uno ──CAN──▶ LinMot drive ──▶ slide
                                         (LinMotActuator)    115200 baud    + MCP2515     500 kbit/s
                                                             22/14-byte
                                                             shield
                                                             CANopen
                                                             node
                                                             12
                                                             frames, CRC
```

| Layer | Where | Does |
| --- | --- | --- |
| Firmware | `firmware/linmot_serial_bridge/` | Serial frames to CAN and back; rejects commands outside 5–195 mm, 3 m/s, 20 m/s²; aborts the motion when no frame arrives for 200 ms (watchdog). `link_test.py` tests it without ROS. |
| Driver | `drivers/linmot_serial_driver/` | `SerialBridge` sends one frame every 20 ms (a command, the streamed target or a status request), so the watchdog only trips if the Pi stops. `LinMotDrive` brings the drive up from its status word, streams VAI go-to-position commands and latches faults. No ROS. |
| Actuator | `rig_controller/src/linmot_actuator.*` | Implements `Actuator` for the controller, selected with `drive_name: linmot`. |
| Controller | `rig_controller/src/controller_node.cpp` | Lifecycle node, trajectories, limits, goals; shared with the wavemakers. |

Three layers of limits, the tightest first: the controller (5–195 mm, 2 m/s, 9.81 m/s², 5 Hz;
step D), the firmware (5–195 mm, 3 m/s, 20 m/s²) and the drive's own position limits in
LinMot-Talk (3 / 197 mm). This rig has no hardware e-stop: beyond those, the drive can only be
switched off from LinMot-Talk (take control, switch off, release control).

## Decisions

- **Link (2026-10-19):** USB serial from the Pi to the Arduino instead of the earlier Wi-Fi/UDP.
  An Ethernet shield clashed with the CAN shield on SPI and interrupts.
- **No drive-internal sine (2026-10-19):** the drive's built-in sine (old command 8) keeps
  running when the Pi stops sending, so it was removed. The Pi streams every position.
- **VAI limits (2026-10-21):** each go-to-position command carries the sine's peak velocity and
  peak acceleration. From experience, lower limits make the motion jitter.
- **Start-up (2026-10-21):** at activation the actuator reads the status word and acts on it:
  acknowledges an error, switches on, homes if needed (`home_if_needed`, default true). A fatal
  error, a silent Arduino or drive, or a drive that does not switch on fails the activation.
- **Board (2026-10-21):** an Arduino Uno R3 for now; an Uno R4 WiFi also runs the firmware.
- **Names (2026-10-21):** `wavemaker_controller` became `rig_controller`; the node, bridge and
  actuator interface became `controller_node`, `bridge_node` and `Actuator`. Actuator classes
  are named after the drive (`IndraDriveActuator`, `LinMotActuator`); the rig's behaviour
  comes from the `rig` parameter (step R).
- **Topics and actions name the rig (2026-10-21):** a `rig` parameter (`wavemaker` |
  `oscillator`, default `wavemaker`) on the controller, bridge and web page; for the oscillator
  `move_oscillator`, `oscillator_setpoint` and so on (step R). Confirmed: for example
  `/wavemakers/ladertanken/move_wavemaker`, `/wavemakers/ladertanken/wavemaker_position`,
  `/oscillators/sloshing/move_oscillator`, `/oscillators/sloshing/oscillator_position`.
- **`rig` decides the rig's behaviour (2026-10-21):** `rig: oscillator` selects the oscillator
  trajectory and checks. `wavemaker_type` (`piston` | `flap`) stays, for wavemakers only; there
  is no `wavemaker_type: oscillator`.
- **Neutral parameter names (2026-10-21):** the parameters both rigs use lose their `wavemaker_`
  prefix (step R); `wavemaker_id`, unused, is removed. Names confirmed 2026-10-21.
- **Pregenerated goals (2026-10-21):** allowed for the oscillator, very low priority.
- **Bringup (2026-10-21):** a separate package `oscillator_bringup` with its own
  `oscillator.launch.py`, namespace `/oscillators/sloshing`.
- **Pi (2026-10-19):** the oscillator and the wavemaker never run at the same time on the lab Pi;
  separate `oscillator@.service`, shared soft stop button on GPIO17.

## Plan

Work for all rigs that comes after this plan, such as a go-to-position action for CLI users, is
in [TODO.md](TODO.md#planned-features-all-rigs).

### Done

- [x] Firmware: framing, CRC, READY, watchdog (2026-10-19); switch-on after power-up and error
  acknowledge (2026-10-20); node 12 and INT on D2 (2026-10-21).
- [x] `link_test.py --move` passes on the rig: link, CAN path, homing, streamed sine, rejected
  command, watchdog (2026-10-21).
- [x] Drive limits in LinMot-Talk and the abort fix (see [Found on the rig](#found-on-the-rig)).
- [x] Step A, `linmot_serial_driver` with `linmot_probe` (2026-10-21; tested against the
  simulator only).
- [x] Renames to `rig_controller`, `controller_node`, `bridge_node`, `Actuator` (2026-10-21).
- [x] Step B code: `LinMotActuator`, `actuator_type` in the controller, tests (2026-10-21,
  `d0cf237`; tested against the simulator only).
- [x] Drive selection for future drives (2026-10-23): `actuator_type` renamed to `drive_name`
  (required, no default) and `actuator_drive_type` to `drive_motion`; old names fail configure.
  The degrees-to-metres mapping moved into `DriveScale` (`rig_controller/src/drive_scale.hpp`),
  used by the controller's limits and `IndraDriveActuator`. Tests pass (130, 0 failures).

### Now: step B on the rig

Commands below assume the environment in [How to run](#how-to-run). Close the Arduino IDE's
Serial Monitor first: only one program can have the port open.

- [x] Commit step B.
- [x] Upload the firmware's command-count change (motion command count instead of an empty
  header and an extra SYNC; PDO1 last). It is committed with step B (`d0cf237`) and ran on the rig
  on 2026-10-22. In the Arduino IDE: open the sketch, board "Arduino Uno", port
  `/dev/ttyACM0`, Upload (needs the `mcp_can` library in `~/Arduino/libraries`). Or, with the
  IDE closed: (rig)

  ```bash
  arduino --upload --board arduino:avr:uno --port /dev/ttyACM0 \
    ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge/linmot_serial_bridge.ino
  ```

- [x] Run `link_test.py` with motion (2026-10-22: all checks passed, 301 of 300 feedbacks,
  round trip median 7.6 ms, max 9.9 ms); expect "one feedback per command: about 300 of 300"
  (before the change it was 600 of 300) and "All checks passed". The drive must be homed; keep
  LinMot-Talk open and connected to switch the drive off (the rig has no e-stop). (rig)

  ```bash
  cd ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge
  python3 link_test.py --move           # add --home if the drive is not homed
  ```

  Without `--move` it only checks the link and the CAN path, and nothing moves. With `--move` it
  also switches on, moves to 100 mm, streams a 5 mm sine, checks that an out-of-range command is
  rejected and that the watchdog stops a move, and switches the drive off.

- [x] Run `linmot_probe` against the Uno and drive (no motion). (rig) 2026-10-22: READY after
  532 ms, 100 feedbacks for 100 frames, 0 bad CRCs, 71 bytes skipped. It also showed the drive in
  error (0x0878) at 102.06 mm after `link_test.py --move`: see the open item below.

  ```bash
  ros2 run linmot_serial_driver linmot_probe /dev/ttyACM0      # 2 s; add a number for longer
  ```

  Expect READY after about 0.5 s, the drive's position and status word, as many feedbacks as
  frames, 0 bad CRCs and about 71 skipped bytes (the Arduino's start-up text). It sends NMT
  operational and status requests only.

- [x] Fix the drive error left after `link_test.py --move`: status 0x0878 at 102.06 mm, where the
  watchdog had stopped the slide; LinMot-Talk: "Motion Cmd In Wrong State". The command-count
  firmware sent the switch-on control word and the next motion command in one frame, so after
  the watchdog's abort the drive got the motion command while still aborting. Fixed in the
  firmware (2026-10-22): when the last control word was not 0x003F, it is sent on its own
  first. `link_test.py` now checks the return to the centre after the watchdog and fails if the
  drive ends in error. Acknowledge the error in LinMot-Talk, upload, then run: (rig)

  ```bash
  arduino --upload --board arduino:avr:uno --port /dev/ttyACM0 \
    ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge/linmot_serial_bridge.ino
  cd ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge
  python3 link_test.py --move           # add --home if the drive is not homed
  ```

  Expect "back at the centre after the watchdog", "drive ends without an error" and still about
  300 of 300 feedbacks during the sine.
  2026-10-22: passed (back at 100.03 mm, final status 0x0830, 301 of 300, round trip median
  7.5 ms, max 9.9 ms). Run right after an upload, the test once took a READY from before the
  reset at port open and lost its bad-CRC frame in that reset; it now flushes the port and only
  accepts a READY that answers its own ping.
- [x] Drive's own sync watchdog (Legacy Sync Watchdog), so the drive stops itself when the link
  goes (USB pulled, Uno or Pi hung): today it stays on and holds, and this rig has no e-stop.
  The drive measures the time between SYNC frames and raises an error if it exceeds 1.5 ×
  the monitored cycle time, only while it is NMT operational (LinMot CANopen manual for SG5).
  The bridge sends a SYNC every 20 ms. (rig)
  - [x] LinMot-Talk: `CANopen Interface → Legacy Sync Watchdog`: `LSW Enable` = Enable
    (2026-10-22).
  - [x] `LSW monitored Cycle Time` = 50000 µs (50 ms: error after 75 ms without a SYNC). The
    default 5000 µs would stop the drive between every two frames. Save a new `.lmc` backup.
    Done 2026-10-22.
  - [x] Firmware: `CMD_PRE_OPERATIONAL` (8) puts the drive into NMT pre-operational, which turns
    the sync watchdog off; `LinMotDrive::disable()` sends it after the switch-off is confirmed,
    `link_test.py` and `linmot_probe` at the end. So the idle drive does not go into error.
    `link_test.py` no longer pauses longer than 20 ms between frames (2026-10-22).
  - [x] Upload the firmware and run the link test; in the watchdog check expect "the drive
    stopped itself too (sync watchdog on)", then "operation enabled again", "still homed" and
    the usual passes. 2026-10-22, first run: the drive stopped itself at 101.31 mm on its way
    to 105 mm, LinMot-Talk showed "CanOpen Err. Ctrl Timeout" (CDh), and the test lost the
    drive: after the error it is NMT pre-operational and sends no feedback until an NMT start.
    The test now sends one after the pause and before the final switch-off. Second run: all
    checks passed; no stop during the sine at 50 ms (so the "divided by 100" bug seems absent),
    the drive stopped itself at 101.30 mm, recovered, back at 100.14 mm, ends without an error.
    Commands:

    ```bash
    arduino --upload --board arduino:avr:uno --port /dev/ttyACM0 \
      ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge/linmot_serial_bridge.ino
    cd ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge
    python3 link_test.py --move           # add --home if the drive is not homed
    ```

  - [x] Repeat the controller's USB test (below): the drive must now stop and switch off by
    itself; LinMot-Talk shows the error (expected: CDh "Error Control Timeout"). 2026-10-22:
    the controller faulted ("serial port down"), the drive stopped itself, and after plugging
    back in `bring_up()` acknowledged its error (0x087A) and switched it on again.
- [x] Stop ModemManager probing the Uno. Ubuntu's ModemManager treats a newly plugged-in
  `/dev/ttyACM*` as a possible modem for a few seconds (AT commands, toggled control lines,
  which reset the Uno); the Uno has `ID_MM_CANDIDATE=1`. The first activation right after
  plugging back in failed with "the drive does not answer on CAN", 15 s later it worked.
  Probably this, not proven. The udev rule `rpi/99-linmot-bridge.rules` excludes the Uno R3
  (USB ID 2341:0043) from ModemManager and also gives it the fixed name `/dev/linmot_bridge`.
  2026-10-22 on the PC, with the first version of the rule (ModemManager line only): the probe
  right after plugging in answered at once (READY after 530 ms, 101 of 101 feedbacks).
  Replace it with the file from the repo, so the PC has `/dev/linmot_bridge` too:

  ```bash
  sudo cp ~/ros_ws/src/ros2_wavemakers/rpi/99-linmot-bridge.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ls -l /dev/linmot_bridge                                   # → ttyACM0
  udevadm info -q property -n /dev/linmot_bridge | grep ID_MM_DEVICE_IGNORE   # → 1
  ```
- [x] First controller run, lifecycle by hand: configure, activate (homing), `return_to_upright`,
  faults, deactivate. Until step D it uses `wavemaker_type: piston` as a placeholder: send no
  `height`/`period` goals, they would be turned into a stroke by wave theory. (rig) The
  parameter file below has the names from step R; the 2026-10-21/22 runs used the old ones.

  Save as `/tmp/oscillator_test.yaml`:

  ```yaml
  /**:
    ros__parameters:
      drive_name: linmot
      serial_port: /dev/ttyACM0
      drive_motion: linear
      wavemaker_type: piston          # placeholder until step D
      water_depth: 1.0                # required > 0 for piston, not used otherwise
      rig: oscillator                 # names: move_oscillator, oscillator_position, ...
      upright_position_m: 0.1
      travel_minimum_m: 0.005
      travel_maximum_m: 0.195
      min_position_m: 0.005
      max_position_m: 0.195
      positioning_velocity_margin: 1.0
      return_to_upright_max_velocity_mps: 0.02   # slow for the first runs
      step_timeout_ms: 2000
  ```

  Terminal 1, the controller:

  ```bash
  ros2 run rig_controller controller_node --ros-args \
    -r __ns:=/oscillators/sloshing --params-file /tmp/oscillator_test.yaml
  ```

  Terminal 2, in place of the soft stop button. The controller allows no motion unless it hears
  `soft_stop: false` at least every 0.2 s; Ctrl-C here acts like pressing the button.

  ```bash
  ros2 topic pub -r 20 /oscillators/sloshing/soft_stop std_msgs/msg/Bool "{data: false}" \
    --qos-reliability reliable --qos-durability transient_local
  ```

  Terminal 3, the lifecycle and a return to 100 mm:

  ```bash
  NS=/oscillators/sloshing
  ros2 lifecycle set $NS/controller configure    # reads parameters, port not opened yet
  ros2 lifecycle set $NS/controller activate     # opens the port, switches on, homes if needed
  ros2 lifecycle get $NS/controller
  ros2 service call $NS/return_to_upright \
    rig_interfaces/srv/ReturnToUpright "{requester: 'cli', tolerance: 0.0}"
  ros2 topic echo $NS/oscillator_position         # only during goals and returns: start it
                                                  # in another terminal before the return
  ros2 lifecycle set $NS/controller deactivate   # switches the drive off
  ```

  Faults, one at a time while active, each followed by a new configure and activate: pull the
  USB cable, switch the drive off in LinMot-Talk. Each should end in a fault
  with a clear reason in terminal 1 and no further motion.

  2026-10-22: configure, activate (0.8 s, drive already homed), `return_to_upright`
  (`status=0`, final position 0.09969 m) and deactivate (drive switched off, 0x0830) work.
  After every activation the controller logs the soft stop timeout once (TODO 36). Fault tests
  (2026-10-22): USB pulled → "Driver fault: serial port down: serial port closed or unplugged",
  and with the sync watchdog on the drive stops itself; switched off in LinMot-Talk → "Driver
  fault: the drive switched off 0x0830 (homed)". Each ends with the controller unconfigured,
  as for the wavemakers. With the USB cable pulled, the Uno and its watchdog lose power and the
  controller cannot reach the drive; before the sync watchdog the drive stayed on and held, now
  it stops itself.

### R: names that fit both rigs

Done 2026-10-22: `rig` on the controller, bridge, web page and `fake_controller.py`; the
parameters renamed (in the controller, `indradrive_actuator.cpp`, `wavemakers.yaml`, the launch
files, tests and docs); `wavemaker_id` removed; the controller rejects the old names at
configure, also from a `--params-file`. `play_dat.py` has no ROS part yet, only its TODO note
names `move_<rig>`. On the rig (2026-10-22): `/oscillators/sloshing/move_oscillator` appears,
and a `return_to_upright` with the new parameter file ends at 0.09976 m (`status=0`).

- [x] `rig` parameter (`wavemaker` | `oscillator`, default `wavemaker`) on `controller_node`,
  `bridge_node` and the web page.
- [x] Topics and the action built from it: `move_<rig>`, `<rig>_setpoint`, `<rig>_position`,
  `<rig>_velocity`, `<rig>_actuator_setpoint`, `<rig>_state`, `<rig>_message`. Neutral names
  (`return_to_upright`, `cancel_all_goals`, `soft_stop`, `heartbeat`, `height`, `period`,
  `start`, `stop`) stay. With the default, every wavemaker topic keeps its name.
- [x] Packages and action type renamed as well (2026-10-22, after step R): `wavemaker_web` →
  `rig_web`, `wavemaker_interfaces` → `rig_interfaces`, `MoveWavemaker` → `MoveRig`
  (`ros2 launch rig_web demo.launch.py [rig:=oscillator]`; goals as
  `rig_interfaces/action/MoveRig`). `wavemaker_bringup` keeps its name: it is wavemaker-only.
- [x] Parameters:

  | Today | New |
  | --- | --- |
  | `wavemaker_upright_position_m` | `upright_position_m` |
  | `wavemaker_upright_is_minimum` | `upright_is_minimum` |
  | `wavemaker_minimum` / `wavemaker_maximum` | `travel_minimum_m` / `travel_maximum_m` (goal travel limits; not the actuator's `min_position_m` / `max_position_m`) |
  | `wavemaker_mode_pregenerated` | `mode_pregenerated` (also the launch argument); removed 2026-10-30, each goal picks its type |
  | `wavemaker_id` | removed (unused) |
  | `wavemaker_type` | unchanged, wavemakers only |

  In the controller, `wavemakers.yaml`, the launch files, `play_dat.py`, `fake_controller.py`,
  the tests, README, TODO and COMMISSIONING.
- [x] The controller fails configure when it is given an old parameter name, with the new name in
  the message. ROS 2 ignores undeclared parameters silently, so an old `wavemaker_minimum`
  would otherwise leave the travel limit at its default.
- [x] `--rig` for `fake_controller.py` and `play_dat.py`.

### C + D: the oscillator in the controller

Code done 2026-10-22 (commit `e903684`): 151 tests pass. Deceleration is sent equal to the
acceleration (the sine brakes as hard as it accelerates). The rig test below is still open.

Three commits, each testable on its own. Decided 2026-10-22: the parameter names below; the start
blend may reject a goal whose steady sine would pass (TODO 25 covers a longer blend later);
pregenerated goals are checked and capped like regular ones, and the smoothing of noisy data
belongs in `play_dat.py` (item at the end).

**D1: oscillator trajectory**

- [x] `WaveTrajectory::configure()`: type `"oscillator"` gives `amplitude_ = H / 2`, with no
  dispersion relation, gain or flap scaling; blend, `sample()` and the required range stay.
- [x] `controller_node.cpp`: the checks of `wavemaker_type`, `water_depth`, flap, hinge and gain
  table only for `rig: wavemaker`; `rig: oscillator` requires `upright_is_minimum: false`; log
  "as an oscillator"; gain lookup only for wavemakers; `trajectory_.configure()` gets type
  `"oscillator"` when `rig` is `oscillator`.

**C: the acceleration through to the drive**

- [x] `wave_math`: `Blend` gets `dds` (`(60u − 180u² + 120u³) / T²`);
  `kQuinticPeakAccelerationFactor = 5.7735` (10/√3, peak acceleration 5.7735 · d / T²).
- [x] `peak_acceleration()`: `WaveTrajectory` (A·ω² after the blend, the blend sampled like
  `peak_velocity()`), `PregeneratedWaveTrajectory` (blend, and second differences of the
  samples), and the return to upright in the node.
- [x] `Actuator`: `to_actuator_setpoint(position, velocity, acceleration)` and
  `max_acceleration_mps2()` (IndraDrive: infinity; LinMot: `max_acceleration_mps2`). Update
  both actuators, `FakeActuator` and the actuator tests.
- [x] Node: parameter `positioning_acceleration_margin` (default 1.2, ≥ 1);
  `accept_peaks(velocity, acceleration)` stores the acceleration cap (peak × margin, at most
  the actuator's maximum) and rejects goals above `max_acceleration_mps2()`;
  `publish_and_write_setpoint()` passes the cap; the return passes its own.
- [x] `LinMotActuator`: remove `linmot_default_acceleration_mps2`; acceleration 0 is refused.
- [x] Oscillator yaml: `positioning_velocity_margin: 1.0`, `positioning_acceleration_margin: 1.0`
  (VAI limits at the sine's maxima).

**D2: goal checks** (0 = off, so the wavemakers are unchanged)

- [x] `max_goal_acceleration_mps2`: reject a goal whose peak acceleration (blend included) is
  above it; 9.81 for the oscillator.
- [x] `min_period_s`: reject a regular goal with a shorter period; 0.2 for the oscillator.
  (2 m/s is already checked against the LinMot actuator's `max_velocity_mps`.)

**Tests and docs**

- [x] `test_wave_trajectory.cpp`: oscillator amplitude H/2 independent of depth and gain;
  `peak_acceleration()` = A·ω² for a steady sine and larger for an off-centre blend; quintic
  peak acceleration 5.7735 · d / T²; second differences of a known parabola.
- [x] `test_controller_node.cpp`: `rig: oscillator` configures without water depth or flap
  parameters; `upright_is_minimum: true` rejected; goals above 9.81 m/s² or below 0.2 s
  rejected; `FakeActuator` sees acceleration = peak × margin, also for a return.
- [x] `test_linmot_actuator.cpp`: `write_setpoint` with acceleration 0 fails.
- [x] README parameter table: `positioning_acceleration_margin`, `max_goal_acceleration_mps2`,
  `min_period_s`, and `rig: oscillator` as the oscillator's type.
- [x] Run the unit tests (no hardware):

  ```bash
  cd ~/ros_ws
  colcon build --packages-up-to rig_controller
  colcon test --packages-select rig_controller linmot_serial_driver
  colcon test-result --verbose
  ```

- [x] On the rig, with `/tmp/oscillator_test.yaml` switched to the oscillator (drop
  `wavemaker_type` and `water_depth`; add the two margins at 1.0,
  `max_goal_acceleration_mps2: 9.81`, `min_period_s: 0.2`, `max_acceleration_mps2: 20.0`), the
  first regular goal, small and slow: 10 mm stroke at 2 s, around 100 mm. Terminals 1 and 2 as
  in step B; terminal 3 (Ctrl-C cancels the goal, then return to upright): (rig)

  ```bash
  ros2 action send_goal $NS/move_oscillator rig_interfaces/action/MoveRig \
    "{height: 0.01, period: 2.0}"
  ros2 service call $NS/return_to_upright \
    rig_interfaces/srv/ReturnToUpright "{requester: 'cli', tolerance: 0.0}"
  ```

  Expect the slide to move between 95 and 105 mm (`ros2 topic echo $NS/oscillator_position` in
  a fourth terminal), and goals above 9.81 m/s² or below 0.2 s to be rejected with a reason in
  terminal 1, for example `"{height: 0.19, period: 0.5}"` (about 15 m/s²) and
  `"{height: 0.01, period: 0.15}"`.

  2026-10-23: passed. Activation homed the drive (21 s), `return_to_upright` ended at
  100.22 mm (`status=0`), the 10 mm / 2 s goal ran and both bad goals were rejected. Recorded
  with `ros2 bag record` of `oscillator_setpoint`, `oscillator_position` and `oscillator_velocity`
  (about 20 s each), compared after the 3 s blend:

  | Margins (velocity / acceleration) | 1.0 / 1.0 | 1.2 / 5.0 |
  | --- | --- | --- |
  | Lag, best match | ≈ 200 ms | ≈ 100 ms |
  | Tracking error rms / max | 2.09 / 2.96 mm | 0.98 / 1.53 mm |
  | Mid-stroke speed dips | 85 of 410 steps | 4 of 409 |
  | Position range | 95.251-104.647 mm | 95.251-104.661 mm |
  | Repeated position samples | 1.8 % | 2.5 % |

  Control cycle 20.0 ms (19.3-20.6 ms) in both. The lag comes from VAI: every setpoint is a
  move that has to stop at its target, so with deceleration = the sine's own peak (A·ω² ≈
  49 mm/s²) the slide runs v²/(2a) ≈ 2.5 mm behind, and with velocity margin 1.0 it cannot
  catch up. Margins 1.2 / 5.0 halve the lag; use them in `oscillator.yaml` (step E). The stroke
  stays 0.59 mm short (0.25 mm at the bottom, 0.34 mm at the top) in both runs, with a stall of
  about 150 ms at each turnaround, so it is not the braking limit; accepted, the rig is for
  demonstrating sloshing. If it ever matters: LinMot-Talk oscilloscope, Demand vs Actual
  Position at a turnaround (demand flat → VAI planning, streaming mode; actual behind demand →
  friction or the drive's gains). On the PC this also needed the current
  `rpi/99-linmot-bridge.rules` for `/dev/linmot_bridge`.

**Later, low priority: `play_dat.py` for pregenerated goals**

- [ ] Its ROS part (the TODOs in `play()`), with `--rig`.
- [ ] Smooth the data (low-pass) and check its acceleration before sending
  (`--max-acceleration`), so noisy files do not hit the controller's acceleration check; report
  the run and time where a file is too fast.

### E: bringup

- [x] Package `oscillator_bringup`: `config/oscillator.yaml` under
  `/oscillators/sloshing/controller` and `launch/oscillator.launch.py` (controller, bridge,
  lifecycle manager, soft stop, web page with `rig:=oscillator`). Start the yaml from the C + D
  rig test file, with `positioning_velocity_margin: 1.2` and
  `positioning_acceleration_margin: 5.0` (see the 2026-10-23 result). Changed to 1.0 / 1.0 in
  step G: 1.2 / 5.0 made the rig hum.
  Decided 2026-10-23: `return_to_upright_max_velocity_mps: 0.1`; the soft stop node (GPIO) is
  tested on the Pi only, so the launch file starts it unconditionally; the web page needs only
  `rig:=oscillator` (`web.launch.py` now picks `oscillators/sloshing` itself). Check on the rig
  that the lifecycle manager's bond (`bond_timeout` 4 s) survives an activation that blocks
  about 20 s while the drive homes.
- [x] `serial_port: /dev/linmot_bridge` in `oscillator.yaml`, the fixed name from
  `rpi/99-linmot-bridge.rules` (step F). Unlike `/dev/ttyACM0` it does not change with other
  USB devices, and unlike the `/dev/serial/by-id/...` path it survives replacing the Uno.
- [x] On the rig (2026-10-23, the ladertanken Pi, which the oscillator shares with the
  ladertanken wavemaker): (rig)

  ```bash
  cd ~ && source /opt/ros/jazzy/setup.bash && source ~/ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=1
  ros2 launch oscillator_bringup oscillator.launch.py
  ```

  Homing 21 s, then `Managed nodes are active`. The bond is no problem: the lifecycle manager
  creates it only after the activation has finished ("Server controller connected with bond"
  right after `LinMot: homed`), so the default 4 s stays. From the web page
  (`http://ladertanken-rpi.local:8080/`): height 0.01 m / period 2 s runs, the soft stop button
  stops it, Go to centre works, Ctrl-C stops the motion. Found and fixed on the way: the plot was
  scaled around 0, so the slide's motion around 0.1 m filled 5 % of it (`app.js` now centres the
  scale on the data); `web.launch.py` takes `rig:=oscillator` alone. The Pi needed the udev rule
  and a full `colcon build`: `wavemaker_bringup` was missing from `install/`, so the wavemaker
  service would not have started either.

### F: service on the Pi

- [x] `rpi/oscillator@.service`, copied from `wavemaker@.service` and, like it, without an
  `[Install]` section, so it never starts (or homes) at boot.
- [x] Each service refuses to start while the other runs (`ExecStartPre`; `Conflicts=` would
  stop the running one instead). Needed: the oscillator runs on the ladertanken Pi, and both use
  port 8080, GPIO17 for the soft stop and the same ROS domain.
- [x] The udev rule on the Pi: ModemManager off for the Uno, and `/dev/linmot_bridge`. Check
  first whether ModemManager runs there (`systemctl status ModemManager`); the symlink is
  needed either way.

  ```bash
  sudo cp ~/ros_ws/src/ros2_wavemakers/rpi/99-linmot-bridge.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ls -l /dev/linmot_bridge
  ```

- [x] The service user in `dialout`; `rpi/README.md` updated.
- [x] Done 2026-10-23 on the ladertanken Pi (commit `95f9d24`, install and use in
  `rpi/README.md`, *Oscillator*). `run.sh` and `stop.sh` take `RIG=oscillator` from the unit;
  `refuse_if_busy.sh` (`ExecStartPre` of both units) refuses a start while the other rig's
  service runs or the web port is taken, which also catches a launch by hand. Rig test: start
  (homing, `Managed nodes are active`), `systemctl stop` mid-wave (ordered shutdown through the
  lifecycle manager, then SIGINT, every process finished cleanly), start refused in both
  directions ("… is running; stop it first"), both refused during a launch by hand, both
  `inactive (dead)` after a reboot. (rig)

### G: commissioning (rig)

- [x] Without water: activation with homing, `return_to_upright`, small sines (for example 10 mm
  at 2 s), then steps towards 2 m/s, 9.81 m/s² and 5 Hz. Record setpoint and position, and
  check whether VAI at the sine's maxima runs without jitter.

  2026-10-23, through `oscillator@ladertanken`, one bag for the session (`/tmp/osc_G_dry`,
  topics `oscillator_setpoint`, `_position`, `_velocity`, `height`, `period`,
  `oscillator_message`):

  - **Envelope.** A sine has v = √(a·A), so with 9.81 m/s² and at most 90 mm amplitude the
    fastest goal is ≈ 0.94 m/s; 2 m/s cannot be reached. The start blend raises the peak
    acceleration: ~9 % at full stroke, ~65 % at 5 Hz (TODO 25), so the largest 5 Hz goal is
    H ≈ 0.017 m. **Press Go to centre before every goal**: after Stop the slide holds where it
    is, and the blend from there can need much more (14.6 m/s² for 0.018 m / 0.2 s, rejected).
  - **Overshoot at the ends** with margins 1.2 / 5.0: from 50 mm strokes the slide passed the
    setpoint by 2.3-3.0 mm at each end; full stroke (0.18 m / 0.9 s) reached 7.2 and 192.9 mm.
    The LinMot sits in a steel box with hard stops at 0 and 200 mm, so the goal travel is now
    `travel_minimum_m: 0.015`, `travel_maximum_m: 0.185` (H ≤ 0.17 m).
  - **Hum.** With 1.2 / 5.0 every run hummed mechanically, worst at 0.10 m / 0.7 s; with
    1.0 / 1.0, 10 mm / 2 s is quiet, 0.10 m / 2 s has a very weak hum and 0.10 m / 0.7 s is
    better but not gone. The high acceleration margin turns every 20 ms setpoint into a hard
    VAI move; what is left at fast large strokes is probably the 50 Hz stepping itself (a
    streaming mode would remove it, not worth it for a demo rig). Margins are now 1.0 / 1.0
    (~200 ms lag, strokes slightly short, accepted). Mechanics checked 2026-10-26 (stator and
    box bolts, play of the slider, the slider-to-tank coupling); nothing reported.
  - Goals accepted from the centre with today's `oscillator.yaml` (largest H per period):
    0.17 m up to 0.7 s (travel), 0.16 m at 0.6 s, 0.11 m at 0.5 s, 0.07 m at 0.4 s, 0.04 m at
    0.3 s, 0.027 m at 0.25 s, 0.017 m at 0.2 s (acceleration).
- [x] Fault paths: soft stop button, USB, LinMot-Talk. Soft stop button done (Pi,
  2026-10-23); USB and LinMot-Talk done on the PC (2026-10-22) and through the service with
  water (2026-10-26):

  - USB pulled during 0.10 m / 2 s: `Driver fault: serial port down: serial write: No such
    device` within 40 ms, then `CRITICAL FAILURE … Shutting down related nodes`.
  - Drive switched off in LinMot-Talk: `Driver fault: no feedback from the drive for 100 ms`
    (not "the drive switched off" as on 2026-10-22: the switch-off also stopped the drive's
    feedback).
  - **Found: the nodes came back up by themselves.** Nav2's lifecycle manager defaults to
    `attempt_respawn_reconnection: true`: after a bond failure it resets the nodes, waits for
    them to answer (ours never died, so at once) and runs the whole start-up again, 1-3 s after
    the fault. Activating the controller switches the drive on (LinMot: acknowledges errors,
    homes if needed; IndraDrive: a new actuator clears the latched fault). Both retries failed
    only by chance (port gone; drive not answering on CAN). Fixed in `oscillator.launch.py`
    and `wavemakers.launch.py` (`attempt_respawn_reconnection: False`, commit `d192733`);
    checked locally with `bridge_node` under the lifecycle manager (True: active again after
    10 s; False: stays unconfigured), then on the rig: oscillator, drive switched off in
    LinMot-Talk, and the ladertanken wavemaker, e-stop (`Driver fault: drive error F2026`,
    undervoltage in the power section). Both end with `Managed nodes have been reset` and
    nothing after it; the service keeps running with the nodes down until it is restarted.
  - Unexplained, once: `Stopped: client heartbeat (LabVIEW or web page) lost` during a
    0.10 m / 2 s run. The bridge stopped and held as designed; the page had stopped
    acknowledging (tab hidden or closed, laptop asleep, or Wi-Fi). Watch for it in
    demonstrations.
- [x] With water: the same steps; watch the tracking error and the drive's following-error
  limit (20 mm), and the ends first (slosh forces act at the turnarounds).

  2026-10-26, filled tank with a lid, margins 1.0 / 1.0, bags `/tmp/osc_G_water*` and
  `/tmp/osc_G_full`. No drive errors or following-error warnings in any run.

  - 10 mm / 2 s: as dry (stroke −4.4 %, lag ~180 ms); the water makes no difference there.
  - **Resonance about 1.0 s** (sweep at H 0.005 m from 1.6 to 0.4 s: biggest waves at 1.0 s,
    seen; on the slide the largest push, 0.45 mm rms after removing the lag, against
    0.18-0.26 mm at the neighbouring periods).
  - At 1.0 s: the water **hits the lid from H 0.02 m**; no leaks. The push on the slide grows
    up to H 0.05 m (1.8 mm rms) and then levels off (1.5-1.6 mm at 0.075 and 0.10 m): the lid
    limits the sloshing. From 0.05 m the water carries the slide **beyond the setpoint**, up to
    2.2 mm at 0.10 m. With the lid, spilling does not limit the stroke.
  - Off resonance (2.0 s): 0.05 and 0.10 m, push 0.6 and 1.1 mm, stroke within +1 %.
  - **Full stroke wet**, H 0.17 m / 0.9 s (near resonance): the slide reached **9.0 and
    190.7 mm**, 5-6 mm beyond the setpoint (dry it was ~3 mm), steady from the first cycle
    (9.0-11.5 / 188.0-190.7 mm per cycle). 9 mm of the box are left: the 15 mm travel margin
    is enough, keep `0.015 / 0.185` (or `0.020 / 0.180`, H ≤ 0.16 m, for at least 10 mm).
  - Correction: the "repeated position samples" are not a feedback problem. They are highest
    on small slow runs (14 % at 5 mm) and zero on large ones: the slide stands still for a
    moment at each turnaround (the stall seen on 2026-10-23).
- [ ] Pregenerated goals: check the peak acceleration taken from the samples.

### Merge

- [x] Merge `feat/oscillator` into master once G is far enough along (merged 2026-11-02, after the rig test in `rpi/README.md`).
- [x] On the lab Pi, remove `build/` and `install/` of `wavemaker_controller` and rebuild. The
  wavemaker runs with its topic names unchanged (`rig` defaults to `wavemaker`); its parameters
  have the new names from step R and `drive_name: indradrive` is required, so any parameter file
  or command kept outside the repository has to be updated.
  2026-10-23, ladertanken Pi: already on `feat/oscillator`, old packages gone; after a full
  `colcon build` the ladertanken wavemaker and the oscillator both run on the branch code
  (step R names, `drive_name`/`DriveScale`, the rig-neutral log texts). lilletanken and mclab
  are not deployed (future, they need gateways), so ladertanken is the only lab Pi.

## How to run

Commands assume a sourced workspace and a ROS domain other than 0:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=<n>
```

The commands for each rig step are under its item in the [Plan](#plan). Build and unit tests,
with no hardware (the driver's tests and `test_linmot_actuator` run against a simulated Arduino
and drive on a pseudo-terminal):

```bash
cd ~/ros_ws
colcon build --packages-up-to rig_controller linmot_serial_driver
colcon test --packages-select linmot_serial_driver rig_controller
colcon test-result --verbose
```

## Found on the rig

What the bench and rig tests showed, 2026-10-21, Uno R3 with the MCP2515 shield and the drive:

- The drive is CANopen node 12 (0x0C); the UDP version's node 4 was wrong. The configuration
  has Node-ID Source Select 1 and a parameter value of 63, so 12 most likely comes from the
  drive's address switches (not checked). The shield's INT line is on D2, not D5. Both were
  found by polling the MCP2515 directly and printing every frame.
- `mcp_can` prints about 71 bytes of debug text at start-up (`DEBUG_MODE 1` in its
  `mcp_can_dfs.h`); the framing skips it. At port open the Uno resets and answers after about
  0.5 s.
- Round trip Pi → drive → Pi: median 8.5 ms, maximum 10.6 ms with 6 CAN frames per command;
  median 7.6 ms, maximum 9.9 ms with 4 (command-count firmware, 2026-10-22). 50 Hz has margin;
  100 Hz only just fits.
- At standstill the slide stays a few 0.1 mm from its target because of friction (more with
  water); `link_test.py` accepts 0.5 mm.
- The slide lags a streamed sine by up to about 2.3 mm at 16 mm/s.
- Homing drives the slide into the 200 mm end stop (Home Position 200 mm), then to the Initial
  Position 100 mm. Positions are only meaningful after homing; the drive loses homing when it
  is powered off and after some errors.
- LinMot-Talk drive configuration (E1200-GP-UC, firmware 6.10). Backups, the seven changes and
  how to restore and check them: `firmware/linmot_serial_bridge/drive_config/`:
  - Minimal / Maximal Position set to 3 / 197 mm, and both checks switched on in the Error
    Detection Mask (they were off, with limits at −57 / 273 mm, the whole motor stroke). The
    drive makes a quick stop if the slide passes them. Homing still works with them on.
  - `/Abort Motion Command` was forced in the Ctrl Word Parameter Force Mask, so the bridge
    could not clear it and the watchdog's abort did nothing. Now not forced; the watchdog
    stops the slide.
  - Quick stop deceleration is 10 m/s²: from 2 m/s that takes 0.2 m. Raising it shortens a
    quick stop at speed but stops harder.
  - The drive has no general velocity or acceleration limit; the following-error limit (warning
    10 mm, error 20 mm) and the 32 A current limit are its only motion checks.
  - While LinMot-Talk's control panel has control, the drive ignores the control word from the
    bus. Release it before running from the Pi.
  - Legacy Sync Watchdog was off (`LSW Enable` Disable, cycle 5000 µs). Without it the drive
    notices nothing when the link goes and holds its last target, switched on (USB test,
    2026-10-22). LinMot's release notes list a fix for this watchdog on the E1200 (the cycle
    time was divided by 100 internally, so it tripped too early); whether firmware 6.10 has the
    fix is not known.
  - With `LSW Enable` on (2026-10-22) the drive stops itself when the SYNC frames stop: error
    CDh, shown in LinMot-Talk as "CanOpen Err. Ctrl Timeout", quick stop, power stage off. It
    also leaves NMT operational (CANopen's default reaction to a communication error), so it
    sends no feedback until the next NMT start; `bring_up()` and `link_test.py` send one.
  - A motion command that arrives in the same frame as the control word ending an abort is
    refused with "Motion Cmd In Wrong State" and a quick stop: the control word has to come
    first, on its own (2026-10-22).
- The drive configuration in use is
  `firmware/linmot_serial_bridge/drive_config/sloshing_2026-10-22.lmc`
  (`FixedConfig.lmc` on the flash drive); the limits, the abort fix and the sync watchdog exist
  only on the drive and in that file. After any change in LinMot-Talk, save a new backup there.
