# Sloshing oscillator

Record of running the LinMot sloshing oscillator from the lab Pi with the same controller,
bridge and web page as the wavemakers (on `master` since 2026-11-02): the rig, the decisions,
what is still open, and what the rig tests found.

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

## Open

Parked until the sloshing rig is connected again (2027, TODO 50). Work for all rigs, such as
go-to-position, is in [TODO.md](TODO.md#planned-features-all-rigs).

- [ ] Pregenerated goals: check the peak acceleration taken from the samples.
- [ ] `play_dat.py` for pregenerated goals (low priority): smooth the data (low-pass) and check
  its acceleration before sending (`--max-acceleration`), so noisy files do not hit the
  controller's acceleration check; report the run and time where a file is too fast.

The plan's steps (A-G, R, merge) are done; their checklists and commands are in
`git show c6bba03:OSCILLATOR.md`. What they found is below, in *Found on the rig*.

## How to run

Commands assume a sourced workspace and a ROS domain other than 0:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=<n>
```

Build and unit tests, with no hardware (the driver's tests and `test_linmot_actuator` run against a simulated Arduino
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

### Firmware (2026-10-22)

The bridge sends a motion command count instead of an empty header and an extra SYNC, with PDO1
last (`d0cf237`), so a repeated command with the same count is the same command.

### Under the controller (step C + D, 2026-10-23, dry)

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
catch up. Margins 1.2 / 5.0 halve the lag but make the slide hum (step G, below), so the yaml
has 1.0 / 1.0. The stroke
stays 0.59 mm short (0.25 mm at the bottom, 0.34 mm at the top) in both runs, with a stall of
about 150 ms at each turnaround, so it is not the braking limit; accepted, the rig is for
demonstrating sloshing. If it ever matters: LinMot-Talk oscilloscope, Demand vs Actual
Position at a turnaround (demand flat → VAI planning, streaming mode; actual behind demand →
friction or the drive's gains). On the PC this also needed the current
`rpi/99-linmot-bridge.rules` for `/dev/linmot_bridge`.

### Commissioning without water (step G, 2026-10-23)

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

### Fault paths (step G, 2026-10-22 – 2026-10-26)


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

### With water (step G, 2026-10-26)

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
