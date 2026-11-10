# Hydrolab rigs

ROS 2 (Jazzy) packages for controlling the Hydrolab rigs: the wavemakers and two oscillators,
the sloshing oscillator (`OSCILLATOR.md`) and the vertical oscillator
([Running the vertical oscillator](#running-the-vertical-oscillator)). On a wavemaker, a lifecycle controller node generates
the paddle motion and sends it to a Rexroth IndraDrive through a Moxa MGate 5101-PBM-MN
Modbus TCP / PROFIBUS gateway.

```
 ROS 2 client ──action/services──▶ controller node ──Modbus TCP──▶ MGate 5101 ──PROFIBUS──▶ IndraDrive ──▶ paddle
 (ros2 CLI, bridge)                (rig_controller)                 (gateway)                (servo drive)
```

> **Safety first.** Do not start physical motion until the checks in
> [COMMISSIONING.md](COMMISSIONING.md) are complete: emergency stop, mechanical travel, actuator
> conversion, software limits, feedback direction, communication, and the physical zero position.
> Before every lab session, go through [STARTUP.md](STARTUP.md). [TODO.md](TODO.md) tracks the open
> software issues; several affect behaviour on the real drive.

## Contents

- [Packages](#packages)
- [Build](#build)
- [Lab computers](#lab-computers)
- [Configuration](#configuration)
- [Running a wavemaker](#running-a-wavemaker)
  - [Water depth](#water-depth)
  - [Web page](#web-page)
- [Running the vertical oscillator](#running-the-vertical-oscillator)
- [Generating motion files](#generating-motion-files)
  - [From test matrix to tank](#from-test-matrix-to-tank)
- [Generating oscillator motion files](#generating-oscillator-motion-files)
- [Interfaces](#interfaces)
- [How a motion runs](#how-a-motion-runs)
- [Faults and stopping](#faults-and-stopping)
- [Choosing the poll interval](#choosing-the-poll-interval)
- [Tests](#tests)
- [Recommended test order on the rig](#recommended-test-order-on-the-rig)

## Packages

| Package | Contents |
|---|---|
| `drivers/mgate5101_driver` | Standalone C++ driver: cyclic Modbus TCP client for the MGate (`MGateDriver`) and the IndraDrive state machine on top of it (`IndraDrive`). Includes `poll_rate_probe` and a monitor/move example. Register mapping is explained in [MAPPING.md](drivers/mgate5101_driver/MAPPING.md). |
| `drivers/linmot_serial_driver` | Standalone C++ driver for the LinMot oscillator: the serial link to the Arduino serial-to-CAN bridge (`SerialBridge`, firmware in `firmware/linmot_serial_bridge`) and the LinMot drive's start-up and faults on top of it (`LinMotDrive`). `linmot_probe` checks the link without moving anything. Not used by the controller yet. |
| `rig_interfaces` | `MoveRig` action, `ReturnToUpright` and `CancelAllGoals` services. |
| `rig_controller` | `controller_node` (the lifecycle controller), the wave and trajectory maths, the IndraDrive actuator adapter, and `bridge_node` (see the note below). |
| `wavemaker_bringup` | Launch file and per-wavemaker configuration (`config/wavemakers.yaml`). |
| `oscillator_bringup` | Launch file and configuration of the oscillators (`config/oscillator.yaml`: `sloshing`, `vertical`); `oscillator:=` picks one. |
| `rig_web` | Web page for running a wavemaker or the oscillator through the bridge (`rig` parameter; see [Web page](#web-page)). |

> **`bridge_node`** connects topic-based clients, such as the web page, to the controller.
> Its topics are `height`, `period`, `start`, `stop` and `heartbeat` in, and `<rig>_state` /
> `<rig>_message` out (`wavemaker_state`, `oscillator_state`, ...).

## Build

Requires ROS 2 Jazzy with `rclcpp`, `rclcpp_lifecycle`, `rclcpp_action`, `bondcpp`, `std_msgs`
and `nav2_lifecycle_manager`.

```bash
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to wavemaker_bringup
source install/setup.bash
```

## Lab computers

Each lab has its own Raspberry Pi, named after the wavemaker in `wavemakers.yaml`: hostname
`<wavemaker>-rpi`, user `<wavemaker>`. Each lab also has its own ROS domain, so the labs do not
see each other's nodes.

| Lab | Hostname | User | ROS domain |
|---|---|---|---|
| ladertanken | `ladertanken-rpi` | `ladertanken` | 1 |

Log in with `ssh ladertanken@ladertanken-rpi.local`. Do not use the old Raspberry Pi default user
`pi`. On a Windows lab computer, the files in [windows/](windows/README.md) start and stop the rigs
with a double-click.

Set the lab's domain on the Pi and on every computer that should talk to it, for example in
`~/.bashrc`: `export ROS_DOMAIN_ID=1`. Do not use the default domain 0: any other ROS user on the
network would see the wavemaker and could command it.

## Configuration

All wavemakers are configured in
[`wavemaker_bringup/config/wavemakers.yaml`](wavemaker_bringup/config/wavemakers.yaml), one block
per wavemaker (`ladertanken`, `lilletanken`, `mclab`). Check every value against the real rig
before connecting to hardware. Changes take effect at the next `configure`.

### Wavemaker and geometry

| Parameter | Meaning |
|---|---|
| `rig` | `wavemaker` (default) or `oscillator`. Names the action and topics: `move_<rig>`, `<rig>_setpoint`, `<rig>_position`, `<rig>_velocity`, `<rig>_actuator_setpoint`, and the bridge's `<rig>_state` / `<rig>_message`. Set the same value on the controller, bridge and web page. `oscillator` also selects the trajectory: `height` is the stroke (amplitude = height / 2 around `upright_position_m`), with no wave theory, water depth or transfer gain, and the wavemaker-only checks (`wavemaker_type`, `water_depth`, flap, gain table) are skipped. See [OSCILLATOR.md](OSCILLATOR.md). |
| `drive_name` | Required, no default. `indradrive` (Rexroth IndraDrive behind an MGate gateway: the wavemakers and the vertical oscillator) or `linmot` (Arduino serial-to-CAN bridge: the sloshing oscillator). Any rig can use either. Configure fails when it is missing or unknown. A new drive is an `Actuator` subclass plus one branch in `make_actuator()` (`rig_controller/src/controller_node.cpp`). |
| `wavemaker_type` | `piston` or `flap`. Selects the wave transfer function. |
| `water_depth` | Water depth in m. Can be overridden for one run with the `water_depth` launch argument (see [Water depth](#water-depth)). |
| `flap_attachment_height` | Flap only: height of the actuator attachment above the tank floor, m. Must be greater than `water_depth`. |
| `hinge_height` | Flap only: height of the flap's hinge above the tank floor, m (default 0.0). Must be below `water_depth`. Used in the wave transfer function (Dean & Dalrymple) and to scale the stroke from the waterline to the attachment point. |
| `upright_position_m` | Paddle position at upright (the mean position), m. Returns go here; waves oscillate around it. |
| `upright_is_minimum` | `true`: the wave runs from upright to one side only. `false`: symmetric around upright. |
| `wavemaker_transfer_gain` | Measured / target wave height for regular waves (default 1.0 = linear theory). The stroke is divided by it; to correct from wave-probe data, new gain = old gain × measured / target. Not applied to pregenerated waves. Can be overridden for one run with the launch argument of the same name. |

### Actuator and limits

| Parameter | Meaning |
|---|---|
| `drive_motion` | `angular` (drive position in degrees, converted with the lead) or `linear` (drive position in its own length unit, converted with `drive_m_per_unit`). The conversion is `DriveScale` (`rig_controller/src/drive_scale.hpp`), shared by the controller's limits, the actuators and `rig_limits`. |
| `drive_m_per_unit` | Linear only: metres per unit of the drive's position tag (default 1.0). The vertical oscillator: 0.001 (the tag is in mm). lilletanken and mclab keep the placeholder 1.0 until their drives are read (TODO 19). |
| `drive_velocity_per_unit_per_s` | Velocity tag units per (position tag unit per second): converts the speed cap sent with each setpoint, and `max_velocity_rpm` back to m/s. Default 1/6, the deg/s → rpm rule of the angular drives. The vertical oscillator: 6.0 (1 mm/s = 6 × 10 mm/min). |
| `drive_direction` | `1` or `-1` (an integer): the sign between the rig (+ = up or forward) and the drive. Applies to setpoints, feedback and the drive limits; the speed cap stays positive. Default 1. The vertical oscillator: -1 (a + command moves its slide down). |
| `actuator_lead_m_per_degree` | Angular only: paddle travel per drive degree. |
| `actuator_upright_angle_deg` | Angular only: drive angle at upright. |
| `min_position_deg` / `max_position_deg` | Angular travel limits in drive degrees. Goals that leave them are rejected; the drive also refuses targets outside them. |
| `travel_minimum_m` / `travel_maximum_m` | Linear travel limits in m around the rig's zero. Goals that leave them are rejected; an IndraDrive's drive-side limits are derived from them (one set of limits). |
| `min_position_m` / `max_position_m` | LinMot only: the bridge firmware's limits (5…195 mm). |
| `max_velocity_rpm` | IndraDrive speed limit, in the drive's velocity tag units: rpm on the angular drives, 10 mm/min on the vertical oscillator (600 = 0.1 m/s). Goals whose peak speed exceeds it are rejected. Required by `play_dat` and the generators (`rig_limits`), which derive the speed in m/s from it. |
| `hold_velocity_rpm` | Speed cap used when stopping and holding position, in the same units (default 10, ≤ `max_velocity_rpm`). |

### Ladertanken: position resolution

The setpoint is not rounded to a coarse step anywhere on the way to the drive:

- Goals carry positions as 64-bit floats in metres (a `.dat` file stores 32-bit floats, about
  0.01 µm at full travel), and the controller converts them to drive degrees in 64-bit floats
  (÷ 0.00053 m/deg).
- The drive's target position register is a 32-bit integer in steps of 0.0001°, and the setpoint
  is rounded to the nearest step (`ProcessImage::set_scaled`). One step is **0.053 µm** at the
  attachment point (about 0.04 µm at the waterline). The register reaches ±214 748°; the travel
  limits use ±275°. The speed cap sent with each setpoint has steps of 0.0001 rpm.
- What limits small motions is the flap's standstill wobble, about ±0.5° at the drive (±0.27 mm
  at the attachment; COMMISSIONING.md, *Verified so far*), some 5000 steps. From about 5 mm paddle
  amplitude (H about 1 cm at 1.0 s) it is a few percent of the motion at most; at 1-2 mm it
  becomes a large part of it.
- The control loop runs every `poll_interval_ms` (20 ms); motion files may use any whole number of
  milliseconds, and the controller interpolates between their samples.

The driver casts the setpoint to 32 bits without a range check; the controller's clamp to the
travel limits keeps it far inside the register's range.

### Communication

| Parameter | Meaning |
|---|---|
| `driver_address`, `driver_port`, `driver_unit_id` | MGate IP address, Modbus port and unit ID. |
| `input_base_reg` / `input_word_count`, `output_base_reg` / `output_word_count` | Process-data areas in the MGate (see [MAPPING.md](drivers/mgate5101_driver/MAPPING.md)). |
| `status_base_reg` / `status_word_count` | Gateway status word and PROFIBUS live list (768 / 9). |
| `profibus_address` | The drive's PROFIBUS address, used to check that it is live. |
| `poll_interval_ms` | Gateway exchange period. **The control loop runs at this rate.** See [Choosing the poll interval](#choosing-the-poll-interval). |
| `write_only_dirty` | Default `false`: outputs are written every cycle (needed for the MGate output fault timeout). Keep it `false`. |
| `timeout_ms`, `reconnect_ms`, `step_timeout_ms` | Modbus timeout, reconnect delay, and timeout per drive state-machine step. |

### Motion tuning (defaults apply if not set)

| Parameter | Default | Meaning |
|---|---|---|
| `return_to_upright_max_velocity_mps` | 0.1 | Peak speed of a return to upright and of the blend at the start of a pregenerated goal; also bounds the start blend's own speed for a regular wave. Must not exceed the drive's limit. |
| `return_to_upright_min_duration_s` | 1.0 | Shortest return or blend. |
| `return_to_upright_default_tolerance_m` | 0.002 | Tolerance for "at upright"; also the margin allowed outside the travel limits at the start of a motion. |
| `return_to_upright_timeout_margin_s` | 2.0 | Extra time after the planned return before it times out. |
| `positioning_velocity_margin` | 1.2 | The drive's speed cap is the motion's peak speed × this. Raise it if the paddle lags at its fastest point; lower it if it moves in steps. |
| `positioning_acceleration_margin` | 1.2 | The same for the acceleration: the drive's acceleration cap is the motion's peak acceleration × this (start blend included), at least 0.001 m/s² and at most the drive's maximum. Only the LinMot drive uses it; 1.0 for the oscillator (the sine's maxima). |
| `max_goal_acceleration_mps2` | 0 (off) | Goals whose peak acceleration (start blend included) is above this are rejected. 9.81 for the oscillator. The start blend from rest reaches about 11 % more than the steady sine. |
| `min_period_s` | 0 (off) | Regular goals with a shorter period are rejected. 0.2 (5 Hz) for the oscillator. |

## Running a wavemaker

There are two ways to run a wavemaker. Both read `config/wavemakers.yaml` and take
`wavemaker:=ladertanken|lilletanken|mclab` (default `ladertanken`).

| | Terminal mode (experienced users) | Bridge mode (normal users) |
|---|---|---|
| Launch file | `wavemakers.launch.py` | `bridge.launch.py` |
| Nodes | `controller` | `controller` and `bridge` |
| Lifecycle | Stepped by hand (lifecycle manager optional, off by default) | Configured and activated automatically by the Nav2 lifecycle manager |
| Control | `ros2 action` / `ros2 service` (see [Interfaces](#interfaces)) | The bridge's topics: `height`, `period`, then `start`; `stop`; a `heartbeat` from the client (the web page) |
| If a node dies | Nothing stops the controller | The lifecycle manager brings both nodes down after `bond_timeout` (default 1 s), which disables the drive |

> Bridge mode with the web page runs on the lab Pi (2026-10-12). For first tests on a new rig,
> use terminal mode.

### Terminal mode

```bash
ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken
```

The controller starts unconfigured. Step through the lifecycle by hand; replace `ladertanken`
with the selected wavemaker.

```bash
ros2 lifecycle set /wavemakers/ladertanken/controller configure   # reads parameters, no connection yet
ros2 lifecycle set /wavemakers/ladertanken/controller activate    # starts the gateway link, enables the drive
ros2 lifecycle get /wavemakers/ladertanken/controller

ros2 lifecycle set /wavemakers/ladertanken/controller deactivate  # stops motion, disables the drive
ros2 lifecycle set /wavemakers/ladertanken/controller cleanup     # frees everything
```

After a driver fault the controller ends up **unconfigured** and must be configured and activated
again (see [Faults and stopping](#faults-and-stopping)). To let the lifecycle manager do the
configure and activate steps, add `enable_lifecycle_manager:=true`; only do this after manual
lifecycle testing.

### Bridge mode

```bash
ros2 launch wavemaker_bringup bridge.launch.py wavemaker:=ladertanken
```

Both nodes are configured and activated as soon as they start, so the drive is enabled
straight away. If either node stops sending bond heartbeats for `bond_timeout` seconds (a crash,
a hang, or the bridge being killed), the lifecycle manager brings both down and the paddle
stops. Set `bond_timeout:=<seconds>` to change this; too short a value can trigger on a briefly
busy PC.

Launch arguments of `wavemakers.launch.py`: `wavemaker`, `with_bridge` (default `false`),
`enable_lifecycle_manager` (default `false`) and `bond_timeout` (default `4.0`, passed to the
lifecycle manager and to both nodes), `water_depth` (see below) and `wavemaker_transfer_gain`.
The last two override the values in `wavemakers.yaml` for that run; empty (the default) keeps the
yaml value.
`bridge.launch.py` sets the first three for you and defaults `bond_timeout` to `1.0`.
`lab.launch.py` passes on `water_depth` and `wavemaker_transfer_gain`.

Every launch takes both goal types: each goal picks its own (see *Action* below), so playing a
`.dat` file with `play_dat` needs no relaunch. `mode_pregenerated` was removed on 2026-10-30; a
yaml file or launch command that still sets it makes configure fail with "was removed".

```bash
# terminal mode with a measured gain
ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken \
  wavemaker_transfer_gain:=0.92
```

### Water depth

The water depth changes with the tank fill, and the controller uses it to compute every wave.
Measure it before a session and pass it at launch; it overrides `water_depth` in
`wavemakers.yaml` for that run only. On a lab Pi running the service, set `WATER_DEPTH` in
`/etc/default/wavemaker` instead (`sudoedit`), then stop and start the service
([rpi/README.md](rpi/README.md)). All three launch files take it:

```bash
# lab Pi: tank filled to 0.95 m
ros2 launch wavemaker_bringup lab.launch.py wavemaker:=ladertanken water_depth:=0.95

# the same for bridge mode or terminal mode
ros2 launch wavemaker_bringup bridge.launch.py wavemaker:=ladertanken water_depth:=0.95
ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken water_depth:=0.95
```

Check what the controller uses:

```bash
ros2 param get /wavemakers/ladertanken/controller water_depth
```

Without the argument the value in `wavemakers.yaml` is used. Change the yaml (on the PC, then
commit and pull on the Pi) only when the usual depth changes; editing it on the Pi makes the next
`git pull` refuse. For a flap the depth must stay below `flap_attachment_height` and above
`hinge_height`, or configure fails. Readjust the spring balance for the new depth ([COMMISSIONING.md](COMMISSIONING.md) item 34).

### Web page

`rig_web` is a page for running a wavemaker from any browser on the lab network. Start bridge mode,
then the page server for the same wavemaker:

```bash
ros2 launch wavemaker_bringup bridge.launch.py wavemaker:=ladertanken
ros2 launch rig_web web.launch.py wavemaker:=ladertanken port:=8080
```

Or both in one command, together with the soft stop node:

```bash
ros2 launch wavemaker_bringup lab.launch.py wavemaker:=ladertanken port:=8080
```

The drive is enabled as soon as this runs. On a lab Pi it runs as the `wavemaker@<wavemaker>`
systemd service, so a dropped SSH session does not stop it; see [rpi/README.md](rpi/README.md).

Open the page by the Pi's hostname, `http://<wavemaker>-rpi.local:8080/` (for ladertanken
`http://ladertanken-rpi.local:8080/`); the server prints this address when it starts. The page has
height and period inputs, Start and Stop, the bridge's state and
messages, a live plot of setpoint and measured position, Return to upright, and a log.

- The log shows what every node in the wavemaker's namespace (controller, bridge, lifecycle
  manager, page server) logs, read from `/rosout`, with a level filter. It does not show output
  that is not a ROS log call, such as `ros2 launch`'s "process has died" or a crash trace; see the
  terminal or `journalctl` for those.

- The server publishes the bridge's `height`, `period`, `start` and `stop` topics and its
  `heartbeat`, which it sends only while an open page keeps acknowledging the server's updates.
  Closing the page, a frozen browser or a lost network stops the wave within about 1.5 s.
- Any open page keeps the wave running, so with two pages open, closing one does not stop it.
- After every start the server checks the height and period the bridge reports in its
  "Starting wave" message, and stops the wave if they differ from what was sent: the three
  topics are separate, and DDS does not guarantee their order.
- The server listens on all interfaces (`host` parameter, default `0.0.0.0`) without a login:
  anyone who can reach the port can run the wavemaker. Keep it on the lab network.
- Only Python's standard library and rclpy are needed, so it also runs on a Raspberry Pi.
- **Idle auto-stop.** After 2 h without motion, the server shuts the bridge and controller
  down through the lifecycle manager (which disables the drive) and exits, and the launch shuts
  down with it. On a lab Pi the service then ends; start it again to continue. Motion is anything the
  controller drives: a wave from the page, a `play_dat` goal or a return to upright (any setpoint
  it publishes); an open page or other clicks do not count. For the last 10 minutes the page shows a
  warning with the stop time and a **Keep running** button, which restarts the 2 h. Set the time
  with the `idle_timeout_s` launch argument of `lab.launch.py` or `web.launch.py` (0 turns it
  off).
- If the server exits or crashes for any other reason, the launch shuts down as well.
- The `rig` launch argument of `web.launch.py` (`wavemaker`, the default, or `oscillator`) picks the
  page's texts and the server's messages, for example "Paddle" and "Return to upright" or "Slide"
  and "Go to centre". The controls are the same.
- The `namespace` launch argument of `web.launch.py` sets where the page runs and which bridge it
  talks to. Empty (the default) gives `wavemakers/<wavemaker>` for `rig:=wavemaker`, where
  `wavemaker` is then required, and `oscillators/sloshing` for `rig:=oscillator`, so the
  oscillator needs only `rig:=oscillator`.

To try the page without hardware, `demo.launch.py` starts a fake controller, the bridge, the
lifecycle manager and the page on their own ROS domain (77), reachable only from this computer,
so the demo cannot reach other computers or the real controller. Open the address the server prints,
`http://<hostname>.local:8080/`:

```bash
ros2 launch rig_web demo.launch.py
ros2 launch rig_web demo.launch.py idle_timeout_s:=120   # try the idle auto-stop
```

## Running the vertical oscillator

A second Rexroth IndraDrive (rotary motor with linear scaling, 20 mm/rev) moves a slide up and
down. Until the daisy-chain cable arrives it takes the ladertanken wavemaker's place on the
ladertanken MGate: the same PROFIBUS address (2), the same gateway blocks, the cable replugged
between the two drives. Only one of the two rigs runs at a time. The plan, decisions and
commissioning log are in [oscillator_sketch.md](oscillator_sketch.md).

**Configuration:** the `vertical` blocks in
[`oscillator_bringup/config/oscillator.yaml`](oscillator_bringup/config/oscillator.yaml)
(`/oscillators/vertical/controller` and `/oscillators/vertical/bridge`): `rig: oscillator`,
`drive_name: indradrive`, `drive_motion: linear` with the drive's units
(`drive_m_per_unit: 0.001`, `drive_velocity_per_unit_per_s: 6.0`, `drive_direction: -1`, see
[Actuator and limits](#actuator-and-limits)), travel −0.2…0.2 m around upright 0, speed limit
`max_velocity_rpm: 600` (0.1 m/s).

**Start and stop** on the ladertanken Pi, through the oscillator service with
`OSCILLATOR=vertical` in `/etc/default/oscillator` ([rpi/README.md](rpi/README.md#oscillator)):

```bash
sudo systemctl start oscillator@ladertanken     # switches the drive on; it does not home
sudo systemctl stop oscillator@ladertanken      # stops the motion, drive off, brake holds
```

By hand: `ros2 launch oscillator_bringup oscillator.launch.py oscillator:=vertical` (default
`sloshing`). The nodes run in `/oscillators/vertical`.

**Switching between the wavemaker and the oscillator** means replugging the PROFIBUS cable:
stop the running service first, replug, identify the drive, then start the matching service
([COMMISSIONING.md](COMMISSIONING.md#vertical-oscillator-ladertanken), *Switching drives*).
Nothing in the cyclic data tells the controller which drive is plugged in, so the wavemaker's
configuration on the oscillator drive would read millimetres as flap degrees.

**Running:** the web page (`http://ladertanken-rpi.local:8080/`): *height* is the stroke, peak to
peak, in metres, around mid-travel; + is up. **Stop** cancels the wave and the slide holds where it
is, drive enabled; **Return to upright** brings it back to mid-travel. Motion files are made with
`generate_oscillations` ([Generating oscillator motion files](#generating-oscillator-motion-files))
and play with `play_dat <file> vertical --rig oscillator`; the speed limit comes from the yaml.

**Tracking (2026-11-09, no load, no water):** the slide follows the setpoint with about 180 ms of
lag; the stroke reaches 97 % at a 2 s period and 90 % at 1 s, for any stroke. It is not tuned yet
and depends on the load (TODO item 46). Record `oscillator_position` with every run.

## Generating motion files

> **Regular and irregular waves played on the rig** (ladertanken, 2026-11-04 and 2026-11-05,
> [rpi/README.md](rpi/README.md)). The wave heights have not yet been measured with probes
> ([TODO.md](TODO.md), item 43).

`generate_waves` (`rig_controller/src/generate_waves.py`) turns a test matrix into a motion file for one wavemaker,
ready for `play_dat`. Wavemakers only; the oscillators have `generate_oscillations`
([Generating oscillator motion files](#generating-oscillator-motion-files)).

### From test matrix to tank

The whole session, in order. Each step links to the details. Work on the Pi: its
`wavemakers.yaml` (gain table, depth, geometry) is the one the controller runs with. The commands
are for ladertanken and the session of 2026-11-05; use your own wavemaker and date.

**1. Log in and start tmux.** On the lab PC (set up once as in
[windows/README.md](windows/README.md)), open a Command Prompt:

```bat
ssh ladertanken@ladertanken-rpi.local
tmux                # or `tmux attach` to get back to an earlier one
```

The commands on the Pi run inside tmux, so a dropped connection does not stop a run: log in
again and `tmux attach`. Steps 5, 7 and 9 happen on the lab PC, in a second window. If `tmux` says *sessions should be nested with care*, you are already in one;
carry on.

**2. Set up the shell** (in every new tmux window):

```bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=1        # the lab's domain; without it play_dat finds no controller
mkdir -p ~/wave_sessions/2026-11-05
cd ~/wave_sessions/2026-11-05
```

One folder per session, named after the date. It ends up holding the matrix, the `.dat`, the
run table and the plot, all of which belong with the probe data.

**3. Write the test matrix** `session.csv` in that folder: see [Test matrix](#test-matrix) for
the columns and [Writing the test matrix](#writing-the-test-matrix) for how. To write it on the
PC instead, copy it over from there (the folder from step 2 must exist):

```bash
scp session.csv ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05/
```

**4. Generate** (details in [Generate](#generate)). Add `--water-depth` if the tank is not at
the yaml's depth:

```bash
ros2 run rig_controller generate_waves ladertanken session.csv -o session_waves
```

It prints how many runs it made and how many it skipped. A skipped run does not fit the
wavemaker; the reason is in `session_waves.csv`.

**5. Look at the plot.** The Pi has no screen. On the computer you sit at, in a second window:

```bat
scp ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05/session_waves.png .
start session_waves.png
```

(`xdg-open` instead of `start` on Linux. With VS Code's Remote-SSH extension, open the Pi's folder
and the plot shows in a tab.) Check that the runs and pauses look as intended, and that position
and speed stay inside the dashed limits.

**6. Dry run**, back in tmux. This reads the file and moves nothing:

```bash
ros2 run rig_controller play_dat session_waves.dat ladertanken --dry-run
```

Every run must say `OK`. Compare `max|x|` and `speed` with what you expect. Runs close to the
speed limit in the first line are the first to watch in the tank.

**7. Start the wavemaker** if it is not running: on the lab PC, double-click
`start_wavemaker.bat` ([windows/README.md](windows/README.md)). It stops the oscillator if that
runs, starts the wavemaker (which enables the drive) and opens the web page. Keep the page open
while playing: it plots the paddle, shows the soft stop, and has Stop and Return to upright.
Its *Log* shows the controller coming up. The web page's idle auto-stop shuts the wavemaker
down after 2 h without motion, so start it close to playing.

**8. Play**, in tmux. The tank must be calm and the soft stop released:

```bash
ros2 run rig_controller play_dat session_waves.dat ladertanken
```

If it stops with `no move_wavemaker server; is the controller up and active?` while the page
shows the controller running, `ROS_DOMAIN_ID` is not set in this tmux window (step 2).

It plays the runs in order, with the pauses between them. `--runs 1` or `--runs 0-1` plays only
some runs (numbers as in the dry run). It warns if the running controller's limits differ from
the yaml's; then stop and find out why. **To stop:** press Ctrl-C in the tmux window, or use
Stop on the web page, the soft stop or the e-stop. A stop cancels the run and ends `play_dat`.
The paddle stops where it is and holds position with the drive enabled; *Return to upright* on
the web page brings it back.

**9. Afterwards.** If nothing else follows, double-click `stop_rigs.bat` on the lab PC; it stops
the wavemaker and disables the drive. Then copy the session folder off the Pi, in a Command
Prompt on the lab PC:

```bat
scp -r ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05 .
```

### Test matrix

A CSV, one row per run, played top to bottom; a repeat is just another row. Each row fills the
columns of its `kind` and leaves the others empty:

```csv
kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,1.0,,1/60,30,,,,,,3,30
regular,1.2,,1/40,30,,,,,,3,30
regular,1.3,0.171,,30,,,,,,3,30
irregular,,,,,0.04,1.2,3.3,300,1,5,60
```

| Column | Kind | Meaning |
|---|---|---|
| `kind` | both | `regular` (default if empty) or `irregular` |
| `period_s` | regular | Wave period T [s] |
| `height_m` or `steepness` | regular | Wave height H [m], **or** the steepness as `1/x` (H/λ = 1/x, e.g. `1/60`); exactly one. Other forms (`60`, `0.0167`) are refused, so they cannot be misread |
| `n_periods` | regular | Length of the run in periods, ramps included |
| `hs_m`, `tp_s` | irregular | Significant wave height Hs [m] and peak period Tp [s] |
| `gamma` | irregular | JONSWAP peak enhancement; empty = 3.3, 1.0 = Pierson–Moskowitz |
| `duration_s` | irregular | Length of the run [s], ramps included; whole samples (any whole second at 20 ms) |
| `seed` | irregular | Random phases; the same seed gives the identical run, another seed another realisation |
| `n_ramp` | both | Ramp at each end, in periods (regular) or peak periods (irregular) |
| `pause_s` | both | Rest after the run, at least 1 s; in practice long enough for the tank to calm |

With steepness rows, H follows from the wavelength, so it depends on the water depth: generate
with the depth of the session (`--water-depth`).

### Writing the test matrix

The matrix is a plain text file. Three ways to make one; pick whichever you are used to. Each
session gets its own folder, named after the date.

**A. On the Pi, by pasting.** Log in (`ssh ladertanken@ladertanken-rpi.local`), then paste the
whole block below into the terminal in one go (right-click pastes in most terminals), with your
own rows between the header line and `END`:

```bash
mkdir -p ~/wave_sessions/2026-11-05
cd ~/wave_sessions/2026-11-05
cat > session.csv <<'END'
kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,1.0,,1/60,30,,,,,,3,60
regular,1.3,0.171,,30,,,,,,3,60
irregular,,,,,0.03,1.2,3.3,120,1,5,60
END
```

The terminal shows `>` at the start of each line while you paste; that is normal. The file is
written when the `END` line arrives. To change a row afterwards, use B.

**B. On the Pi, in an editor.** In the session folder:

```bash
nano session.csv
```

Type or paste the rows, using the arrow keys to move. Save with **Ctrl+O** then **Enter**, and
quit with **Ctrl+X**. If nano asks "Save modified buffer?" on quitting, answer **Y** then
**Enter**.

**C. On the lab PC (Windows), then copy it.** Open **Notepad**, type the rows, and save with *File
→ Save as*: *Save as type* *All files*, file name `session.csv`. Copy it to the Pi as in step 3 of
[From test matrix to tank](#from-test-matrix-to-tank). Excel also works if you save as *CSV UTF-8 (comma delimited)*, but on a PC with
Norwegian settings Excel writes semicolons and decimal commas (`1;0`), which `generate_waves`
cannot read: it then stops with `row 0: n_ramp is required`. Notepad avoids that.

**Check the file** before generating. On the Pi:

```bash
cat session.csv
```

Every line must have the same number of commas as the header (11), and decimals with a point
(`1.0`, not `1,0`). A mistake in a row makes `generate_waves`
stop with the row number (`row 0` is the first row after the header) and what is wrong; no
files are written then.

### Generate

```bash
cd ~/wave_sessions/2026-11-05    # the session folder, with the test matrix
ros2 run rig_controller generate_waves ladertanken session.csv -o session_waves
```

File names are relative to the current folder, for the test matrix and for `-o`; from elsewhere
give full paths (`~/wave_sessions/2026-11-05/session.csv -o ~/wave_sessions/2026-11-05/session_waves`). It writes:

- `session_waves.dat`: the motion file for `play_dat`.
- `session_waves.csv`: the run table: per run its start and end, the parameters, H and 1/x
  (regular), the paddle amplitude or its standard deviation and peak (irregular), and `skipped`
  with the reason for a run that does not fit the wavemaker.
- `session_waves.png`: paddle position and speed over the whole file, with the wavemaker's
  limits dashed and the runs numbered as in the run table.
- `session_waves.txt`: how the file was made (date, command, commit, yaml, water depth, dt, speed
  limit) and a copy of the test matrix. Keep it with the `.dat`: it is what the probe data is
  read against.

and prints a summary and the `play_dat` command to check the file:

```
session_waves.dat: 4 runs, 0 skipped, 9.3 min
ros2 run rig_controller play_dat session_waves.dat ladertanken --dry-run
```

For a linear drive the command also carries `--max-velocity-mps`, and `--config` whenever one was given.

**Give `-o` a different name from the test matrix:** `-o session` would overwrite `session.csv`
with the run table.

| Option | Use |
|---|---|
| `--water-depth 0.95` | The depth of this session instead of the yaml's; warns when the gain table was measured at another depth |
| `--dt 0.01` | Sample interval [s], whole milliseconds; default 0.02 |
| `--show` | Also open the plot in a window |
| `--no-plot` | Do not write the `.png` |
| `--config <file>` | Another `wavemakers.yaml`; default the installed one, else the repository's when running from the source tree |
| `--max-velocity-mps` | Override the speed limit from the yaml (`max_velocity_rpm` in the drive's units); the printed `play_dat` command then carries the same value |

### Check and play

Look at the `.png`, then let `play_dat` read the file without moving anything. Source the
workspace first; the file name is relative to the current folder:

```bash
source ~/ros_ws/install/setup.bash
ros2 run rig_controller play_dat session_waves.dat ladertanken --dry-run
```

The name gives the limits, the upright position and the namespace from the yaml, and every run
prints where its limits came from. To play it, drop `--dry-run`, with the controller running (the
wavemaker service or `lab.launch.py`); `play_dat` then warns if the running controller's limits
differ from the yaml's. `--runs 2-3` plays only some runs. `--min-m`, `--max-m`,
`--max-velocity-mps` and `--upright-m` override the yaml; without a name, give the first three.
See `play_dat --help`.

### Compatibility

- **With `play_dat`:** the file and run table are in the format `play_dat` reads, and the
  generator reads every file back with `play_dat`'s own functions before it finishes.
- **With the controller:** the wave physics is the controller's own (dispersion relation,
  transfer function, transfer gain, flap scaling to the attachment point), so a run in a file and
  the same H and T sent as a regular goal give the same stroke. Each run is checked against the
  controller's position, speed and acceleration limits for that wavemaker; a run that does not fit
  is listed in the run table as `skipped`, with the reason, and left out of the file.
- **With `wavemakers.yaml`:** the geometry and limits are read from the wavemaker's block. The
  type, water depth, geometry and limits must be stated there; the generator refuses to fall back
  on the controller's generic defaults for them.
- **Trigger:** channel 1 of the file is 1 during each run and 0 at rest. Nothing plays it yet
  ([TODO.md](TODO.md), *Trigger output*).

### Making your own motion file

Use `generate_waves` where you can: it applies the wavemaker's geometry, gain table and limits for
you. Make your own file only for motion it cannot generate, and then follow these rules. The
controller plays the numbers as they are: it adds no wave theory, gain or depth to a file, and
only refuses motion outside the limits. A wrong number makes a wrong wave, without any warning.

**What a position is.** Metres (not mm), at the **actuator attachment point** on the flap (the
actuator arm), relative to upright, with the gain already applied. Not the stroke at the
waterline (z = 0). From linear wavemaker theory, for a flap:

```
x_attachment = (S / 2) / gain × (attachment_height − hinge_height) / (water_depth − hinge_height)
```

where S is the stroke at the waterline for wave height H (S = H / transfer function, with the
hinge at `hinge_height`), and the gain, heights and depth are from the wavemaker's block in
`wavemakers.yaml`. For ladertanken (attachment 1.30 m, hinge 0.09 m) the last factor is 1.33 at
1.0 m depth and 1.36 at 0.98 m, and the gain is the table for the period (0.64-0.80). A piston
moves the same at every height: x = (S / 2) / gain. A file is only right for the depth it was
made for; note the depth with it.

**Limits** (ladertanken; others in their yaml block): positions within ±0.146 m and speed below
0.763 m/s. `play_dat --dry-run` checks every run against them; a file in mm is refused there,
as its travel is 1000 times too large.

**Layout:** little-endian; `int32` time step in whole milliseconds (10 or 20
ms), `int32` number of channels, then `float32` frames, all channels of step 0, then of step 1,
and so on. Channel 0 is the position; channel 1 the trigger (optional, not played yet); further
channels are ignored.

**Runs and rests.** Each run starts and ends at 0 (within 2 mm), with a ramp of a few periods at
each end. Between runs the position must be exactly 0 for at least 1 s; that is how `play_dat`
splits the file into runs. A `<file>.csv` next to the `.dat` with the columns `run`, `start_s`
and `end_s` names the runs and their boundaries; without it they are found from the rests.

**Check before the tank:** `play_dat <file> <wavemaker> --dry-run` lists each run with its period,
largest position and peak speed, and says why any run would be refused. Compare the largest
position with what you computed.

### Irregular waves (decided 2026-11-03)

| Decision | Choice | Why |
|---|---|---|
| Spectrum | JONSWAP with Hs, Tp, γ (γ = 1 is Pierson–Moskowitz) | The standard in model testing; one formula covers both |
| Amplitudes | Fixed by the spectrum, random phases only | The generated Hs is exact, and a seed reproduces the run exactly |
| Reproducibility | A `seed` per run in the test matrix and the run table | Re-running a sea state must give the same motion as the probe data |
| Frequency spacing | Δf = 1 / duration, synthesised by inverse FFT | No repetition within a run; fast for 30-minute runs |
| Frequency band | 0.5 fp to 3 fp (`F_MIN_FACTOR`, `F_MAX_FACTOR` in the script) | Below it the paddle needs a large stroke for little wave; above it speed and acceleration grow |
| Hs | Scaled over the generated band only | The generated waves have exactly the requested Hs; the cut tails carry no energy |
| Transfer function | The regular-wave paddle amplitude per frequency | Same physics and gain as regular waves, nothing new to keep in step |
| Gain table | Applied per frequency, clamped outside the table | Calibrated with regular waves, so an approximation for irregular seas |
| Second-order correction | Not included | Needs further theory; regular waves do not have it either |

The peaks of an irregular run depend on the seed: a sea state that fits with one seed can be
skipped with another, and the run table gives the paddle's peak for each.

### Tests

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test/test_generate_waves.py -v
```

## Generating oscillator motion files

`generate_oscillations` (`rig_controller/src/generate_oscillations.py`) is the oscillators'
`generate_waves`: a test matrix in, the same four files out, every run checked against the
oscillator's limits and the file read back with `play_dat`. It works for both oscillators in
`oscillator.yaml` (`vertical`, `sloshing`). The input is the slide's **stroke**, peak to peak, in
metres; there is no wave physics, no gain and no water depth.

### Test matrix

One row per run, played in file order; each row fills its kind's columns and leaves the others
empty. Every line has the same number of commas as the header (10), decimals with a point.

```csv
kind,period_s,stroke_m,n_periods,sig_stroke_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
regular,2.0,0.020,10,,,,,,2,5
regular,1.0,0.010,10,,,,,,2,5
irregular,,,,0.010,1.5,3.3,120,1,3,10
```

| Column | Kind | Meaning |
|---|---|---|
| `kind` | both | `regular` (default when empty) or `irregular` |
| `period_s` | regular | Period [s] |
| `stroke_m` | regular | Stroke, peak to peak [m]; the amplitude is half of it |
| `n_periods` | regular | Length in periods, ramps included; at least `2 × n_ramp` |
| `sig_stroke_m` | irregular | Significant stroke: 4σ of the position [m], as Hs is 4σ of the water surface |
| `tp_s` | irregular | Peak period [s] |
| `gamma` | irregular | JONSWAP peak enhancement; empty = 3.3, 1.0 = Pierson–Moskowitz |
| `duration_s` | irregular | Length [s], ramps included; at least `2 × n_ramp × tp_s` |
| `seed` | irregular | Random phases; the same seed gives the same run |
| `n_ramp` | both | Ramp at each end, in periods (regular) or peak periods (irregular); at least 1 |
| `pause_s` | both | Rest after the run [s], at least 1 |

Irregular runs have the JONSWAP **shape** for the slide's motion, in the band 0.5 fp to 3 fp,
with random phases (as the wavemaker's irregular waves, without the wave-to-paddle step). The
ramps are **cosine** ramps: a linear ramp steps the acceleration where it ends, which shows as a
jerk spike.

A mistake in a row stops the program with the row number (`row 0` is the first row after the
header, the same as the run number) and the column; no files are written then.

### Generate

```bash
source ~/ros_ws/install/setup.bash
cd ~/oscillator_sessions/2026-11-10    # the session folder, with the test matrix
ros2 run rig_controller generate_oscillations vertical session.csv -o session_motion
```

It writes:

- `session_motion.dat`: the motion file for `play_dat`.
- `session_motion.csv`: the run table: per run its start and end, the parameters, the amplitude
  (regular) or the standard deviation and peak (irregular), `peak_jerk_mps3`, and `skipped` for a
  run that does not fit the oscillator.
- `session_motion.png`: position and speed over the whole file, the limits dashed.
- `session_motion.txt`: how the file was made (date, command, commit, yaml, oscillator, jerk
  limit, dt, speed limit) and a copy of the test matrix. Keep it with the `.dat` and the measured
  motion.

and prints a summary and the command to check the file:

```
session_motion.dat: 3 runs, 0 skipped, 2.9 min
ros2 run rig_controller play_dat session_motion.dat vertical --rig oscillator --dry-run
```

**Give `-o` a different name from the test matrix:** `-o session` would overwrite `session.csv`.

| Option | Use |
|---|---|
| `--max-jerk-mps3 3.0` | Warn above this peak jerk [m/s³]; default 3.0, the vertical IndraDrive's jerk limit (S-0-0193) |
| `--max-velocity-mps` | Override the speed limit from the yaml; the printed `play_dat` command then carries the same value |
| `--dt 0.01` | Sample interval [s], whole milliseconds; default 0.02 |
| `--show` / `--no-plot` | Also open the plot in a window / do not write the `.png` |
| `--config <file>` | Another `oscillator.yaml`; default the installed one |

**Limits** come from `oscillator.yaml` through `rig_limits`, as `play_dat` reads them:

| Oscillator | Travel (around upright) | Speed | Acceleration |
|---|---|---|---|
| `vertical` | −0.2…+0.2 m (upright 0) | 0.1 m/s (`max_velocity_rpm` 600 × 10 mm/min) | none set |
| `sloshing` | 0.015…0.185 m (upright 0.1 m) | 2.0 m/s (the LinMot's default) | 9.81 m/s² |

A run outside them is listed as `skipped` with the reason and left out of the file.

**Jerk is a warning, not a skip.** The drive does not refuse a motion with more jerk than its
limit; it rounds it off, so the slide moves less than asked. The run is kept, its peak jerk is in
the run table, and the program prints `run N: peak jerk J m/s^3 above 3.0: the drive rounds the
motion off`. A sine of stroke s and period T has a peak jerk of (s/2)(2π/T)³: 20 mm at 1 s is
2.5 m/s³, 20 mm at 0.8 s is 4.8. Irregular runs reach the limit sooner, because jerk grows with
f³ and the band reaches 3 fp.

### Check and play

Look at the `.png`, then the dry run, then play it with the oscillator service running
([Running the vertical oscillator](#running-the-vertical-oscillator)):

```bash
ros2 run rig_controller play_dat session_motion.dat vertical --rig oscillator --dry-run
ros2 run rig_controller play_dat session_motion.dat vertical --rig oscillator
```

The vertical slide lags the setpoint and loses some stroke at short periods (97 % at 2 s, 90 % at
1 s, unloaded; TODO item 46): record `oscillator_position` with every run and compare it with the
run table.

### Tests

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test/test_generate_oscillations.py -v
```

## Interfaces

All names are under `/wavemakers/<wavemaker>/`.

### Action `move_wavemaker` (`rig_interfaces/action/MoveRig`)

Each goal picks its type: `positions` empty means a regular wave, otherwise a pregenerated
one. A goal with both `positions` and `height`/`period` is rejected.

Regular wave (`positions` empty). `height` is the wave height H (crest to
trough) in m and `period` is in s; the controller converts it to a paddle stroke with the
wave transfer function. The wave runs until it is cancelled.

```bash
ros2 action send_goal --feedback /wavemakers/ladertanken/move_wavemaker \
  rig_interfaces/action/MoveRig "{height: 0.0001, period: 10.0}"
```

For ladertanken (flap, 1.0 m depth, 1.30 m attachment, 0.00053 m/deg) this is about ±1.2° at the
drive. Recalculate if those values change.

Pregenerated (`height` and `period` left at 0): absolute paddle positions in m at a fixed
`sample_interval` in s. The goal succeeds after the last sample.

```bash
ros2 action send_goal --feedback /wavemakers/ladertanken/move_wavemaker \
  rig_interfaces/action/MoveRig \
  "{positions: [-0.00053, 0.0, 0.00053, 0.0, -0.00053], sample_interval: 1.0}"
```

A goal is **rejected** if the controller is not active, another goal or a return is running, the
drive is faulted or not live, the paddle is outside the travel limits (plus the tolerance), any
position the motion needs is outside the limits, or its peak speed exceeds the drive's limit.

Feedback: `desired_position`, `actual_position` (m) and `elapsed_time` (s).

Cancel a goal with Ctrl-C in `ros2 action send_goal`, or cancel everything with the service below.
A cancelled wave stops and holds position; call `return_to_upright` to go back to upright
(see TODO 18).

### Service `return_to_upright` (`rig_interfaces/srv/ReturnToUpright`)

Moves the paddle to `upright_position_m` with a smooth (quintic) profile and replies when
it is there. `tolerance` 0 uses the default.

```bash
ros2 service call /wavemakers/ladertanken/return_to_upright \
  rig_interfaces/srv/ReturnToUpright "{requester: 'cli', tolerance: 0.0}"
```

Reply `status`: `SUCCESS`, `CONTROLLER_INACTIVE`, `BUSY`, `ACTUATOR_NOT_READY`, `TIMEOUT`,
`ACTUATOR_FAULT`, `POSITION_INVALID` or `SOFT_STOP`, with `final_position` and a `message`.
An exception during the return gives `ACTUATOR_FAULT` with its text; a drive fault gives
`ACTUATOR_FAULT` even when it deactivates the node; a soft stop gives `SOFT_STOP` (the actuator
is halted and stays enabled). `CONTROLLER_INACTIVE` is left for a lifecycle transition.

### Service `cancel_all_goals` (`rig_interfaces/srv/CancelAllGoals`)

```bash
ros2 service call /wavemakers/ladertanken/cancel_all_goals \
  rig_interfaces/srv/CancelAllGoals "{requester: 'cli'}"
```

Replies when the cancel is accepted, not when the paddle has stopped.

### Topics (`std_msgs/Float64`, published every control cycle during goals and returns)

| Topic | Content |
|---|---|
| `wavemaker_setpoint` | Paddle position sent to the drive, m (after clamping to the limits). |
| `wavemaker_position` | Measured paddle position, m. |
| `wavemaker_velocity` | Trajectory velocity, m/s. |
| `wavemaker_actuator_setpoint` | Debug: `wavemaker_setpoint` as sent to the drive, in actuator units (degrees for an angular drive). |

Plot setpoint against position to see how well the drive tracks:

```bash
ros2 run rqt_plot rqt_plot /wavemakers/ladertanken/wavemaker_setpoint/data \
  /wavemakers/ladertanken/wavemaker_position/data
```

### Bridge (`bridge_node`, bridge mode)

The bridge turns topic commands from a client (the web server) into `move_wavemaker` goals and
reports what happens. It runs one regular wave at a time; pregenerated motion goes straight to
the action (`play_dat`). For the oscillator (`rig: oscillator`) the action is `move_oscillator`
and the status topics are `oscillator_state` / `oscillator_message`.

| Name | Type | QoS | Direction | Content |
|---|---|---|---|---|
| `height` | `std_msgs/Float64` | reliable, depth 10 | to the bridge | Wave height H [m] (the stroke for the oscillator) |
| `period` | `std_msgs/Float64` | reliable, depth 10 | to the bridge | Wave period [s] |
| `start` | `std_msgs/Bool` | reliable, depth 10 | to the bridge | `true` starts a wave with the last height and period |
| `stop` | `std_msgs/Bool` | reliable, depth 10 | to the bridge | `true` stops the wave |
| `stop` (service) | `std_srvs/Trigger` | | to the bridge | Stops the wave and replies when it has stopped (below) |
| `heartbeat` | `std_msgs/Bool` | sensor data (best effort) | to the bridge | From the client, at least every `heartbeat_timeout` (1.0 s); the web server sends 10 Hz |
| `wavemaker_state` | `std_msgs/String` | reliable, transient local, depth 1 | from the bridge | The state (below); on every change and again every 1 s |
| `wavemaker_message` | `std_msgs/String` | reliable, transient local, depth 1 | from the bridge | What happened last, in words; published with each state |

**Start sequence.** Publish `height` and `period`, then `start`. They are separate topics and DDS
does not order them, so a start sent right after the values can use old ones. The bridge puts the
values it used in the message (`Starting wave: height 0.0200 m, period 1.000 s`); the web server
checks it. A start is **refused**, with the state unchanged and a message beginning
`start refused:`, unless the bridge is `idle`, height and period are positive and finite, a
heartbeat has arrived within `heartbeat_timeout`, and the action server is available.

**Heartbeat.** While a wave is `starting` or `running`, a heartbeat older than
`heartbeat_timeout` stops it: "Stopped: client heartbeat (LabVIEW or web page) lost". A frozen
page or a dropped network therefore stops the paddle.

`wavemaker_state` values:

| State | Meaning | Next |
|---|---|---|
| `inactive` | Bridge configured but not active, or deactivated. Commands are not received | `idle` on activate |
| `idle` | Ready for a start | `starting` |
| `starting` | Goal sent, the controller has not accepted it yet | `running`; `idle` if rejected; `stopping` on a stop |
| `running` | The controller accepted the goal; the wave runs until stopped | `stopping`; `idle` if the controller aborts it |
| `stopping` | Cancel sent, waiting for the controller's result | `idle` |

How a wave ends (`wavemaker_message` with `idle`):

| Message | Cause |
|---|---|
| `Stopped, holding position.` | Stopped (topic, service, lost heartbeat) and confirmed by the controller. The paddle holds; return to upright separately |
| `Wave aborted: <reason>` | The controller aborted the goal (fault, soft stop, its deactivation) |
| `Goal rejected by controller (see its log)` | The controller refused the goal (limits, busy, soft stop) |
| `<reason>; stop not confirmed by controller, verify` | No result within 2 s of the stop, or the action is no longer available. The bridge gives up on the goal so a new start is possible; **check the paddle** |

A stop during `starting` cancels the goal as soon as the controller accepts it. The `stop` service
replies `success: true` with the end message once the controller confirms, `success: false` when
the stop is not confirmed, and `success: true`, "Nothing running" when no wave was running (it
still cancels any goal on the action, e.g. from the terminal). Deactivating the bridge stops the
wave and leaves it `inactive`, also when the controller's result arrives later.

Parameters: `rig` (`wavemaker` or `oscillator`), `heartbeat_timeout` (s, default 1.0) and
`bond_timeout` (set by the launch file).

## How a motion runs

- **Start blend.** A regular wave blends from the measured position into the wave over one
  period, or more whole periods when the paddle starts far from the wave, so the blend stays
  under `return_to_upright_max_velocity_mps` (quintic smoothstep, phase-continuous). A pregenerated goal blends from the measured position to
  its first sample at the return speed, then plays the samples with linear interpolation.
- **Control loop.** Runs at `1000 / poll_interval_ms` Hz on the steady clock, so every setpoint
  reaches the drive.
- **Drive commands.** The drive runs in drive-internal interpolation: each cycle it gets a target
  position and a speed cap. The cap is the motion's peak speed × `positioning_velocity_margin`
  (never above `max_velocity_rpm`).
- **Stopping.** At the end of a goal, on cancel, and after a return, the drive holds the last target
  and stays enabled, ready for the next motion. Deactivation disables the drive.

## Faults and stopping

| Event | What happens |
|---|---|
| Drive error, PROFIBUS slave lost, or Modbus link lost | The driver latches a fault and sets the control word to 0 (drive off). The controller aborts the running goal and deactivates; it ends up unconfigured. Configure and activate again after fixing the cause. |
| Setpoint rejected by the drive | Same as a fault: goal aborted, drive stopped, controller deactivated. |
| Lifecycle deactivate / cleanup / shutdown | Running goal aborted, return stopped, drive disabled. |
| Ctrl-C on the controller | The node stops the drive in its destructor. |
| PC or controller crashes | The gateway keeps the last outputs unless its **output fault timeout** is set. Configure it on the MGate (fault value 0, a few times `poll_interval_ms`, e.g. 500 ms, longer than the ~210 ms stalls seen on ladertanken) so the drive switches off by itself. |

## Choosing the poll interval

The gateway exchanges data with the drive every `poll_interval_ms`, and the control loop runs at
that rate. Shorter intervals give smoother motion, as long as the gateway keeps up. Measure it on
the rig, **with the controller stopped and the drive disabled** (the tool writes control word 0):

```bash
ros2 run mgate5101_driver poll_rate_probe --seconds 10
# options: --host 192.168.1.70 --port 502 --intervals 50,40,30,20,15,10,5,2
```

It reports cycle times, overruns and errors per interval and recommends the shortest usable one.
Set `poll_interval_ms` to that (or the next longer value), and keep the MGate output fault timeout
at several times it. The register layout in the tool matches ladertanken.

## Tests

```bash
cd ~/ros_ws
colcon build --packages-select mgate5101_driver rig_controller
colcon test --packages-select mgate5101_driver rig_controller
colcon test-result --verbose
```

- `test_controller_node`: lifecycle, goals, cancel, return to upright, faults, speed limits, clamping
  and topics, using a fake actuator.
- `test_bridge_node`: the bridge against a fake `move_wavemaker` server: start and refused
  starts, stop by topic and service, stop during start, lost heartbeat, controller rejection and
  abort, deactivation, and a stop the controller never confirms.
- `test_wave_trajectory`: start blends, peak-speed bounds and pregenerated sampling.
- `test_indradrive_actuator`: unit conversion (angular, and linear with the drive's units and
  direction), parameter checks, the drive limits from the travel, and `halt()` against a fake
  MGate (`test/fake_mgate.hpp`).
- `test_controller_node` also configures, activates and moves the vertical oscillator's block on
  the fake MGate.
- `mapping` (driver): register mapping.

The fake actuator does not model all drive behaviour, so passing tests do not replace the rig
checks. `colcon test` also runs the linters; uncrustify and lint_cmake still report style issues
in older files (TODO 32).

## Recommended test order on the rig

1. Complete the physical checks in [COMMISSIONING.md](COMMISSIONING.md).
2. Run `poll_rate_probe` and set `poll_interval_ms`; configure the MGate output fault timeout.
3. Launch one wavemaker with the lifecycle manager off, and check communication without motion
   (`mgate_example` in monitor mode, or configure only).
4. Test the lifecycle transitions by hand and confirm that activation causes no motion.
5. Call `return_to_upright`, then send a very small, low-frequency goal.
6. Test cancel, then a second goal, deactivation while moving, communication loss and the
   emergency stop.
7. Plot `wavemaker_setpoint` against `wavemaker_position` and tune `positioning_velocity_margin`.
8. Enable the lifecycle manager only after manual testing succeeds.
