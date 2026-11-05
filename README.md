# Hydrolab rigs

ROS 2 (Jazzy) packages for controlling the Hydrolab rigs: the wavemakers and the sloshing
oscillator (`OSCILLATOR.md`). On a wavemaker, a lifecycle controller node generates
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
- [Generating motion files](#generating-motion-files)
  - [On the Pi](#on-the-pi)
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
| `rig_web` | Web page for running a wavemaker or the oscillator through the bridge (`rig` parameter; see [Web page](#web-page)). |

> **`bridge_node`** connects topic-based clients, such as the web page, to the controller.
> Its topics are `height`, `period`, `start`, `stop` and `heartbeat` in, and `wavemaker_state` /
> `wavemaker_message` out.

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
| `drive_name` | Required, no default. `indradrive` (MGate gateway, the wavemakers) or `linmot` (Arduino serial-to-CAN bridge, the oscillator). Configure fails when it is missing or unknown. A new drive is an `Actuator` subclass plus one branch in `make_actuator()` (`rig_controller/src/controller_node.cpp`). |
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
| `drive_motion` | `angular` (drive position in degrees, converted with the lead) or `linear`. The conversion is `DriveScale` (`rig_controller/src/drive_scale.hpp`), shared by the controller's limits and the actuators. Linear units are not verified yet (TODO 19). |
| `actuator_lead_m_per_degree` | Angular only: paddle travel per drive degree. |
| `actuator_upright_angle_deg` | Angular only: drive angle at upright. |
| `min_position_deg` / `max_position_deg` | Angular travel limits in drive degrees. Goals that leave them are rejected; the drive also refuses targets outside them. |
| `travel_minimum_m` / `travel_maximum_m`, `min_position_m` / `max_position_m` | Linear travel limits in m (node and drive side; keep them equal). |
| `max_velocity_rpm` | Drive speed limit. Goals whose peak speed exceeds it are rejected. |
| `hold_velocity_rpm` | Speed cap used when stopping and holding position (default 10, ≤ `max_velocity_rpm`). |

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
| `return_to_upright_max_velocity_mps` | 0.1 | Peak speed of a return to upright, and of the blend at the start of a pregenerated goal. Must not exceed the drive's limit. |
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

## Generating motion files

> **Regular and irregular waves played on the rig** (ladertanken, 2026-11-04 and 2026-11-05,
> [rpi/README.md](rpi/README.md)). The wave heights have not yet been measured with probes
> ([TODO.md](TODO.md), item 43).

`generate_waves` (`rig_controller/src/generate_waves.py`) turns a test matrix into a motion file for one wavemaker,
ready for `play_dat`. Wavemakers only; the oscillator gets a separate generator later
([TODO.md](TODO.md), *Oscillator motion generator*).

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

### Generate

```bash
source ~/ros_ws/install/setup.bash
cd ~/wave_sessions          # the folder with the test matrix
ros2 run rig_controller generate_waves ladertanken session.csv -o session_waves
```

File names are relative to the current folder, for the test matrix and for `-o`; from elsewhere
give full paths (`~/wave_sessions/session.csv -o ~/wave_sessions/session_waves`). It writes:

- `session_waves.dat`: the motion file for `play_dat`.
- `session_waves.csv`: the run table: per run its start and end, the parameters, H and 1/x
  (regular), the paddle amplitude or its standard deviation and peak (irregular), and `skipped`
  with the reason for a run that does not fit the wavemaker.
- `session_waves.png`: paddle position and speed over the whole file, with the wavemaker's
  limits dashed and the runs numbered as in the run table.

and prints a summary and the `play_dat` command to check the file:

```
session_waves.dat: 4 runs, 0 skipped, 9.3 min
ros2 run rig_controller play_dat session_waves.dat ladertanken --dry-run
```

For a linear drive the command also carries `--max-velocity-mps`, and for another yaml `--config`.

**Give `-o` a different name from the test matrix:** `-o session` would overwrite `session.csv`
with the run table.

| Option | Use |
|---|---|
| `--water-depth 0.95` | The depth of this session instead of the yaml's; warns when the gain table was measured at another depth |
| `--dt 0.01` | Sample interval [s], whole milliseconds; default 0.02 |
| `--show` | Also open the plot in a window |
| `--no-plot` | Do not write the `.png` |
| `--config <file>` | Another `wavemakers.yaml`; default the installed one, else the repository's |
| `--max-velocity-mps` | Required for linear drives (lilletanken, mclab), whose speed limit is not known from the yaml yet (TODO item 19) |

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

### On the Pi

Generate and play on the Pi: its `wavemakers.yaml` (gain table, depth, geometry) is the one the
controller runs with, and `play_dat`'s cross-check compares the limits only. From the lab PC (set
up once as in [windows/README.md](windows/README.md)) or any computer with ssh, copy the test
matrix into a new session folder and log in:

```bash
ssh ladertanken@ladertanken-rpi.local "mkdir -p wave_sessions/2026-11-05"
scp session.csv ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05/
ssh ladertanken@ladertanken-rpi.local
```

On the Pi, work in `tmux` so that a dropped connection does not stop a run (`tmux attach` gets
back to it):

```bash
tmux
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=1        # the lab's domain; without it play_dat finds no controller
cd ~/wave_sessions/2026-11-05
ros2 run rig_controller generate_waves ladertanken session.csv -o session_waves
```

The Pi has no screen, so `--show` does not apply. Copy the plot to the computer you sit at and
open it there (in a second window on that computer):

```bat
scp ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05/session_waves.png .
start session_waves.png
```

(`xdg-open` instead of `start` on Linux.) Then the dry run and play as above, in the `tmux`
window. With VS Code's Remote-SSH extension, the Pi's folder opens directly and the plot shows in
a tab.

Keep one folder per session (matrix, `.dat`, run table, plot): the run table says which run was
played when, so it belongs with the probe data. Copy the folder off the Pi afterwards:

```bash
scp -r ladertanken@ladertanken-rpi.local:wave_sessions/2026-11-05 .
```

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
`ACTUATOR_FAULT` or `POSITION_INVALID`, with `final_position` and a `message`.

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

## How a motion runs

- **Start blend.** A regular wave blends from the measured position into the wave over one period
  (quintic smoothstep, phase-continuous). A pregenerated goal blends from the measured position to
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
- `test_wave_trajectory`: start blends, peak-speed bounds and pregenerated sampling.
- `test_indradrive_actuator`: unit conversion and parameter checks.
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
