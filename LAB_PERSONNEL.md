# Lab personnel

Building, configuring, testing and calibrating the rigs: the parts of the documentation
for whoever maintains them. Running a rig and making motion files is in
[README.md](README.md); one-time checks on the rig in [COMMISSIONING.md](COMMISSIONING.md),
the check before every session in [STARTUP.md](STARTUP.md), open work in [TODO.md](TODO.md).

## Contents

- [Packages](#packages)
- [Build](#build)
- [Configuration](#configuration)
- [Running a wavemaker by hand](#running-a-wavemaker-by-hand)
  - [Terminal mode](#terminal-mode)
  - [Launch arguments](#launch-arguments)
- [Motion files: background](#motion-files-background)
  - [Transfer gain from a run](#transfer-gain-from-a-run)
- [Interfaces](#interfaces)
- [How a motion runs](#how-a-motion-runs)
- [Faults and stopping](#faults-and-stopping)
- [Choosing the poll interval](#choosing-the-poll-interval)
- [Tests](#tests)

## Packages

| Package | Contents |
|---|---|
| `drivers/mgate5101_driver` | Standalone C++ driver: cyclic Modbus TCP client for the MGate (`MGateDriver`) and the IndraDrive state machine on top of it (`IndraDrive`). Includes `poll_rate_probe` and a monitor/move example. Register mapping is explained in [MAPPING.md](drivers/mgate5101_driver/MAPPING.md). |
| `drivers/linmot_serial_driver` | Standalone C++ driver for the LinMot oscillator: the serial link to the Arduino serial-to-CAN bridge (`SerialBridge`, firmware in `firmware/linmot_serial_bridge`) and the LinMot drive's start-up and faults on top of it (`LinMotDrive`). `linmot_probe` checks the link without moving anything. Not used by the controller yet. |
| `rig_interfaces` | `MoveRig` action, `ReturnToUpright` and `CancelAllGoals` services. |
| `rig_controller` | `controller_node` (the lifecycle controller), the wave and trajectory maths, the IndraDrive actuator adapter, and `bridge_node` (see the note below). Also the Python tools: `play_dat`, `generate_waves`, `generate_oscillations` and `transfer_function` (the wavemaker's gain from a run and a probe record; [Transfer gain from a run](#transfer-gain-from-a-run)). |
| `wavemaker_bringup` | Launch file and per-wavemaker configuration (`config/wavemakers.yaml`). |
| `oscillator_bringup` | Launch file and configuration of the oscillators (`config/oscillator.yaml`: `sloshing`, `vertical`); `oscillator:=` picks one. |
| `rig_web` | Web page for running a wavemaker or the oscillator through the bridge (`rig` parameter; see [Web page](README.md#web-page)). |

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
| `water_depth` | Water depth in m. Can be overridden for one run with the `water_depth` launch argument (see [Water depth](README.md#water-depth)). |
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
  (÷ 0.000413 m/deg on ladertanken).
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

## Running a wavemaker by hand

For first tests on a new rig and for debugging. Users run the wavemaker in bridge mode or
through the service ([README.md](README.md#running-a-wavemaker)).

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

### Launch arguments

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

## Motion files: background

How `generate_waves` fits the controller, why the irregular waves are made as they are, and
measuring the transfer gain. Making and playing files is in
[README.md](README.md#generating-motion-files).

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

### Transfer gain from a run

`transfer_function` (`rig_controller/src/transfer_function.py`) measures the wavemaker's
transfer gain from one run of a motion file and a catman record of it: across the whole band
from one irregular run, or at its one period from a regular run. It prints the gain next to the
yaml's table; it doesn't change the yaml. The design and its decisions are in the git history
(`git show c6bba03:transfer_function_sketch.md`); finding each run in the record in
`git show 88363ba:run_timeline_sketch.md`.

**What it needs:**

- The `.dat` and its run table, from `generate_waves`. Files made before 2026-11-13 have no
  `ramp_s` column and are refused: generate the file again from its test matrix and its
  `.txt`'s depth. The seeds are fixed, so the motion is the same.
- A catman record of the whole file: an ASCII export (`.ASC`, read with its channel names and
  `dt`), or a CSV with a time column in seconds and one column per channel. At least the wave
  probe; best also the flap's draw wire (`WM_Position`), which splits the gain into the drive's
  part and the water's part.
- Still water before the first run, the probe's distance from the flap, the measured water
  depth, and the draw wire's height above the tank floor.
- **WP1's scaling checked in catman** before the session: the channel must use the sensor
  database's 0.0229 m/V (on 2026-11-13 it had 0.03981, and every height read 1.74 × too high).

```bash
ros2 run rig_controller transfer_function item38.dat ladertanken --run 3 \
    --record item38.ASC --probe WP1 --probe-distance-m 3.0 \
    --stroke WM_Position --stroke-scale -1.342 --stroke-height-m 1.375 \
    --depth-m 0.98 -o item38_run3_tf --plot
```

`--stroke-scale -1.342` is ladertanken's draw wire: it counts positive away from the beach and
reads 0.74 of the horizontal travel at its height (calibrated with an inclinometer, 2026-11-13,
COMMISSIONING.md, *Actuator conversion calibration*).

It finds where each run is in the record by itself, also in a long file with several runs. One
offset for the whole file doesn't hold: `play_dat` used to start each run's goal about 50 ms
later than the file says, and catman's clock runs against the Pi's (+253 ppm on 2026-11-17, half
a second over a 17-minute session). So it:

1. **anchors** on the first run, from the draw wire if there is one (on at most 120 s of the run),
   else from the wave's envelope (on the whole run);
2. **walks** every run in time order, in pieces of at most 60 s, each searched within ±0.4
   periods of the last (with the probe only, the probe with its travel and linear theory's 90°
   lead taken out);
3. **fits** one start per run and one clock drift for the file, from runs of two pieces or more
   (at least 180 s of them; otherwise the drift is *not measured* and taken as 0), and reads the
   analysed run at the record times this gives.

It prints a row per period of the yaml's gain table, with the measured gain, its ±, the
coherence and the table's value (and with the draw wire also the drive's and the water's part),
then a summary: where the run is, the window, the averages and the checks. It writes:

- `<output>.csv`: every trusted frequency bin, raw and smoothed.
- `<output>.txt`: the command, the commit and the summary: what the result was made from.
- `<output>.png` (with `--plot`): gain, drive and water, coherence and residual phase against
  period.

| Option | Use |
|---|---|
| `--run N` | The run, as numbered in the run table (required) |
| `--record`, `--probe` | The record (`.ASC` or `.csv`) and the probe's channel name (required) |
| `--probe-distance-m` | The probe's distance from the flap (required) |
| `--depth-m` | The measured depth (required: no default from the yaml) |
| `--probe-scale`, `--stroke-scale` | To metres: `0.001` for a channel in mm; negative flips the sign |
| `--stroke`, `--stroke-height-m` | The draw wire's channel and its height above the floor (together) |
| `--offset-s` | This run's start in the record less its start in the file, in seconds (the `offset` line), when the anchor or the walk can't find it. Skips both; the drift then comes from this run's own pieces |
| `--drift-ppm` | catman's clock against the Pi's, when it can't be fitted; `0` with `--offset-s` is one offset for the run |
| `--segment-s 40` | Welch segment length; longer resolves finer, with a larger ± |
| `--smooth-hz 0.05` | Average over the beach reflection's ripple; `0` for the raw gain |
| `--config <file>` | Another `wavemakers.yaml` |

**Reading the output:**

- **`offset`** is the analysed run's start in the record less its start in the file, and how it
  was found (the anchor's match: refused below 2× the next best). It includes the drive's lag
  (0.1–0.2 s) when found from the draw wire.
- **`runs`** is the same for every run walked, and **`late`** each run's start against the
  first, drift removed: the goals' accumulated start latency. With the current `play_dat` it
  should stay within about ±20 ms of 0 (the drive's lag changes a little with period); in
  records played with an older `play_dat` it rises by about 50 ms a run.
- **`drift`** is catman's clock against the Pi's, with the pieces it came from and their rms
  scatter about the fit; a warning above 10 ms means something moved the timeline inside a
  run. Above 1000 ppm it's refused (the wrong record, or a gap in it). Measure it every
  session; don't carry it over.
- **`gain`** is |G_cmd| over theory: the number for the gain table. **`drive`** is |G_drive|
  and **`water`** is |G_water| over theory, both at the waterline and both 1.0 when ideal (only
  with `--stroke`). The ± columns are absolute: the value × the random error / √n_smooth.
- **Drive below 1 and falling with frequency** is the drive lagging.
- **Water about 1 and drive about the table's gain** means the table is correcting the drive,
  not the water. The drive's response may change with load; the water's doesn't.
- **`chain`**, the largest difference between G_cmd and drive × water: a few % at most. A large
  one means the stroke and the probe don't belong together (wrong channel, wrong scale).
- **`delay`** (G_cmd's residual phase as a straight line) is the drive's delay plus the
  offset's error, so it's no measure of the drive. It checks the offset: well under 1 s without
  the stroke, and about the drive's lag (0.1–0.2 s) or less with it.
- **`water delay`** is free of the offset (stroke and probe are sampled together): 0 when the
  distance and the depth are right. A slope is an error in `--probe-distance-m` or `--depth-m`.
  The residual phases are after the wave's travel to the probe and linear theory's 90° lead
  over the stroke are taken out.
- **Curvature in a residual phase** is a lag; a straight line is a pure delay.
- **Coherence** below 0.8 removes a bin, and so does everything above 1.5 fp, where
  second-order waves at 2 fp would count as gain. At the band's edges the bins have the least
  energy and the largest ±.
- **The stillness warning** compares the probe's std in the 10 s before the run with 10 % of the
  run's, with a 1 mm floor.
- **Segments**: `--segment-s 40` gives df = 0.025 Hz and about 14 averages on a 300 s run.
  Longer segments resolve finer and show the beach reflection's ripple in |G| fully, with a
  larger ±; `--smooth-hz` averages it away.

**Limits:**

- The drive's absolute delay isn't measured: only the shape of its response.
- A low irregular wave (Hs 0.03 m) gives the linear gain only. Steepness effects need regular
  runs.
- The wave's travel is taken out with linear dispersion; for steep waves the real speed is a
  little higher.
- With a regular run first, the draw-wire anchor can refuse a correct offset (a sine flipped
  looks like a sine shifted half a period). Put an irregular run first, or give `--offset-s`
  (TODO 43).
- A run whose start moved more than 0.4 periods from the run before (a goal started very late)
  is refused, naming the run: give `--offset-s` for it.

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

For ladertanken (flap, 1.0 m depth, 1.39 m attachment, 0.000413 m/deg) this is about ±1.8° at the
drive. Recalculate if those values change.

Pregenerated (`height` and `period` left at 0): absolute paddle positions in m at a fixed
`sample_interval` in s. The goal succeeds after the last sample.

```bash
ros2 action send_goal --feedback /wavemakers/ladertanken/move_wavemaker \
  rig_interfaces/action/MoveRig \
  "{positions: [-0.000413, 0.0, 0.000413, 0.0, -0.000413], sample_interval: 1.0}"
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

### Motion file tools (Python)

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test/test_generate_waves.py -v
python3 -m pytest rig_controller/test/test_transfer_function.py -v
```

`test_transfer_function.py` builds a session with a known drive and water, optionally with late
goals and a drifting catman clock, and checks that the tool gives them back.

### Oscillator motion files

```bash
cd ~/ros_ws/src/hydrolab_rigs
python3 -m pytest rig_controller/test/test_generate_oscillations.py -v
```

