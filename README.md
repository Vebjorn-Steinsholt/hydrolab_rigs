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

This README is for running the rigs and making motion files. Building, configuration, the
interfaces, tests and the transfer gain calibration are for lab personnel, in
[LAB_PERSONNEL.md](LAB_PERSONNEL.md).

## Contents

- [Lab computers](#lab-computers)
- [Running a wavemaker](#running-a-wavemaker)
  - [Water depth](#water-depth)
  - [Web page](#web-page)
- [Running the vertical oscillator](#running-the-vertical-oscillator)
- [Generating motion files](#generating-motion-files)
  - [From test matrix to tank](#from-test-matrix-to-tank)
- [Generating oscillator motion files](#generating-oscillator-motion-files)
- [If it stops](#if-it-stops)

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

## Running a wavemaker

There are two ways to run a wavemaker. Both read `config/wavemakers.yaml` and take
`wavemaker:=ladertanken|lilletanken|mclab` (default `ladertanken`).

| | Terminal mode (experienced users) | Bridge mode (normal users) |
|---|---|---|
| Launch file | `wavemakers.launch.py` | `bridge.launch.py` |
| Nodes | `controller` | `controller` and `bridge` |
| Lifecycle | Stepped by hand (lifecycle manager optional, off by default) | Configured and activated automatically by the Nav2 lifecycle manager |
| Control | `ros2 action` / `ros2 service` (see [Interfaces](LAB_PERSONNEL.md#interfaces)) | The bridge's topics: `height`, `period`, then `start`; `stop`; a `heartbeat` from the client (the web page) |
| If a node dies | Nothing stops the controller | The lifecycle manager brings both nodes down after `bond_timeout` (default 1 s), which disables the drive |

> Bridge mode with the web page runs on the lab Pi (2026-10-12). For first tests on a new rig,
> use terminal mode.

### Bridge mode

```bash
ros2 launch wavemaker_bringup bridge.launch.py wavemaker:=ladertanken
```

Both nodes are configured and activated as soon as they start, so the drive is enabled
straight away. If either node stops sending bond heartbeats for `bond_timeout` seconds (a crash,
a hang, or the bridge being killed), the lifecycle manager brings both down and the paddle
stops. Set `bond_timeout:=<seconds>` to change this; too short a value can trigger on a briefly
busy PC.

Launch arguments and terminal mode:
[LAB_PERSONNEL.md](LAB_PERSONNEL.md#running-a-wavemaker-by-hand).

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
down. It is daisy-chained behind the ladertanken wavemaker's drive on the ladertanken MGate:
PROFIBUS address 3, gateway registers 7 (inputs) and 6 (outputs); the wavemaker keeps address 2
and registers 0 / 0. The two rigs can run at the same time (one soft stop button stops both). The decisions and
code plan are in `git show c6bba03:oscillator_sketch.md` (done); the drive, the commissioning
steps and the log are in [COMMISSIONING.md](COMMISSIONING.md#vertical-oscillator-ladertanken).

**Configuration:** the `vertical` blocks in
[`oscillator_bringup/config/oscillator.yaml`](oscillator_bringup/config/oscillator.yaml)
(`/oscillators/vertical/controller` and `/oscillators/vertical/bridge`): `rig: oscillator`,
`drive_name: indradrive`, `drive_motion: linear` with the drive's units
(`drive_m_per_unit: 0.001`, `drive_velocity_per_unit_per_s: 6.0`, `drive_direction: -1`, see
[Actuator and limits](LAB_PERSONNEL.md#actuator-and-limits)), travel −0.2…0.2 m around upright 0, speed limit
`max_velocity_rpm: 1800` (0.3 m/s, tested on the rig 2026-11-12, [COMMISSIONING.md, *Done*](COMMISSIONING.md#done)).

**Start and stop** on the ladertanken Pi, through the oscillator service with
`OSCILLATOR=vertical` in `/etc/default/oscillator` ([rpi/README.md](rpi/README.md#oscillator)):

```bash
sudo systemctl start oscillator@ladertanken     # switches the drive on; it does not home
sudo systemctl stop oscillator@ladertanken      # stops the motion, drive off, brake holds
```

By hand: `ros2 launch oscillator_bringup oscillator.launch.py oscillator:=vertical` (default
`sloshing`). The nodes run in `/oscillators/vertical`. Add `upright_position_m:=0.05` to oscillate
around another centre (m, + = up) without editing `oscillator.yaml`; empty or left out, the yaml's
value is used. Press *Return to upright* before the first wave, and check files with
`play_dat --upright-m 0.05`. Through the service, set `VERTICAL_UPRIGHT_M` in
`/etc/default/oscillator` instead ([rpi/README.md](rpi/README.md#use-1)).

**Running it next to the wavemaker:** start both services, in either order
([rpi/README.md](rpi/README.md#oscillator)). Both drives stay connected; each yaml addresses
only its own drive's block. The sloshing oscillator, by contrast, runs only alone.

**Running:** the web page (`http://ladertanken-rpi.local:8081/`): *height* is the stroke, peak to
peak, in metres, around mid-travel; + is up. **Stop** cancels the wave and the slide holds where it
is, drive enabled; **Return to upright** brings it back to mid-travel. Motion files are made with
`generate_oscillations` ([Generating oscillator motion files](#generating-oscillator-motion-files))
and play with `play_dat <file> vertical --rig oscillator`; the speed limit comes from the yaml.

**Tracking (2026-11-09, no load, no water):** the slide follows the setpoint with about 180 ms of
lag; the stroke reaches 97 % at a 2 s period and 90 % at 1 s, for any stroke. It is not tuned yet
and depends on the load (TODO item 46). Record `oscillator_position` with every run.

## Generating motion files

> **Measured with probes on ladertanken** (2026-11-05 and 2026-11-13): with the gain table of
> 2026-11-13, regular waves at 1.0–1.6 s came within 2 % of the target; 0.8 s was 8 % low. To be
> confirmed with WP1 recalibrated ([TODO.md](TODO.md), item 43).

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

It plays the runs in order, with the pauses between them. Each goal goes out at its run's time
in the file, counted from the first goal, not a pause after the previous run's result, so a
goal's start latency (about 50 ms) doesn't add up over a session. `--runs 1` or `--runs 0-1`
plays only some runs (numbers as in the dry run); after a skipped run, the pause is played from
the previous result. It warns if the running controller's limits differ from
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
- `session_waves.csv`: the run table: per run its start and end, the length of each ramp
  (`ramp_s`), the parameters, H and 1/x (regular), the paddle amplitude or its standard
  deviation and peak (irregular), and `skipped` with the reason for a run that does not fit the
  wavemaker.
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
`wavemakers.yaml`. For ladertanken (attachment 1.39 m, hinge 0.09 m, both above the floor) the
last factor is 1.43 at 1.0 m depth and 1.46 at 0.98 m, and the gain is the table for the period
(0.88-1.16, since 2026-11-13). A piston
moves the same at every height: x = (S / 2) / gain. A file is only right for the depth it was
made for; note the depth with it.

**Limits** (ladertanken; others in their yaml block): positions within ±0.114 m and speed below
0.595 m/s (±275° and 240 rpm at 0.000413 m/deg). `play_dat --dry-run` checks every run against them; a file in mm is refused there,
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
- `session_motion.csv`: the run table: per run its start and end, the length of each ramp
  (`ramp_s`), the parameters, the amplitude (regular) or the standard deviation and peak
  (irregular), `peak_jerk_mps3`, and `skipped` for a run that does not fit the oscillator.
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
| `vertical` | −0.2…+0.2 m (upright 0) | 0.3 m/s (`max_velocity_rpm` 1800 × 10 mm/min) | none set |
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

## If it stops

A fault on the drive, the gateway or the network stops the motion and switches the drive off,
and the page shows the message. Nothing switches the drive on again by itself: on the Pi the
nodes stay down while the service keeps running. Check the journal
(`journalctl -u wavemaker@<wavemaker>`), the emergency stop and the drive's mains, then stop and
start the service ([rpi/README.md](rpi/README.md#how-it-stops)). What happens in each case, for
lab personnel: [LAB_PERSONNEL.md](LAB_PERSONNEL.md#faults-and-stopping).

