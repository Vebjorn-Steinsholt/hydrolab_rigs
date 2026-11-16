# Running a lab Pi as a service

`wavemaker@.service` runs `lab.launch.py` (controller, bridge, lifecycle manager and web page)
under systemd; on the ladertanken Pi, `oscillator@.service` runs `oscillator.launch.py` the same
way, for the sloshing or the vertical oscillator (see [Oscillator](#oscillator)). The soft stop
button has a unit of its own, `soft_stop@.service`, which the rig units start and which serves
every rig running on the Pi. Unlike a launch in an SSH session, it keeps running when the connection
drops, logs to the journal, and is started and stopped the same way on every Pi.

It is a template unit: the part after `@` is the wavemaker, which is also the Pi's user
(`wavemaker@ladertanken` runs as user `ladertanken`; see *Lab computers* in the top-level README).

| File | Purpose |
|---|---|
| `wavemaker@.service` | The unit. Installed as `/etc/systemd/system/wavemaker@.service`. |
| `oscillator@.service` | The oscillator's unit (ladertanken Pi only). Installed next to the wavemaker's. |
| `soft_stop@.service` | The soft stop button (GPIO17), started by the rig units (`Requires=`) and stopped when the last of them stops. Installed next to them. |
| `run.sh` | `ExecStart`: sources ROS and the workspace, then `exec`s `lab.launch.py`, or `oscillator.launch.py` when the unit sets `RIG=oscillator`, both with `soft_stop:=false`. For the oscillator it first writes which one runs to `/run/oscillator-<user>/oscillator`. |
| `stop.sh` | `ExecStop`: asks the lifecycle manager to shut the nodes down before the launch is stopped. |
| `refuse_if_busy.sh` | `ExecStartPre` of both rig units: refuses the combinations that cannot run together (the sloshing oscillator next to the wavemaker), and a start while the unit's own web port is in use (for example a launch by hand). |
| `soft_stop.sh` | `ExecStart` of `soft_stop@`: one soft stop node that publishes to every controller that can run on the Pi. |
| `99-linmot-bridge.rules` | udev rule for the oscillator's Arduino: `/dev/linmot_bridge`, and ModemManager kept off it. |

> **The service never starts at boot.** The unit has no `[Install]` section, so it cannot be
> enabled: the drive is enabled as soon as `lab.launch.py` runs, and a power cut or reboot must
> not do that by itself.

## Install (once per Pi)

Before you start, as the wavemaker user:

- The workspace is built (see *Build* in the top-level README), and the scripts in `rpi/` are
  executable (`ls -l rpi/*.sh` shows `-rwxr-xr-x` for all four).
- The user is in the `gpio` group (`id` lists `gpio`); the soft stop node needs it.
- `python3-matplotlib` is installed (`sudo apt install python3-matplotlib`); `generate_waves` needs
  it for its plot.
- No `lab.launch.py` is running by hand. A second copy would clash on port 8080 and GPIO17.

Install the units:

```bash
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/{wavemaker,soft_stop}@.service /etc/systemd/system/
sudo systemctl daemon-reload
systemd-analyze verify wavemaker@ladertanken.service soft_stop@ladertanken.service   # no output = OK
```

The units set `WorkingDirectory=/home/%i` because a service otherwise starts in `/`, which the user
cannot write to. lgpio, which the soft stop node uses for GPIO on the Pi 5, creates small files in
the working directory; without a writable one, opening GPIO17 fails and the soft stop stays engaged.

Create `/etc/default/wavemaker` with `sudoedit /etc/default/wavemaker`:

```bash
# The lab's ROS domain. Required, and not 0.
ROS_DOMAIN_ID=1
PORT=8080
# Water depth in m. Empty uses wavemakers.yaml.
WATER_DEPTH=
# The MGate's address (driver_address in the yaml). Required.
MGATE_HOST=192.168.1.70
```

Before each start, the unit sets the route to `MGATE_HOST` to resend a lost packet after 20 ms
instead of Linux's 200 ms minimum (`rto_min`, as root for this one command). With two rigs on
one MGate the gateway drops a frame now and then, and a 200 ms wait stalled the gateway cycle up
to 415 ms, close to the 500 ms fault timeout (COMMISSIONING.md, *Vertical oscillator*, *Done*). Check after a start:
`ip route get 192.168.1.70` shows `rto_min 20ms`.

systemd only treats lines starting with `#` as comments; a comment after a value becomes part of
the value.

systemd does not read `~/.bashrc`, so `ROS_DOMAIN_ID` must be set here. `run.sh` refuses to start
without it, because on domain 0 anyone on the network could command the wavemaker. The settings
are read at start; after a change, stop and start the service.

To start and stop without a password, add a sudoers rule with
`sudo visudo -f /etc/sudoers.d/wavemaker` (always through `visudo`: a syntax error breaks sudo):

```
ladertanken ALL=(root) NOPASSWD: /usr/bin/systemctl start wavemaker@ladertanken, /usr/bin/systemctl stop wavemaker@ladertanken
```

**WiFi off** (done on ladertanken 2026-11-11). The Pi runs on the cable only: with WiFi on the
same subnet, its name also answers with the WiFi address, and traffic to the page, ssh and ROS
can go over WiFi (TODO 49). On a Pi 5 with Ubuntu, where cloud-init wrote the WiFi into
`/etc/netplan/50-cloud-init.yaml`:

```bash
echo "dtoverlay=disable-wifi" | sudo tee -a /boot/firmware/config.txt   # under the last [all]
echo "network: {config: disabled}" | sudo tee /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
sudo mv /etc/netplan/50-cloud-init.yaml /root/50-cloud-init.yaml.bak
sudo netplan generate && sudo reboot
ip -4 -br addr        # after the reboot: only lo and eth0
```

Log in over the cable (`ssh <user>@<the eth0 address>`) before doing this.

`sudo -l` should list it. The soft stop unit needs no rule: systemd starts it for the rig unit.
Then start the service once and check the journals (see *Use*): the wavemaker's should show
`Managed nodes are active` and `Soft stop connected`; the soft stop's
(`journalctl -u soft_stop@ladertanken`), with the button at rest, `Soft stop released`.

## Update

After a `git pull`:

- **Code changed** (anything outside `rpi/`): `cd ~/ros_ws && colcon build`.
- **A unit file changed** (`rpi/*.service`): copy the installed ones again and run
  `sudo systemctl daemon-reload`.
- **A script changed** (`rpi/*.sh`): nothing to do; the units run them from the repository.

Then stop and start the service; a running service keeps the old code.

### Repository renamed to `hydrolab_rigs` (once per Pi, 2026-11)

The repository was `ros2_wavemakers`; the units now run the scripts from
`~/ros_ws/src/hydrolab_rigs`. On a Pi still cloned under the old name, after the rename is merged
into `master` (only the units that are installed):

```bash
sudo systemctl stop wavemaker@ladertanken oscillator@ladertanken
cd ~/ros_ws/src && mv ros2_wavemakers hydrolab_rigs && cd hydrolab_rigs
git remote set-url origin https://github.com/Vebjorn-Steinsholt/hydrolab_rigs.git
git checkout master && git pull
sudo cp rpi/wavemaker@.service rpi/oscillator@.service /etc/systemd/system/
sudo systemctl daemon-reload
cd ~/ros_ws && rm -rf build install log && colcon build
sudo systemctl start wavemaker@ladertanken
systemctl status wavemaker@ladertanken --no-pager | head -3
```

Expected: `Summary: 7 packages finished`, then `active (running)` and the page shows *idle*. The
clean build is needed because CMake's cache in `build/` records the old source path.

## Use

```bash
sudo systemctl start wavemaker@ladertanken     # enables the drive
sudo systemctl stop wavemaker@ladertanken      # stops the wave and disables the drive
systemctl status wavemaker@ladertanken
journalctl -u wavemaker@ladertanken -f         # follow the log; -b for this boot only
```

Type the commands as shown; the sudoers rule matches them exactly.

`soft_stop@ladertanken` starts with the first rig and stops after the last one; it is never
started or stopped by hand. Its log: `journalctl -u soft_stop@ladertanken -f`.

## How it stops

`systemctl stop`:

1. `stop.sh` calls the lifecycle manager's `manage_nodes` with `SHUTDOWN`, which deactivates,
   cleans up and shuts down the bridge and then the controller; the controller disables the drive.
   The call has a 15 s timeout.
2. systemd then sends SIGINT to `ros2 launch` (`KillMode=mixed`), which forwards it to the
   remaining processes. If step 1 failed, this alone still stops the drive: the controller stops
   it when it receives SIGINT.
3. `ros2 launch` escalates to SIGTERM after 20 s (`run.sh` sets `sigterm_timeout`), and systemd
   kills whatever is left after `TimeoutStopSec`.

The soft stop unit stops by itself once no rig unit runs (`StopWhenUnneeded=yes`). The other way
round, if the soft stop unit stops or dies, systemd stops every rig unit with it (`Requires=`);
before that, each controller already treats the missing messages as stopped after 0.2 s
(`soft_stop_timeout_s`).

**Idle auto-stop.** After 2 h without motion (a wave, a `play_dat` goal or a return to
upright), the web server does step 1 itself and exits,
and the launch shuts down with it; the service then ends on its own (`inactive`). The page warns 10
minutes before, with a **Keep running** button. See *Web page* in the top-level README.

The service is never restarted automatically (`Restart=no`): a restart would re-enable the drive.
If a node dies, `ros2 launch` keeps running with the rest. Check the journal, then stop and start
the service.

**After a drive fault** (USB pulled, drive switched off, no feedback) the controller switches the
drive off and drops its bond; the lifecycle manager logs `CRITICAL FAILURE … Shutting down related
nodes` and `Managed nodes have been reset`, and the nodes stay down while the service keeps
running. Nothing switches the drive on again by itself: the launch files set
`attempt_respawn_reconnection: False` (Nav2's default `True` brought the nodes back up within
seconds, found 2026-10-26). Check the journal, fix the cause, then stop and start the service.

## Troubleshooting

`journalctl -f` shows only the last lines, so earlier errors are easy to miss. Search the whole
run instead (`-o cat` keeps the lines short):

```bash
journalctl -u wavemaker@ladertanken -u soft_stop@ladertanken -b --no-pager -o cat | grep -E "ERROR|WARN|died|exception|Soft stop"
```

| Symptom | Cause |
|---|---|
| Fails at once with `set ROS_DOMAIN_ID` | `/etc/default/wavemaker` missing, or `ROS_DOMAIN_ID` empty or 0. |
| Fails at once with `set MGATE_HOST` | The env file has no `MGATE_HOST=` line (see *Install*). |
| `Soft stop released` never appears, and the button changes nothing | `soft_stop` could not open GPIO17 (`Cannot open GPIO17` in `journalctl -u soft_stop@ladertanken`): check `WorkingDirectory=` in the installed `soft_stop@.service` and the `gpio` group. If GPIO is fine, the button is pressed in. |
| The soft stop is released, but one rig still says `SOFT_STOP` or `No Soft stop message received` | That rig's env file has another `ROS_DOMAIN_ID`: `soft_stop@` uses the one in `/etc/default/wavemaker`. Make both the same. |
| Start fails with `Unit soft_stop@ladertanken.service not found` | `soft_stop@.service` is not installed: see *Install*. |
| `ros2` commands on another computer do not see the wavemaker | That computer uses another ROS domain or another DDS (`RMW_IMPLEMENTATION`). The Pi uses the default, Fast DDS. |
| `status` shows `failed` after a stop | Check the journal around the stop for `ERROR` lines. |
| Start fails with `… only the vertical oscillator can run with …` | The sloshing oscillator and the wavemaker cannot run together: stop the one that runs first. Also shown for a moment while the oscillator unit is still starting (`runs 'unknown'`); try again a few seconds later. |
| Start fails with `port 8080 is in use` (or `8081`) | Something else serves the unit's web port, usually a `ros2 launch` started by hand. Stop it first. |

## Rig test

1. Start the service, then run a wave from the web page.
2. `sudo systemctl stop wavemaker@ladertanken` mid-wave. The wave stops and the drive is disabled.
   The journal shows the lifecycle manager's `Deactivate, cleanup, and shutdown nodes`, then a
   clean exit of every process.
3. Make `stop.sh` fail (for example `exit 1` at the top), start, and stop again: the drive must
   still be disabled, through SIGINT.
4. Idle auto-stop. The service always uses the 2 h default, so for a short test stop the service
   and launch by hand with `ros2 launch wavemaker_bringup lab.launch.py idle_timeout_s:=900`: the
   warning appears after 5 min, Keep running restarts the count, and at the timeout everything
   stops and the launch exits.
5. Reboot the Pi. `systemctl status wavemaker@ladertanken` reports `inactive (dead)`.

Earlier rig tests of particular changes (TODO 20-26, 38, 40; 2026-11-02 to 11-06) all passed;
their steps and results are in `git show c6bba03:rpi/README.md`.

## Oscillator

Two oscillators run on the ladertanken Pi as `oscillator@ladertanken`, user `ladertanken`:

- **sloshing:** the LinMot drive behind an Arduino Uno (`OSCILLATOR.md`).
- **vertical:** an IndraDrive on the ladertanken MGate, daisy-chained behind the wavemaker's
  drive (PROFIBUS address 3; [README](../README.md#running-the-vertical-oscillator),
  [COMMISSIONING.md](../COMMISSIONING.md#vertical-oscillator-ladertanken)).

`OSCILLATOR=` in `/etc/default/oscillator` picks one (`run.sh` passes it to the launch as
`oscillator:=`; empty or missing = `sloshing`), and the nodes run in `/oscillators/<name>`. To
switch, edit that line, then stop and start the unit. Its web page is on port 8081, the
wavemaker's on 8080.

Which rigs run together:
- **The vertical oscillator and the wavemaker:** yes, in either order. One soft stop button stops
  both.
- **The sloshing oscillator:** only alone. Each unit's `ExecStartPre` (`refuse_if_busy.sh`)
  refuses the start in both directions.
- **Sloshing and vertical:** never; they are the same unit.

> Like the wavemaker's, this unit never starts at boot. Starting it switches the drive on. The
> LinMot also homes (moves to its home stop, about 20 s) if its drive was powered off; the vertical
> oscillator's IndraDrive does not home, and its mains must be on at the cabinet.

### Install (once)

In addition to the wavemaker install above, as user `ladertanken` with the Uno plugged in:

```bash
# Uno: fixed name /dev/linmot_bridge, ModemManager kept off it
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/99-linmot-bridge.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger && sudo udevadm settle
ls -l /dev/linmot_bridge                # → /dev/linmot_bridge -> ttyACM0
id | grep -o dialout                    # → dialout (else: sudo usermod -aG dialout ladertanken, log in again)

# All three units (the wavemaker's has the ExecStartPre check too)
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/{oscillator,wavemaker,soft_stop}@.service /etc/systemd/system/
sudo systemctl daemon-reload
systemd-analyze verify {oscillator,wavemaker,soft_stop}@ladertanken.service   # no output = OK
```

Create `/etc/default/oscillator` with `sudoedit /etc/default/oscillator`, with **the same domain
as `/etc/default/wavemaker`** (the soft stop unit uses the wavemaker's; on another domain the
oscillator never hears the button and stays stopped):

```bash
# The lab's ROS domain. Required, and not 0.
ROS_DOMAIN_ID=1
PORT=8081
# Which oscillator: sloshing (default) or vertical. Nothing after the value.
OSCILLATOR=vertical
# The same MGate as in /etc/default/wavemaker. Required.
MGATE_HOST=192.168.1.70
```

Extend the sudoers rule (`sudo visudo -f /etc/sudoers.d/wavemaker`) to one line:

```
ladertanken ALL=(root) NOPASSWD: /usr/bin/systemctl start wavemaker@ladertanken, /usr/bin/systemctl stop wavemaker@ladertanken, /usr/bin/systemctl start oscillator@ladertanken, /usr/bin/systemctl stop oscillator@ladertanken
```

`sudo -l` should list all four; the soft stop unit needs none. The rule matches each command
exactly, so stop one unit per command: `sudo systemctl stop wavemaker@ladertanken oscillator@ladertanken` asks for the
password.

### Use

```bash
sudo systemctl start oscillator@ladertanken     # switches the drive on (the LinMot homes if needed)
sudo systemctl stop oscillator@ladertanken      # stops the motion and switches the drive off
journalctl -u oscillator@ladertanken -f -o cat
```

The page is at `http://ladertanken-rpi.local:8081/`. The height field is the stroke, peak to
peak, in metres: `0.01` moves the sloshing slide between 95 and 105 mm, the vertical slide ±5 mm
around mid-travel. On the vertical oscillator, Stop holds the slide where it is; *Return to
upright* brings it to mid-travel.

After the unit file changes (`git pull`), copy it again and `sudo systemctl daemon-reload`;
`run.sh` and `stop.sh` run from the repository.

If the controller crashed (a fault, a kill), `systemctl stop` takes about 45 s: the lifecycle manager waits
for the dead controller and is killed, and the unit ends `Failed with result 'exit-code'`. The next
start is normal. On the vertical oscillator the MGate's fault timeout has already switched the
drive off when the controller died (COMMISSIONING item 5).

### Rig test (sloshing)

1. Start; the journal shows `LinMot: homed` (if it homed), `Managed nodes are active` and
   `Soft stop released`. Run a wave from the page.
2. `sudo systemctl stop oscillator@ladertanken` mid-wave: the slide stops, the journal shows
   `Managed nodes have been shut down` and every process `finished cleanly`.
3. With the oscillator running, `sudo systemctl start wavemaker@ladertanken` fails, and its
   journal says `oscillator@ladertanken runs 'sloshing'; only the vertical oscillator can run
   with the wavemaker`. The other way round: `wavemaker@ladertanken is running; only the vertical
   oscillator can run with it`.
4. With `ros2 launch oscillator_bringup oscillator.launch.py` running by hand, the oscillator's
   start fails with `port 8081 is in use`.
5. After a reboot, all three units are `inactive (dead)`.

Passed 2026-10-23, with the messages of the time (one rig at a time, port 8080 for both).

### Rig test (wavemaker and vertical together)

`OSCILLATOR=vertical`, both drives' mains on, E-stop within reach.

1. `sudo systemctl start wavemaker@ladertanken`, then `sudo systemctl start oscillator@ladertanken`:
   both run, `systemctl is-active soft_stop@ladertanken` says `active`, and the journal of
   `soft_stop@` lists the three topics. Pages on 8080 and 8081.
2. Stop both, then start them the other way round: both run.
3. Press the soft stop during a wave on each page: both rigs stop and both pages show
   `SOFT_STOP`. Release: both accept a new wave.
4. Stop the wavemaker: the oscillator keeps running and the button still stops it.
5. Stop the oscillator: `soft_stop@ladertanken` is `inactive` a moment later.
6. Both running, `sudo systemctl stop soft_stop@ladertanken` (asks for the password; not in the
   sudoers rule): both rigs stop.
7. `OSCILLATOR=sloshing`: refused next to the running wavemaker, and the wavemaker refused next to
   it (see *Rig test (sloshing)*, step 3).

**Run 2026-11-11 on ladertanken, commit `f9ba5fc`: steps 1-6 passed.** All three units `active`,
`ip route get 192.168.1.70` with `rto_min lock 20ms`, the soft stop publishing to all three
topics. One press stopped both waves; with the wavemaker stopped the button still stopped the
oscillator; after the last rig `soft_stop@` was `inactive`; `systemctl stop soft_stop@...` stopped
both. Every wavemaker start after a stop needed F2074 cleared in IndraWorks (TODO 48; fixed the
same day with the MGate fault value `0x0002`). Step 7 waits for the sloshing rig (TODO 50, 2027).
