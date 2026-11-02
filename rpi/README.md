# Running a lab Pi as a service

`wavemaker@.service` runs `lab.launch.py` (controller, bridge, lifecycle manager, soft stop and
web page) under systemd; on the ladertanken Pi, `oscillator@.service` runs the sloshing
oscillator's `oscillator.launch.py` the same way (see [Oscillator](#oscillator)). Unlike a launch in an SSH session, it keeps running when the connection
drops, logs to the journal, and is started and stopped the same way on every Pi.

It is a template unit: the part after `@` is the wavemaker, which is also the Pi's user
(`wavemaker@ladertanken` runs as user `ladertanken`; see *Lab computers* in the top-level README).

| File | Purpose |
|---|---|
| `wavemaker@.service` | The unit. Installed as `/etc/systemd/system/wavemaker@.service`. |
| `oscillator@.service` | The oscillator's unit (ladertanken Pi only). Installed next to the wavemaker's. |
| `run.sh` | `ExecStart`: sources ROS and the workspace, then `exec`s `lab.launch.py`, or `oscillator.launch.py` when the unit sets `RIG=oscillator`. |
| `stop.sh` | `ExecStop`: asks the lifecycle manager to shut the nodes down before the launch is stopped. |
| `refuse_if_busy.sh` | `ExecStartPre` of both units: refuses to start while the other rig's service runs or the web port is in use (for example a launch by hand). |
| `99-linmot-bridge.rules` | udev rule for the oscillator's Arduino: `/dev/linmot_bridge`, and ModemManager kept off it. |

> **The service never starts at boot.** The unit has no `[Install]` section, so it cannot be
> enabled: the drive is enabled as soon as `lab.launch.py` runs, and a power cut or reboot must
> not do that by itself.

## Install (once per Pi)

Before you start, as the wavemaker user:

- The workspace is built (see *Build* in the top-level README), and `rpi/run.sh` and `rpi/stop.sh`
  are executable (`ls -l rpi/*.sh` shows `-rwxr-xr-x`).
- The user is in the `gpio` group (`id` lists `gpio`); the soft stop node needs it.
- No `lab.launch.py` is running by hand. A second copy would clash on port 8080 and GPIO17.

Install the unit:

```bash
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/wavemaker@.service /etc/systemd/system/
sudo systemctl daemon-reload
systemd-analyze verify wavemaker@ladertanken.service     # no output = OK
```

The unit sets `WorkingDirectory=/home/%i` because a service otherwise starts in `/`, which the user
cannot write to. lgpio, which the soft stop node uses for GPIO on the Pi 5, creates small files in
the working directory; without a writable one, opening GPIO17 fails and the soft stop stays engaged.

Create `/etc/default/wavemaker` with `sudoedit /etc/default/wavemaker`:

```bash
# The lab's ROS domain. Required, and not 0.
ROS_DOMAIN_ID=1
PORT=8080
# Water depth in m. Empty uses wavemakers.yaml.
WATER_DEPTH=
```

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

`sudo -l` should list it. Then start the service once and check the journal (see *Use*): it should
show `Managed nodes are active`, `Soft stop connected` and, with the button at rest,
`Soft stop released`.

## Update

After a `git pull`:

- **Code changed** (anything outside `rpi/`): `cd ~/ros_ws && colcon build`.
- **`wavemaker@.service` changed:** copy it again and run `sudo systemctl daemon-reload`.
- **`run.sh` or `stop.sh` changed:** nothing to do; the unit runs them from the repository.

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
journalctl -u wavemaker@ladertanken -b --no-pager -o cat | grep -E "ERROR|WARN|died|exception|Soft stop"
```

| Symptom | Cause |
|---|---|
| Fails at once with `set ROS_DOMAIN_ID` | `/etc/default/wavemaker` missing, or `ROS_DOMAIN_ID` empty or 0. |
| `Soft stop released` never appears, and the button changes nothing | `soft_stop` could not open GPIO17 (`Cannot open GPIO17` in the journal): check `WorkingDirectory=` in the installed unit and the `gpio` group. If GPIO is fine, the button is pressed in. |
| `ros2` commands on another computer do not see the wavemaker | That computer uses another ROS domain or another DDS (`RMW_IMPLEMENTATION`). The Pi uses the default, Fast DDS. |
| `status` shows `failed` after a stop | Check the journal around the stop for `ERROR` lines. |
| Start fails with `… is running; stop it first` | The other rig on this Pi (wavemaker or oscillator) runs. Stop it with the command shown. |
| Start fails with `port 8080 is in use` | Something else serves the web port, usually a `ros2 launch` started by hand. Stop it first. |

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

### Goal type per goal and `play_dat` (TODO 26; written 2026-11-01, first run planned 2026-11-02)

Checks the change that lets every goal pick its type (`mode_pregenerated` removed) and
`play_dat` running next to the web page. Keep a hand near the e-stop: these are the first goals
from this code on the drive. Run `play_dat` on the Pi (ssh), where the workspace is built; from
another machine it needs `ROS_DOMAIN_ID=1` too.

0. **Code on the Pi.** The changes must be committed and pushed on `feat/oscillator` first. On
   the Pi:

   ```bash
   cd ~/ros_ws/src/hydrolab_rigs && git fetch && git checkout feat/oscillator && git pull
   git log --oneline -1                      # the commit you expect
   cd ~/ros_ws && colcon build
   ```

   Expected: `Summary: ... packages finished`, no failures.
1. **Start.** `sudo systemctl start wavemaker@ladertanken`, then:

   ```bash
   journalctl -u wavemaker@ladertanken -b --no-pager -o cat | grep -E "ERROR|was removed|renamed"
   ```

   Expected: no output. "Parameter mode_pregenerated was removed" means a yaml file or launch
   command still sets it. The page at `http://ladertanken-rpi.local:8080/` shows *idle*.
2. **Regular wave from the page** (the bridge's goal, now without the mode parameter): height
   0.02 m, period 1.0 s, Start. Expected: the wave runs and the plot follows. Stop: the page
   shows "Stopped, holding position." Then Return to upright.
3. **Test file and dry run** (on the Pi):

   ```bash
   source ~/ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=1
   python3 ~/ros_ws/src/hydrolab_rigs/rig_controller/test/make_test_dat.py /tmp/test_runs.dat
   ros2 run rig_controller play_dat /tmp/test_runs.dat --min-m -0.146 --max-m 0.146 \
     --max-velocity-mps 0.54 --dry-run
   ```

   Expected: `6 runs, 67.5 s, max |x| 0.020 m, peak speed 0.157 m/s`, then runs 0-5, all
   `regular`, status `OK`, exit 0. (±0.146 m is ±275° at 0.00053 m/deg; 0.02 m at the
   attachment is about ±38° at the drive.)
4. **One run with the page open:**

   ```bash
   ros2 run rig_controller play_dat /tmp/test_runs.dat --min-m -0.146 --max-m 0.146 \
     --max-velocity-mps 0.54 --runs 1 --ros-args -r __ns:=/wavemakers/ladertanken
   ```

   Expected: after the 3 s gap, `run 1 (regular): 8.0 s, max 0.0200 m, 0.126 m/s`; the paddle
   ramps up, makes 8 waves of 1.0 s and stops at upright; the page plots it (the status stays
   *idle*, it shows the bridge's state); exit 0. In the journal: no "Rejecting goal".
5. **Stop on the page during a run.** Start `--runs 3` (12 s) as above and press Stop after a few
   seconds. Expected: the paddle stops and holds; `play_dat` logs `run 3: ended with status 5;
   stopping` and exits 5; the journal shows the bridge's `Stop: 1 goal(s) canceling`. Return to
   upright from the page.
6. **Soft stop during a run.** Start `--runs 3` again and press the soft stop button. Expected:
   the paddle halts, `play_dat` logs `ended with status 6` and exits 5, the journal shows the
   soft stop. While the button is pressed a new `play_dat` exits 4 ("goal rejected"). Release,
   Return to upright, and `--runs 1` plays again.
7. **All runs and Ctrl-C.** Play the whole file (no `--runs`, about 68 s) and watch for 2-3 runs,
   then Ctrl-C. Expected: the paddle stops within a moment, `play_dat` exits 130, the journal
   shows the cancel.
8. **Idle timer counts `play_dat`.** Stop the service and launch by hand with a short timeout:

   ```bash
   sudo systemctl stop wavemaker@ladertanken
   source ~/ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=1
   ros2 launch wavemaker_bringup lab.launch.py idle_timeout_s:=900
   ```

   Note the time. After about 4 min, play the whole file (68 s) in a second ssh session.
   Expected: no idle warning at 5 min (the run resets the count); the warning appears about
   5 min after the run *ended*. Press Keep running (or Ctrl-C the launch) and start the
   service again.

If all of this passes, merge `feat/oscillator` into `master` (TODO 26). Note the date and any
deviation here.

**Run 2026-11-02 on ladertanken, commit `e096801`: all steps passed.** Step 1: no errors since
the 12:46 start (older F2026/F2074/PROFIBUS lines in `journalctl -b` were from 26-28 October).
Step 3 printed the expected table. Steps 5-7 exited 5, 5 (then 4 while held) and 130. Step 8:
launch up 13:02:44, whole file played 13:06:43 to about 13:07:50 (run 5 started 13:07:42, about
1 s later than the file's 57.4 s after run 0), no warning at 13:07:44, warning at 13:12:50, 5 min
after the run ended. With a hand-started launch the warning is only in that terminal and on the
page, not in `journalctl`.

## Oscillator

The sloshing oscillator (LinMot drive behind an Arduino Uno, see `OSCILLATOR.md`) runs on the
ladertanken Pi as `oscillator@ladertanken`, user `ladertanken`. It shares the web port, the soft
stop button on GPIO17 and the ROS domain with the ladertanken wavemaker, so only one of the two
can run: each unit's `ExecStartPre` (`refuse_if_busy.sh`) refuses to start while the other runs.

> Like the wavemaker's, this unit never starts at boot. Starting it switches the drive on, and the
> slide homes (moves to its home stop, about 20 s) if the drive was powered off.

### Install (once)

In addition to the wavemaker install above, as user `ladertanken` with the Uno plugged in:

```bash
# Uno: fixed name /dev/linmot_bridge, ModemManager kept off it
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/99-linmot-bridge.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger && sudo udevadm settle
ls -l /dev/linmot_bridge                # → /dev/linmot_bridge -> ttyACM0
id | grep -o dialout                    # → dialout (else: sudo usermod -aG dialout ladertanken, log in again)

# Both units (the wavemaker's has the ExecStartPre check too)
sudo cp ~/ros_ws/src/hydrolab_rigs/rpi/oscillator@.service \
        ~/ros_ws/src/hydrolab_rigs/rpi/wavemaker@.service /etc/systemd/system/
sudo systemctl daemon-reload
systemd-analyze verify oscillator@ladertanken.service wavemaker@ladertanken.service   # no output = OK
```

Create `/etc/default/oscillator` with `sudoedit /etc/default/oscillator`, with the same domain as
`/etc/default/wavemaker`:

```bash
# The lab's ROS domain. Required, and not 0.
ROS_DOMAIN_ID=1
PORT=8080
```

Extend the sudoers rule (`sudo visudo -f /etc/sudoers.d/wavemaker`) to one line:

```
ladertanken ALL=(root) NOPASSWD: /usr/bin/systemctl start wavemaker@ladertanken, /usr/bin/systemctl stop wavemaker@ladertanken, /usr/bin/systemctl start oscillator@ladertanken, /usr/bin/systemctl stop oscillator@ladertanken
```

`sudo -l` should list all four.

### Use

```bash
sudo systemctl start oscillator@ladertanken     # switches the drive on, homes if needed
sudo systemctl stop oscillator@ladertanken      # stops the motion and switches the drive off
journalctl -u oscillator@ladertanken -f -o cat
```

The page is at `http://ladertanken-rpi.local:8080/`. The height field is the stroke, peak to
peak, in metres: `0.01` moves the slide between 95 and 105 mm.

### Rig test

1. Start; the journal shows `LinMot: homed` (if it homed), `Managed nodes are active` and
   `Soft stop released`. Run a wave from the page.
2. `sudo systemctl stop oscillator@ladertanken` mid-wave: the slide stops, the journal shows
   `Managed nodes have been shut down` and every process `finished cleanly`.
3. With the oscillator running, `sudo systemctl start wavemaker@ladertanken` fails, and its
   journal says `oscillator@ladertanken is running; stop it first`. The same the other way round.
4. With `ros2 launch oscillator_bringup oscillator.launch.py` running by hand, both starts fail
   with `port 8080 is in use`.
5. After a reboot, both units are `inactive (dead)`.

Passed 2026-10-23.
