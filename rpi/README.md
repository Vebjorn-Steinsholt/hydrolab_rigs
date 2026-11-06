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
- `python3-matplotlib` is installed (`sudo apt install python3-matplotlib`); `generate_waves` needs
  it for its plot.
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
   ros2 run rig_controller play_dat /tmp/test_runs.dat ladertanken --dry-run
   ```

   Expected: `limits from wavemakers.yaml (ladertanken): -0.14575..+0.14575 m, 0.7632 m/s, upright
   +0.0000 m`, `namespace /wavemakers/ladertanken`, `6 runs, 67.5 s, max |x| 0.020 m, peak speed
   0.157 m/s`, then runs 0-5, all `regular`, status `OK`, exit 0. (±0.146 m is ±275° at
   0.00053 m/deg; 0.02 m at the attachment is about ±38° at the drive.)
4. **One run with the page open:**

   ```bash
   ros2 run rig_controller play_dat /tmp/test_runs.dat ladertanken --runs 1
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

### `play_dat` limits from the yaml (TODO 40; run 2026-11-04)

Checks that `play_dat` takes its limits, upright and namespace from the rig's name, and that the
cross-check against the running controller works on the real drive. Keep a hand near the e-stop.

0. **Code on the Pi.** `git status --short` empty, then:

   ```bash
   sudo systemctl stop wavemaker@ladertanken
   cd ~/ros_ws/src/hydrolab_rigs && git fetch && git checkout feat/wave-generation && git pull
   git log --oneline -1                      # the commit you expect
   cd ~/ros_ws && colcon build
   ```

1. **Start.** `sudo systemctl start wavemaker@ladertanken`; no `ERROR` in the journal, the page
   shows *idle*.
2. **Test file** (on the Pi):

   ```bash
   source ~/ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=1
   mkdir -p /tmp/rigcheck && cd /tmp/rigcheck
   printf 'kind,period_s,height_m,n_periods,n_ramp,pause_s\nregular,1.0,0.03,10,2,3\nregular,1.3,0.05,10,2,3\n' > matrix.csv
   ros2 run rig_controller generate_waves ladertanken matrix.csv -o test_waves --no-plot
   ```

   Expected: `2 runs, 0 skipped`, and the command `play_dat test_waves.dat ladertanken --dry-run`.
3. **Dry run by name:** `ros2 run rig_controller play_dat test_waves.dat ladertanken --dry-run`.
   Expected: `limits from wavemakers.yaml (ladertanken): -0.14575..+0.14575 m, 0.7632 m/s, upright
   +0.0000 m`, `namespace /wavemakers/ladertanken`, runs 0 and 1 `OK`.
4. **Cross-check and one run:** the same without `--dry-run`, with `--runs 0`. Expected: `limits
   match the controller` before `run 0 (regular): 10.0 s, max 0.0170 m, 0.107 m/s`; 10 waves,
   back to upright.
5. **A difference only warns.** A copy of the yaml with a lower speed limit:

   ```bash
   sed 's/max_velocity_rpm: 240.0/max_velocity_rpm: 200.0/' \
     ~/ros_ws/install/wavemaker_bringup/share/wavemaker_bringup/config/wavemakers.yaml > alt.yaml
   ros2 run rig_controller play_dat test_waves.dat ladertanken --config alt.yaml --runs 0
   ```

   Expected: `0.6360 m/s` in the limits line, `yaml and controller differ: max_velocity_mps:
   alt.yaml 0.6360, controller 0.7632`, and the run still starts. Ctrl-C (exit 130), Return to
   upright from the page.
6. **The old form:** `play_dat test_waves.dat --min-m -0.146 --max-m 0.146 --max-velocity-mps 0.54
   --dry-run`. Expected: `limits from the command line: -0.14600..+0.14600 m, 0.5400 m/s, upright
   +0.0000 m`, no namespace line, both runs `OK`.

Afterwards put the Pi back on `master` (`git checkout master && git pull`, `colcon build`) once
the branch is merged.

**Run 2026-11-04 on ladertanken, commit `8a458eb`: all steps passed.** Steps 2-3 printed the same
file, limits and table as on the development PC. Step 4: limits matched the controller and run 0
played. Step 5: the warning at 15:19:16, run 0 started 2 s later and was stopped with Ctrl-C.
Step 6 as expected. The oscillator was not run (its limits are covered by the tests only).

### `generate_waves` on the rig (TODO 38; run 2026-11-05)

Generates a test matrix on the Pi and plays it on ladertanken: two regular runs, one close to the
travel limit, and the first irregular run on the rig. Keep a hand near the e-stop.

1. **Pi up to date and the water depth set.** Measure the depth; below, 0.98 stands for the measured
   value. With the Pi on `master`, run `git pull` and `colcon build`. Then
   `sudoedit /etc/default/wavemaker`, set `WATER_DEPTH=0.98` with nothing after the number, and stop
   and start the service. Check the start:

   ```bash
   journalctl -u wavemaker@ladertanken --since "-10min" --no-pager -o cat | grep -E "ERROR|WARN"
   ros2 param get /wavemakers/ladertanken/controller water_depth    # Double value is: 0.98
   ```

   Expected: no `ERROR` after the start, and the page shows *idle*. `journalctl -b` also shows
   every earlier start since the Pi booted. The controller only logs `water_depth` when the value is
   invalid, so check it with `ros2 param get`.
2. **Matrix and file**, on the Pi in `tmux`. `tmux` opens a new shell, so set the variables inside
   it:

   ```bash
   source ~/ros_ws/install/setup.bash
   export ROS_DOMAIN_ID=1
   D=0.98                                    # the same as WATER_DEPTH
   mkdir -p ~/wave_sessions/2026-11-05_item38 && cd ~/wave_sessions/2026-11-05_item38
   cat > matrix.csv <<'END'
   kind,period_s,height_m,steepness,n_periods,hs_m,tp_s,gamma,duration_s,seed,n_ramp,pause_s
   regular,1.0,,1/60,30,,,,,,3,60
   regular,1.5,,1/40,30,,,,,,3,60
   regular,1.3,0.171,,30,,,,,,3,60
   irregular,,,,,0.03,1.2,3.3,120,1,5,60
   END
   ros2 run rig_controller generate_waves ladertanken matrix.csv -o item38 --water-depth $D
   ```

   Expected: `4 runs, 0 skipped, 7.9 min`. Run 2 needs 0.144 m of the 0.146 m travel at 0.95 m, and
   0.132 m at 1.0 m. At a shallower depth it may be skipped, and `item38.csv` says why. The plot
   needs `python3-matplotlib` (`sudo apt install python3-matplotlib`); without it, use `--no-plot`.
   Copy `item38.png` to the PC with `scp`.
3. **Dry run, then each run in full**, with the page open: `play_dat item38.dat ladertanken
   --dry-run`, then `--runs 0` to `--runs 3`, one at a time. Expected: the dry run lists runs 0-2
   `regular` and run 3 `irregular`, all `OK`. Each play logs `limits match the controller`, then
   `run N (...)`, and exits 0. Before run N, `play_dat` first waits for the rest that comes before
   it in the file. Here that is about 60 s for runs 1-3, and nothing is logged while it waits.
4. **Stops**, after the tank has calmed. `--runs 0` and Stop on the page after a few waves: the
   paddle holds, `play_dat` exits 5; Return to upright. `--runs 1` and the soft stop: the paddle
   halts, exit 5, and while the stop is held a new `play_dat` exits 4. Release the stop and Return to
   upright.
5. **Pass:** every run accepted and no faults, and the page's plot of each run matches the `.png`.
   Measure the wave heights with probes (TODO 43).

**Run 2026-11-05 on ladertanken, Pi on `master`: steps 1-4 passed. The wave heights were not
measured, because no probes were available (TODO 43).** Water depth 0.98 m; the gain table is for
1.0 m, and `generate_waves` warned about it. Step 1: no errors after the 08:42 start. The F2026,
F2074 and PROFIBUS lines in `journalctl -b` came from 26 October to 05 November 08:21. Step 2: `4
runs, 0 skipped, 7.9 min`. In the plot, run 2 reached ±0.137 m of ±0.146 m, at 0.66 of 0.77 m/s.
`python3-matplotlib` was installed on the Pi during this test. Step 3: dry run all `OK`. Runs 0-3
were all played in full, and run 3 ramped in and ended at upright. Step 4 as expected. At 09:32:07,
mid-session, the controller logged `No Soft stop message received for over 200 ms`, and the soft
stop was triggered and released 5 s later (TODO 36).

**Probes, 2026-11-05 about 12:14-12:21: `item38.dat` played whole, WP1 3 m from the flap,
depth assumed 0.98 m.** Recorded in catman, `item38.bin` (200 Hz, 456 s; the recording starts about
26 s before run 0 and stops 14 s after run 3). Analysed with `wave_analysis`
(`sessions/2026-11-05_item38.toml`): measured / target H was 1.03 for run 0 (1.0 s, 1/60), **1.12
for run 1** (1.5 s, 1/40, measured before the beach reflection; 1.16 with it) and 1.05 for run 2
(1.3 s, 0.171 m; second harmonic 10 %). So runs 0 and 2 pass and run 1 fails the ±6 % limit. Compared with `Calib20oct.BIN` (WP1 also at 3 m): the
paddle sensor measured 0.572 of the commanded amplitude over the steady part, as in October
(with `ramp = 3` in the session; without it the 3-period ramps pull the fit to 0.55), and the wave
height per paddle amplitude matched October within 3 % at 1.0, 1.3 and 1.5 s. Run 1 was 1.08-1.12
at the start and rose to 1.16-1.19 about 21 s in, when the beach reflection arrived; October's
1.5 s run lasted 15 s, so it never saw that. The rest of the difference is in the command: per
metre of target height, run 1 commanded 8 % more stroke than October's 1.5 s run (0.993 against
0.918), 4 % from the gain table (0.75 now, 0.78 then) and 4 % from the 0.98 m depth. Runs 0 and 2
stayed flat within 2 %. Irregular run 3: Hm0 0.0347 m (target 0.030; about
±10 % from the 104 s record), H1/3 0.0324 m, Tp 1.18 s. Before each run the probe was still to
within 1.3 mm, so the 60 s pauses are long enough.

### Controller and driver fixes (TODO 20-25; run 2026-11-06)

Checks the fixes on branch `bugfix-items-20-25` on the drive before they are merged: the start
blend over whole periods (25), the `return_to_upright` replies (24), `cancel_all_goals` right
after start (22), the hold after Stop (21), and the fault text (20). Keep a hand near the e-stop.

0. **Code on the Pi.** Push the branch from the PC, then on the Pi:

   ```bash
   sudo systemctl stop wavemaker@ladertanken
   cd ~/ros_ws/src/hydrolab_rigs && git fetch && git checkout bugfix-items-20-25 && git pull
   cd ~/ros_ws && colcon build               # rig_interfaces changed: ReturnToUpright has SOFT_STOP
   sudo systemctl start wavemaker@ladertanken
   journalctl -u wavemaker@ladertanken -f -o cat     # in a second ssh window
   ```

   For the terminal steps: `source ~/ros_ws/install/setup.bash` and
   `export ROS_DOMAIN_ID=1 NS=/wavemakers/ladertanken`.
1. **`cancel_all_goals` (22).** Straight after `sudo systemctl stop wavemaker@ladertanken && sudo
   systemctl start wavemaker@ladertanken`:
   `ros2 service call $NS/cancel_all_goals rig_interfaces/srv/CancelAllGoals "{requester: cli}"`.
   Expected: `status=1` (`NO_ACTIVE_GOALS`), not `2`. Again with a wave running (1.0 s, 0.026 m):
   `status=0`, `goals_canceling=1`, the wave stops.
2. **Hold after Stop (21).** 1.3 s, 0.085 m; Stop mid-stroke. Expected: the paddle holds where it
   is, no fault, the next wave starts normally.
3. **Start blend far from upright (25).** 1.5 s, 0.108 m, Stop near a crest (about 0.1 m off
   upright); then 0.7 s, 0.026 m. Expected: accepted, and it blends in smoothly over 3-4 periods
   (2-3 s), not one. From upright the same wave blends in over one period. To watch the setpoint:
   `ros2 topic echo $NS/wavemaker_setpoint std_msgs/msg/Float64 --field data` (without the type,
   `ros2 topic echo` can fail with `Could not determine the type`).
4. **Soft stop during a return (24).** Off upright, *Return to upright*, and the soft stop during
   the move. Expected: the page shows `SOFT_STOP`, the journal `Soft stop engaged during return to
   upright`, the drive stays enabled. After the release, a second return ends in `SUCCESS`.
5. **Fault during a return (20, 24), optional.** Off upright, start a return and switch the drive's
   power off. Expected: `ACTUATOR_FAULT` with the drive's reason. Power on, stop and start the
   service.

**Run 2026-11-06 on ladertanken, branch `bugfix-items-20-25` (code as in `ca87b35`): all steps
passed, including step 5.** Step 0: clean starts at 08:25 (drive `SW=0xC81E OM AF inpos`,
`slave=live`). Step 1: `status=1` with the call waiting for the service, so it was answered as soon
as the service existed; with a wave, `status=0`, `goals_canceling=1`. Step 2: held, and a new wave
started at once. Step 3 as expected. Step 4: `Soft stop engaged during return to upright` 15 ms
after the button, page `SOFT_STOP`, then `SUCCESS`. Step 5: the return loop logged `Actuator fault
detected`, then `Driver fault: drive error F2026` (undervoltage in the power section); the page
showed `ACTUATOR_FAULT: Drive error F2026`, and the controller went through `on_error` to
unconfigured. After that the lifecycle manager logged `SERVER controller IS DOWN` (the bond is
broken on leaving active) and, while resetting, `No transition matching 4/2 found for current state
unconfigured`: the controller had already left through `on_error`. Unchanged by this branch.
Also seen: at every start the controller logged `Soft stop connected` only after the activation
ended, although the soft stop published its release during it (TODO 36, `sketch.md`).

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
