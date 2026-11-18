# Sketch: the oscillator's centre from `/etc/default/oscillator` (TODO 54)

What to code so an experiment can oscillate around another centre without editing the tracked
`oscillator.yaml` on the Pi, and why. Was T19 in the vertical oscillator's
sketch (done; `git show c6bba03:oscillator_sketch.md`).

**Status (2026-11-18): code, tests and docs done.** Launch argument checked by hand
(`ros2 param get` shows the override, and the yaml value without it). Tests:
`rig_controller/test/test_oscillator_centre.py`. Left: *Check on the rig* below; then retire this
sketch and TODO 54.

## Problem

Some experiments oscillate around another point than 0. Today that means editing
`upright_position_m` in the tracked `oscillator.yaml` on the Pi and rebuilding: `git pull` then
fails on the edited file, and the value is easy to leave behind for the next user. The wavemaker
already does this for the depth (`WATER_DEPTH` → `water_depth:=` → a parameter override in
`wavemakers.launch.py`, `_launch_setup`); do the same for the oscillator's centre.

**What changes for the user.** One line in `/etc/default/oscillator`, then a service restart:

```bash
# Centre of oscillation in m, + = up for vertical. Empty uses oscillator.yaml.
VERTICAL_UPRIGHT_M=0.050
SLOSHING_UPRIGHT_M=
```

One variable per oscillator, not one `UPRIGHT_M`: `OSCILLATOR=` switches between the two in the
same file, and a vertical centre of 0.05 m is a valid but wrong sloshing centre (inside its
0.015..0.185 m travel), so a shared name would carry over silently.

## Decisions

1. **Override at launch, not a yaml edit.** Same path as `WATER_DEPTH`: the controller sees
   `upright_position_m` as an ordinary parameter, and all its configure checks (inside the travel)
   and every user of the offset (waves, `return_to_upright`, files' travel checks) stay as they are.
   No change in `controller_node.cpp`.
2. **Empty keeps the yaml.** An unset or empty variable passes nothing, so today's behaviour is
   the default.
3. **`play_dat` stays as it is** (recommended). It checks files against the yaml's upright and
   already warns when the running controller's differs (`cross_check_limits`, compares
   `upright_m` beyond 0.1 mm). With an override the user passes `--upright-m 0.050`, which exists.
   Alternative, later: take the upright from the running controller when it answers. That changes
   the rule "files are checked against the yaml", so decide separately.

## Changes

1. **`oscillator_bringup/launch/oscillator.launch.py`.** Declare the argument, and build the
   controller in an `OpaqueFunction`, as `wavemakers.launch.py` does, so an empty value can be left
   out (a `ParameterValue` of `''` as a float fails):

   ```python
   from launch.actions import OpaqueFunction

   def _controller(context):
       overrides = {}
       upright = context.launch_configurations['upright_position_m']
       if upright:
           overrides['upright_position_m'] = float(upright)
       return [LifecycleNode(
           package='rig_controller', executable='controller_node',
           namespace=['oscillators/', LaunchConfiguration('oscillator')], name='controller',
           parameters=[config_file, {'bond_timeout': bond_timeout}, overrides],
           output='screen')]

   # in generate_launch_description(), next to the other arguments:
   DeclareLaunchArgument(
       'upright_position_m', default_value='',
       description='Centre of oscillation in m for this run; empty uses oscillator.yaml'),
   # and in place of the controller's LifecycleNode:
   OpaqueFunction(function=_controller),
   ```

   `config_file` and `bond_timeout` move to module level, or into `_controller`. The bridge,
   manager, soft stop and web page are unchanged.

2. **`rpi/run.sh`, the `oscillator)` case.** Pick the chosen oscillator's variable and log it, so
   the journal says which centre a session ran with:

   ```bash
   oscillator)
     osc="${OSCILLATOR:-sloshing}"
     launch=(oscillator_bringup oscillator.launch.py)
     args=(port:="${PORT:-8081}" oscillator:="$osc" soft_stop:=false)
     upright_var="${osc^^}_UPRIGHT_M"           # VERTICAL_UPRIGHT_M or SLOSHING_UPRIGHT_M
     if [[ -n "${!upright_var}" ]]; then
       args+=(upright_position_m:="${!upright_var}")
       echo "$upright_var=${!upright_var}: centre from /etc/default/oscillator, not oscillator.yaml"
     fi
     ;;
   ```

   The unit file is unchanged (`EnvironmentFile=/etc/default/oscillator` passes every line).

3. **Nothing in the controller**, as decision 1. It already refuses a centre outside the travel at
   configure (`upright_position_m must be within travel_minimum_m and travel_maximum_m`), and the
   service then fails to start: the journal shows why.

## Tests and docs (Claude)

- `test_launch_soft_stop.py`'s pattern for the launch: with `upright_position_m:=0.05` the
  controller gets the override; with it empty, no `upright_position_m` beyond the yaml's.
- `run.sh` with a stub `ros2` (as `test_units_rto_min.py` stubs `ip`): `OSCILLATOR=vertical` with
  `VERTICAL_UPRIGHT_M=0.05` passes `upright_position_m:=0.05`; `SLOSHING_UPRIGHT_M` is ignored
  then; both empty pass nothing.
- `rpi/README.md`: the two lines in the `/etc/default/oscillator` example; under *Oscillator → Use*,
  changing the centre (edit, restart, *Return to upright* before the first wave, `play_dat
  --upright-m`), and that a stroke must fit in 2 × (200 mm − |centre|) on the vertical.

## Check on the rig

1. `VERTICAL_UPRIGHT_M=0.050`, restart: the journal shows the `run.sh` line;
   `ros2 param get /oscillators/vertical/controller upright_position_m` → `0.05`.
2. *Return to upright* moves the slide to +50 mm; a 20 mm wave at 2 s from the page runs between
   +40 and +60 mm (bag or IndraWorks).
3. `play_dat <file> vertical --rig oscillator --dry-run` warns that the controller's upright differs;
   with `--upright-m 0.050` it does not.
4. `VERTICAL_UPRIGHT_M=0.250` (outside the travel): the service fails at configure with the travel
   message; nothing moves.
5. Empty again, restart: back to 0.
