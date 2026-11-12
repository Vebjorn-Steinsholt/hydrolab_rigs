# Sketch: towing carriage

What to code for the carriage, and why. The carriage does two things:

- **Tow:** a run to an end position at a set speed and acceleration, with position and speed
  feedback. The new `GoToPosition` action ([TODO.md, *Go to position*](TODO.md#go-to-position)).
- **Oscillate:** as the vertical oscillator, through the existing `MoveRig` action.

Everything on the rig (drive parameters, gateway setup, test procedures, results) goes in
COMMISSIONING.md under a *Carriage* section once there is hardware.

Line numbers are as of `feat-carriage` on 2026-11-12.

## Order

| Task | What | Needs |
|---|---|---|
| C1 | Rig name `carriage` accepted (Rig-agnostic configuration, steps 1-2) | - |
| C2 | `GoToPosition`: interface, profile, controller (Go to position, steps 1-4) | - |
| C3 | IndraDrive: acceleration in the cyclic outputs | the drive's telegram changed (C7) |
| C4 | `IndraDriveActuator`: native moves | C2, C3 |
| C5 | `carriage.yaml` cleaned up | the drive and the gateway known |
| C6 | `carriage_bringup` as a package, with a launch | C1 (or C1's step 3) |
| C7 | On the rig: telegram, MGate block, tests | hardware |

C1 and C2 are on every rig and can start now. C3 and C4 can be coded and tested against the fake
before the hardware exists.

## Decisions

1. **One action for towing and go-to-position** (`GoToPosition`), not a separate tow action. A
   tow is a move to an end position with a speed and an acceleration; the carriage adds only
   what a tow needs: measured `velocity`, `elapsed_time` and `phase` in the feedback.
2. **Acceleration in the cyclic telegram** (2026-11-12). The IndraDrive today gets only a target
   and a speed cap; its acceleration is a fixed drive parameter. A streamed profile (a new target
   every 20 ms) gives speed ripple during the cruise, the part a tow is for. With S-0-0260 in the
   outputs, a tow is one command and the drive makes the profile. Rejected: ramping the speed cap
   (the controller times the braking on feedback one cycle old) and a fixed acceleration in
   IndraWorks (no acceleration per tow).
3. **Oscillating stays on `MoveRig`** with the `stroke` motion model (today's oscillator). Around
   the carriage's position at the goal, not only `upright_position_m`, comes later (TODO.md,
   end of *Rig-agnostic configuration*).
4. **Tow from the CLI first.** The web page gets nothing new until towing works; a tow page may
   come later on the same action.

## C1: rig name `carriage`

Today `rig` must be `wavemaker` or `oscillator`:

- `controller_node.cpp:161-164`, `on_configure()`;
- `bridge_node.cpp:57-60`, `on_configure()`;
- `RIG_TEXTS` and `web_server.py:120`; `web.launch.py`, `demo.launch.py`; the `case` in
  `rpi/run.sh` / `rpi/stop.sh`.

Do [TODO.md, *Rig-agnostic configuration*](TODO.md#rig-agnostic-configuration) steps 1 and 2 as
written there: `motion_model` (`stroke` for the carriage), then a free rig name. `carriage.yaml`
then has `rig: carriage` and `motion_model: stroke`, and the actions are `move_carriage` and
`go_to_position` under `/carriage/<tank>/`.

## C2: `GoToPosition`

As in [TODO.md, *Go to position*](TODO.md#go-to-position), steps 1-4, with the tow additions
already written there:

- Feedback: `position`, `velocity`, `remaining`, `elapsed_time`, `phase`
  (`ACCELERATING`, `CRUISING`, `BRAKING`, `SETTLING`).
- Rejections: `max_tow_velocity_mps` (when set), and a braking distance
  `max_velocity² / (2 max_acceleration)` that does not fit before the travel limit.
- `velocity` for the feedback: `Actuator` has no measured velocity today. Add
  `virtual double actual_velocity_mps() const` next to `actual_position_m()`
  (`actuator.hpp`); the IndraDrive reads S-0-0040 (input bytes 6-9, already in the telegram),
  the LinMot its actual velocity.
- `phase` on a native move: from the measured velocity against `max_velocity` (CRUISING within a
  band, for example 2 %), and BRAKING once `remaining` is below the braking distance.

## C3: IndraDrive acceleration

`drivers/mgate5101_driver`:

- **Layout** (`include/mgate/indradrive.h:5-12`): outputs grow from 12 to 16 bytes:

  ```
  //   12 S-0-0260 pos. acceleration I32
  ```

- **Tag** (`src/indradrive.cpp`, next to `PosVelocity` at line 44): `PosAcceleration`, Output,
  I32, offset 12, big-endian. The scale comes from the drive's acceleration scaling (S-0-0160);
  read it in IndraWorks before setting the factor.
- **Config** (`IndraDriveConfig`, `indradrive.h:59`): `max_acceleration` with the drive's limit;
  `move_to()` refuses above it, as for the velocity.
- **`move_to(position, velocity, acceleration)`** (`src/indradrive.cpp:235`): write the
  acceleration first, then the velocity, then the target, so a cycle between the writes never
  pairs a new target with an old limit (same rule as the comment at line 250). Keep the
  two-argument `move_to()` for the wavemaker and the vertical oscillator; it leaves the
  acceleration out.
- **Enable** (`src/indradrive.cpp:209` writes a safe `PosVelocity`): also write a safe
  acceleration, so the first command after enable never runs with 0.
- **Only when the telegram has it.** A config flag (`has_acceleration_output`, default false) so
  the wavemaker's and the oscillator's 12-byte telegrams keep working; `move_to()` with an
  acceleration fails when the flag is off.

To check in the MPx04 manual: that S-0-0260 is used in drive-internal interpolation (S-0-0032)
as written in the cyclic data, and which deceleration it uses (S-0-0260 for both, or a separate
S-0-0359).

## C4: `IndraDriveActuator` native moves

`rig_controller/src/indradrive_actuator.cpp/.hpp`:

- `supports_native_move()`: true when the drive config has the acceleration output.
- `move_absolute(target)`: `indradrive_->move_to(target.position, |velocity| in rpm units,
  |acceleration| in drive units)`; store `last_target_` as `write_setpoint()` does (line 245).
- `native_move_done()`: status word `kTargetReached` and the position within
  `in_position_tol_deg` of `last_target_` (as `IndraDrive::wait_in_position()`, without waiting).
- `max_acceleration_mps2()` (line 284, infinity today): the drive's limit when it has the output.
- `actual_velocity_mps()` (C2).

`to_actuator_setpoint()` (line 321) already converts the acceleration with `rate_to_drive()`;
check that the drive's acceleration unit matches (the velocity has its own
`velocity_per_unit_per_s`; the acceleration probably needs an `acceleration_per_unit_per_s2`).

## C5: `carriage.yaml`

It is a copy of the vertical oscillator's block. Fix:

- `rig: lilletanken` → `rig: carriage` (C1), and add `motion_model: stroke`.
- `drive_motion: angular` but the comments and the travel are linear → `linear`.
- `driver_address`, the slave, the blocks (`input_base_reg`, `output_base_reg`) and
  `profibus_address`: from the carriage's own gateway setup. The comments are still the
  oscillator's.
- `output_word_count: 8` (16 bytes, C3).
- The scaling (`drive_m_per_unit`, `drive_velocity_per_unit_per_s`, `drive_direction`) and the
  limits (`travel_minimum_m`, `travel_maximum_m`, `upright_position_m`, `max_velocity_rpm`)
  from the carriage drive; the comments now refer to the oscillator's T2 values.
- New: `max_tow_velocity_mps`, and the acceleration limit for the drive.
- The bridge block's comment (`oscillator_state`) → `carriage_state`.

Range check: 20 m of travel in 0.0001 mm units is 2·10⁸, inside I32 (2.1·10⁹). The velocity
tag in 0.001 mm/min reaches I32 at about 36 m/s, far above any tow.

## C6: `carriage_bringup` package

Today the folder has only `config/carriage.yaml` and an empty `launch/`; colcon does not see it.

- `package.xml`, `CMakeLists.txt` installing `config` and `launch`, as `oscillator_bringup`.
- `launch/carriage.launch.py`: a copy of `oscillator.launch.py` (namespace `/carriage/<tank>`,
  `soft_stop` argument, port), or a thin wrapper if Rig-agnostic step 3 (`rig.launch.py`) is done
  first. Recommended: step 3 first, then the carriage needs no launch of its own.
- On the Pi: a `carriage@` unit and its `/etc/default/` file once the Pi is known; the
  mutual-exclusion rules in `refuse_if_busy.sh` depend on what else runs on that Pi.

## C7: on the rig

- IndraWorks: parameter backup; S-0-0260 into P-0-4081 (outputs); check the acceleration scaling.
- MGate: the slave's output block 6 → 8 words; check it does not overlap another slave's
  block; backup to `MGATE_config_backup/`.
- Tests: tows at a few speeds and accelerations, measured speed steady during `CRUISING`; Ctrl-C
  and the soft stop mid-tow; a rejection for a braking distance that does not fit; oscillation
  through the page.

## Tests and docs (Claude)

- `test_wave_trajectory.cpp` / profile tests for C2, as in the TODO.
- The fake MGate: the 16-byte output layout; tests for `move_to()` with an acceleration (write
  order, refusal above the limit, refusal without the flag, enable writes a safe acceleration).
- `test_indradrive_actuator.cpp`: `move_absolute()`, `native_move_done()`,
  `actual_velocity_mps()`.
- `test_controller_node.cpp`: a tow on a native `FakeActuator`: phases in order, the braking
  rejection, `max_tow_velocity_mps`.
- README.md: the tow command; COMMISSIONING.md: a *Carriage* section when the hardware comes.

## Open questions

- **Which drive, gateway and Pi?** `carriage.yaml` assumes an IndraDrive on an MGate; the address
  (.71) and the slave are placeholders. If the carriage shares a gateway with a wavemaker, the
  blocks must not overlap and Test 2's cycle times must be checked with three slaves.
- **Travel, top speed and acceleration** of the carriage, and the safe braking distance at the
  ends (the drive's own limit switches?).
- **Deceleration:** the same as the acceleration, or a separate goal field? Today one value is
  used for both.
- **What does a tow log?** Position and velocity at 50 Hz in a bag, or a file per run like the
  wavemaker's?
