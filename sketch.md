# Sketch: controller and driver fixes (TODO items 20, 21, 22, 24, 25)

The previous sketch (`generate_waves` records how a file was made, items 39, 41, 42) is done and in
the git history (`git show ac8d0c2:sketch.md`).

## Problem

Five small, independent items from TODO.md, *Medium* and *Low*:

- **20.** `IndraDrive::last_error_` is written by `fail()` in whatever thread runs a drive command,
  and read through `last_error()` by executor threads (`IndraDriveActuator::fault_reason()`). A
  `std::string` written and read at once is undefined behaviour.
- **21.** `IndraDriveActuator::halt()` holds the paddle by writing the last target with
  `hold_velocity_rpm` and leaving Drive Start set (Drive Halt would make the drive ignore later
  targets). Nothing tests that; `HaltBeforeStartDoesNotThrow` only covers a disabled drive.
- **22.** `cancel_all_goals` replies `SERVER_UNAVAILABLE` when the controller's own action server
  is not yet discovered, which can happen right after configure.
- **24.** `return_to_upright` replies with the wrong status or message in three cases: an
  exception (`ACTUATOR_FAULT` with an empty message), a fault that deactivates the node
  (`CONTROLLER_INACTIVE`, "Interrupted by lifecycle transition"), and a soft stop (the same).
- **25.** A regular wave always blends in over one period. Far from the wave's first positions
  with a short period, that blend is fast and the goal is rejected for speed.

## Decisions (2026-11-05)

1. **20: a mutex, `last_error()` returns a copy.** The existing `fault_mtx_` guards
   `fault_reason_`; `last_error_` gets its own (`error_mtx_`), so a slow reader never holds up the
   fault path. The return type changes from `const std::string&` to `std::string`; the callers
   use the value at once, so they compile unchanged.
2. **21: a fake MGate in the test, not a seam in the actuator.** `halt()` only acts on an enabled
   drive, and `enable()` waits for the drive's status word. A small Modbus/TCP server inside the
   gtest answers like the gateway and the drive (status word from the control word, live list),
   and the test reads what the driver writes. The actuator stays as it is; `tests/stub_server.py`
   in the driver only echoes registers, so it cannot pass `enable()`.
3. **22: wait up to 1 s in the service callback.** `wait_for_action_server(1s)` before replying
   `SERVER_UNAVAILABLE`. The service has its own mutually exclusive callback group and `main()`
   uses a `MultiThreadedExecutor`, so the wait blocks only further `cancel_all_goals` calls.
   Discovery normally completes within a few ms; a timer-based retry is more code for no gain.
4. **24: a new status `SOFT_STOP = 7`** in `ReturnToUpright.srv`. A soft stop is neither a
   lifecycle transition nor a fault, and a client (the web page) should be able to tell it from
   both. The alternative, `CONTROLLER_INACTIVE` with a soft-stop message, keeps the interface
   but leaves the status wrong. The web server's status names get the new entry.
5. **25: whole periods, enough to keep the blend under `return_to_upright_max_velocity_mps`.**
   The blend's own speed is at most 1.875 D / (n T), D the largest distance from the start to
   the wave's positions. Take the smallest whole n ≥ 1 with that at or below the return speed.
   Whole periods keep the phase continuous, as now. Near upright, n stays 1, so today's waves do
   not change. The wave's own speed is checked against the drive as before.

## Files

| File | Change | Section |
|---|---|---|
| `drivers/mgate5101_driver/include/mgate/indradrive.h`, `src/indradrive.cpp` | `last_error_` under a mutex, returned by value | 1 |
| `rig_controller/test/test_indradrive_actuator.cpp` | Fake MGate and three halt tests | 2 |
| `rig_controller/src/controller_node.cpp` | Wait for the action server (22); return replies (24); pass the return speed to the wave (25) | 3, 4, 5 |
| `rig_interfaces/srv/ReturnToUpright.srv` | `SOFT_STOP = 7` | 4 |
| `rig_web/scripts/web_server.py` | `7: 'SOFT_STOP'` in `RETURN_STATUS` | 4 |
| `rig_controller/src/wave_trajectory.hpp`, `.cpp` | Blend over n periods | 5 |
| `rig_controller/test/test_controller_node.cpp` | Tests for 22, 24, 25 | 3, 4, 5 |
| `rig_controller/test/test_wave_trajectory.cpp` | Tests for 25 | 5 |
| `README.md`, TODO.md | Reply statuses, start blend; remove items 20-25 done | 6 |

## 1. `IndraDrive::last_error_` (item 20)

`indradrive.h`:

```cpp
    // Copy: fail() writes it from the thread running a drive command.
    std::string last_error() const;
    ...
    mutable std::mutex error_mtx_;
    std::string        last_error_;
```

`indradrive.cpp`:

```cpp
std::string IndraDrive::last_error() const {
    std::lock_guard<std::mutex> lk(error_mtx_);
    return last_error_;
}

bool IndraDrive::fail(const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(error_mtx_);
        last_error_ = why;
    }
    std::fprintf(stderr, "IndraDrive: %s\n", why.c_str());
    return false;
}
```

Callers: `indradrive_actuator.cpp:193` (`.c_str()` within the log call) and `:277` (returns the
string), `examples/example_main.cpp:131`. All use the value within the expression, so a temporary
is fine. The LinMot driver's `last_error()` has the same shape but is only used from one thread;
leave it.

## 2. Testing `halt()` (item 21)

**Fake MGate** (in `test_indradrive_actuator.cpp`, ~120 lines): a thread that listens on
127.0.0.1 with an ephemeral port (passed as `driver_port`) and serves:

- FC4 at `input_base_reg` (7 words): status word, position (int32, 0.0001°), velocity, diag
  number. Position follows `TargetPosition` at once.
- FC3 / FC16 at `output_base_reg` (6 words): control word, target position, speed cap,
  override. It keeps what the driver writes; `write_only_dirty` is false, so every cycle writes
  the whole area.
- FC4 at `status_base_reg` (9 words): gateway status "Operate" (bits 1:0 = 11) and the live-list
  bit for `profibus_address` 2 (as `IndraDrive::slave_live()` reads it).

The status word follows the control word as the drive does in `enable()`: operating mode bit →
mode `10` and ready `Ab`; plus Drive On → ready `AF`; plus Drive Start → "following" (bit 7 clear),
in standstill and target reached. A test can also set a starting position.

**Tests:**

| Test | Steps | Expect at the fake after one cycle |
|---|---|---|
| `HaltHoldsLastTargetWithHoldSpeed` | `start()`, `write_setpoint(100°, 300 deg/s)`, `halt()` | target 100.0°, speed cap `hold_velocity_rpm` (10), control word Operating mode + Drive On + Drive Start |
| `HaltWithoutTargetHoldsPolledPositionClamped` | fake position 600° (limit 500°), `start()`, `halt()` | target 500.0°, speed cap 10 rpm, Drive Start set |
| `HaltAfterStopDoesNothing` | `start()`, `stop()`, `halt()` | control word 0, no exception |

`start()` re-parks the target on the polled position before enabling, so the second test sets
the fake's position before `start()`. Each test waits for the outputs with a deadline of a few
poll cycles (`poll_interval_ms` 50 in `actuator_options()`).

## 3. `cancel_all_goals` right after configure (item 22)

`controller_node.cpp`, in the service callback (now at about line 415):

```cpp
        // Our own action server; discovery can lag right after configure.
        if (!cancel_client_ ||
          !cancel_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
          respond(
            CancelAllGoals::Response::SERVER_UNAVAILABLE,
            0, "Action server is unavailable");
          return;
        }
```

`wait_for_action_server()` returns at once when the server is already known.

**Tests** (`test_controller_node.cpp`; there are none for this service yet):

- `CancelAllGoalsRightAfterActivate`: configure, activate, call the service as soon as it is
  available. Expect `NO_ACTIVE_GOALS`, not `SERVER_UNAVAILABLE`.
- `CancelAllGoalsCancelsARunningGoal`: start a regular goal, call the service. Expect
  `REQUEST_ACCEPTED`, `goals_canceling` 1, and the goal ends canceled.

## 4. `return_to_upright` replies (item 24)

`ReturnToUpright.srv`: add `uint8 SOFT_STOP = 7`.

`controller_node.cpp`:

- `enum class ReturnStatus { Success, Timeout, Fault, PositionInvalid, Stopped, SoftStop };`
- In `move_to_upright()`, the soft stop returns `ReturnStatus::SoftStop` (line ~1179), and the
  end of the loop tells a fault from a lifecycle stop. A driver fault calls
  `handle_driver_fault()`, which stops the worker before it sees `faulted()` in the loop:

  ```cpp
    actuator_->halt();
    // A driver fault deactivates the node, which also ends this loop.
    return actuator_->faulted() ? ReturnStatus::Fault : ReturnStatus::Stopped;
  ```

  `stop_execution()` joins the worker before `release_resources()` resets `actuator_`, so
  `actuator_` is valid here and in the reply.
- In the worker lambda, keep the exception text:

  ```cpp
          std::string failure;
          try {
            status = move_to_upright(tolerance, final_position);
          } catch (const std::exception & error) {
            failure = error.what();
            ...
          } catch (...) {
            failure = "unknown exception";
            ...
          }
  ```

  and reply with it:

  ```cpp
              case ReturnStatus::Fault:
                respond(
                    ReturnToUpright::Response::ACTUATOR_FAULT, final_position,
                    failure.empty() ? actuator_->fault_reason() :
                    "Return to upright failed: " + failure);
                break;
              ...
              case ReturnStatus::SoftStop:
                respond(
                    ReturnToUpright::Response::SOFT_STOP, final_position,
                    "Soft stop engaged; actuator halted and remains enabled");
                break;
  ```

  The soft-stop message is the one a goal gets (`execute_goal`, line ~1250).

`web_server.py`: `7: 'SOFT_STOP'` in `RETURN_STATUS`, so the page shows the name, not `7`.

**Tests** (`test_controller_node.cpp`, with `ReturnToUprightRig`):

- `ReturnToUprightReportsExceptionText`: `FakeActuator` gets `throw_on_position`; expect
  `ACTUATOR_FAULT` and a message containing the exception's text.
- `ReturnToUprightReportsFaultThatDeactivates`: during the move, set `fault` and call the fault
  callback (as the driver does). Expect `ACTUATOR_FAULT`, `fake fault`. Today it is
  `CONTROLLER_INACTIVE`.
- `ReturnToUprightReportsSoftStop`: during the move, publish `soft_stop` true. Expect
  `SOFT_STOP`, the paddle halted, the node still active.

`ReturnToUprightReportsFaultDuringMove` (fault seen by the loop) stays as it is.

## 5. Start blend over whole periods (item 25)

`wave_trajectory.hpp`:

```cpp
// Regular wave with a quintic blend from the start position over whole periods: one, or more
// when the start is far from the wave, so the blend itself stays under blend_max_velocity.
class WaveTrajectory
{
public:
  void configure(
    const WavemakerGeometry & geometry, double wave_height, double period,
    double start_position,
    double blend_max_velocity = std::numeric_limits<double>::infinity());
  ...
  double startup_duration() const {return startup_duration_;}
```

The default keeps today's single period, so existing callers and tests are unchanged.

`wave_trajectory.cpp`, at the end of `configure()`, in place of `startup_duration_ = period;`:

```cpp
  // The blend adds at most kQuinticPeakVelocityFactor * distance / duration to the wave's speed.
  // Whole periods keep the phase continuous (sample()).
  const double distance = std::max(
    std::abs(required_maximum() - start_position_),
    std::abs(required_minimum() - start_position_));
  const double periods = std::isfinite(blend_max_velocity) && blend_max_velocity > 0.0 ?
    std::ceil(kQuinticPeakVelocityFactor * distance / (period * blend_max_velocity)) : 1.0;
  startup_duration_ = std::max(1.0, periods) * period;
```

`required_minimum()` / `required_maximum()` are the wave's extent and depend only on amplitude,
upright and `upright_is_minimum_`, which are set by then. `kQuinticPeakVelocityFactor` (1.875)
is in `wave_math.hpp` already.

`sample()`: the comment "only because startup_duration_ equals the period" becomes "a whole
number of periods"; the code is right for any whole number (`wave()` is periodic).

`peak_velocity()` / `peak_acceleration()` sample the blend with 10 000 points; their margin
assumes one period. Scale the count: `kBlendSamplesPerPeriod * n`, n = `startup_duration_ * omega_
/ (2 pi)` rounded.

`controller_node.cpp`, `trajectory_.configure(...)` (line ~863): pass `return_max_velocity_mps_`
for a wavemaker and the oscillator alike.

Example, ladertanken: paddle at +0.14 m, 0.7 s, 0.026 m (amplitude 15 mm): D = 0.155 m,
1.875 × 0.155 / (0.7 × 0.1) = 4.2 → 5 periods (3.5 s) instead of 0.7 s. Near upright (D = 15 mm)
n = 1.

**Tests:**

- `test_wave_trajectory.cpp`:
  - `BlendStaysOnePeriodNearUpright`: start at upright: `startup_duration()` == T.
  - `BlendUsesWholePeriodsFromFarStart`: the example above: 5 T, and over the blend
    |velocity| ≤ amplitude × ω + return speed (sampled).
  - `PhaseContinuousAfterLongBlend`: position and velocity just before and after 5 T agree to
    1e-9 m.
  - `PeakVelocityBoundsEverySample` again with a five-period blend.
- `test_controller_node.cpp`: `RegularWaveFarFromUprightIsAccepted`: fake actuator at the travel
  end, short period, a drive speed that rejects today's one-period blend but takes the wave.
  `RegularWaveAboveDriveSpeedIsRejected` still passes (the wave itself is too fast).

## 6. Documentation

- **README:**
  - *Service `return_to_upright`*: add `SOFT_STOP` to the reply statuses; one line on what each
    of the three changed cases now replies.
  - *How a motion runs*, start blend: "over one period, or more whole periods when the paddle
    starts far from the wave, so the blend stays under `return_to_upright_max_velocity_mps`".
  - *Tests*: the new actuator tests run against a fake MGate.
- **TODO.md:** remove items 20, 21, 22, 24 and 25.

## Out of scope

- The LinMot driver's `last_error()` (one thread only; see section 1).
- A blend for pregenerated goals: it already blends at the return speed (`prepare_goal`).
- Item 36 (soft stop subscription not served while `on_activate` blocks): separate.
