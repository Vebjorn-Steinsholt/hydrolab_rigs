# Sketch: controller and driver fixes (TODO items 20, 21, 22, 24, 25)

The previous sketch (`generate_waves` records how a file was made, items 39, 41, 42) is done and in
the git history (`git show ac8d0c2:sketch.md`).

Line numbers are as of commit `6b994c7`. Each change names the function or block it goes in, so
it can be found again once earlier edits shift the lines.

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

| File | Where | Change | Section |
|---|---|---|---|
| `drivers/mgate5101_driver/include/mgate/indradrive.h` | `last_error()` (111); members (128-130) | Declaration returns by value; add `error_mtx_` | 1 |
| `drivers/mgate5101_driver/src/indradrive.cpp` | `IndraDrive::fail()` (139); new `last_error()` before it | Lock around write and read | 1 |
| `rig_controller/test/test_indradrive_actuator.cpp` | Anonymous namespace (12-62); after `HaltBeforeStartDoesNotThrow` (98-110) | `FakeMGate`, `actuator_options(port)`, three halt tests | 2 |
| `rig_controller/src/controller_node.cpp` | `cancel_service_` lambda (415); `enum class ReturnStatus` (705); `move_to_upright()` (1177-1209); `return_to_upright` worker lambda (503-563); `trajectory_.configure(...)` (863) | 22, 24, 25 | 3, 4, 5 |
| `rig_interfaces/srv/ReturnToUpright.srv` | After `POSITION_INVALID = 6` | `SOFT_STOP = 7` | 4 |
| `rig_web/scripts/web_server.py` | `RETURN_STATUS` (91-94) | `7: 'SOFT_STOP'` | 4 |
| `rig_controller/src/wave_trajectory.hpp` | Class comment and `configure()` (22-29); includes | `blend_max_velocity`, `startup_duration()` | 5 |
| `rig_controller/src/wave_trajectory.cpp` | `configure()` (41-65); `peak_velocity()` (77-92); `peak_acceleration()` (108-124); `sample()` comment (128) | Blend over n periods | 5 |
| `rig_controller/test/test_controller_node.cpp` | `FakeActuator` (34-148); `speed_limited_node()` / `goal_accepted_with_speed_limit()` (286-346); new tests after 602, 1009, 1145 | Tests for 22, 24, 25 | 3, 4, 5 |
| `rig_controller/test/test_wave_trajectory.cpp` | After `PeakVelocityBoundsEverySample` (137-156) | Tests for 25 | 5 |
| `README.md` | 644-645, 731-732, 782 | Reply statuses, start blend, tests | 6 |
| `TODO.md` | 27-42 | Remove items 20, 21, 22, 24, 25 | 6 |

## 1. `IndraDrive::last_error_` (item 20)

**`indradrive.h`**, line 111, in the public section after `request_abort()`: replace the inline
accessor with a declaration:

```cpp
    // Copy: fail() writes it from the thread running a drive command.
    std::string last_error() const;
```

Lines 128-130, private members: add the mutex right above `last_error_`, so the pair reads like
`fault_mtx_` / `fault_reason_` just above it:

```cpp
    mutable std::mutex fault_mtx_;
    std::string        fault_reason_;
    mutable std::mutex error_mtx_;
    std::string        last_error_;
```

`<mutex>` and `<string>` are already included (lines 28-29).

**`indradrive.cpp`**, line 139: add `last_error()` directly above `IndraDrive::fail()` and put
the write in `fail()` under the lock. The `fprintf` stays outside it:

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

`fail()` is the only writer of `last_error_` (`grep last_error_ src/indradrive.cpp`).

**Callers, no change:** `rig_controller/src/indradrive_actuator.cpp:193` (`.c_str()` within the
log call), `:277` (`fault_reason()` returns the string), `examples/example_main.cpp:131`. All use
the value within the expression, so a temporary is fine. The LinMot driver's `last_error()` has
the same shape but is only used from one thread; leave it.

## 2. Testing `halt()` (item 21)

All of this goes in **`rig_controller/test/test_indradrive_actuator.cpp`**. No CMake change:
`test_indradrive_actuator` already links `mgate5101_driver::mgate5101` (`CMakeLists.txt:95-104`),
and the fake needs only POSIX sockets and `<thread>`.

**Includes** (lines 1-4): add `<arpa/inet.h>`, `<netinet/in.h>`, `<sys/socket.h>`, `<unistd.h>`,
`<atomic>`, `<chrono>`, `<cstdint>`, `<mutex>`, `<thread>`, `<array>`.

**`actuator_options()`** (line 15): take the port, `actuator_options(int port = 1502)`, and pass
it as `driver_port`. The existing tests keep calling it without an argument.

**`FakeMGate`**: a class in the anonymous namespace, after `actuator_options()` and before
`IndraDriveActuatorTest` (between lines 46 and 48), ~120 lines. Its constructor binds a socket
to 127.0.0.1 port 0, reads the port back with `getsockname()` (`port()`), and starts one thread
that accepts a single connection and serves requests until the destructor closes the socket and
joins. It serves:

- FC4 at `input_base_reg` 0 (7 words): status word, position (int32, 0.0001°), velocity, diag
  number. Position follows `TargetPosition` at once.
- FC3 / FC16 at `output_base_reg` 0 (6 words): control word, target position, speed cap,
  override. It keeps what the driver writes; `write_only_dirty` is false, so every cycle writes
  the whole area.
- FC4 at `status_base_reg` 768 (9 words): gateway status "Operate" (bits 1:0 = 11) and the
  live-list bit for `profibus_address` 2 (as `IndraDrive::slave_live()` reads it).

The status word follows the control word as the drive does in `enable()`: operating mode bit →
mode `10` and ready `Ab`; plus Drive On → ready `AF`; plus Drive Start → "following" (bit 7
clear), in standstill and target reached. Public members for the test, under a mutex:
`set_position_deg(double)`, `control_word()`, `target_deg()`, `speed_cap_rpm()`, and
`wait_for(predicate, timeout)` that polls them.

**Tests**, after `HaltBeforeStartDoesNotThrow` (ends line 110), before
`CanBeRecreatedAfterParametersAreDeclared`. Each builds the node like `HaltBeforeStartDoesNotThrow`
(the same three `declare_parameter` calls) but with `actuator_options(fake.port())`:

| Test | Steps | Expect at the fake after one cycle |
|---|---|---|
| `HaltHoldsLastTargetWithHoldSpeed` | `start()`, `write_setpoint(100°, 300 deg/s)`, `halt()` | target 100.0°, speed cap `hold_velocity_rpm` (10), control word Operating mode + Drive On + Drive Start |
| `HaltWithoutTargetHoldsPolledPositionClamped` | fake position 600° (limit 500°), `start()`, `halt()` | target 500.0°, speed cap 10 rpm, Drive Start set |
| `HaltAfterStopDoesNothing` | `start()`, `write_setpoint(30°)`, `stop()`, `halt()` | control word Operating mode only (`disable()` turns torque off), target still 30°, no exception |

`start()` re-parks the target on the polled position before enabling, so the second test sets
the fake's position before `start()`. Each test waits for the outputs with `fake.wait_for(...)`
and a deadline of a few poll cycles (`poll_interval_ms` 50 in `actuator_options()`). The code
under test is `IndraDriveActuator::halt()`, `indradrive_actuator.cpp:202-213`; it is not changed.

## 3. `cancel_all_goals` right after configure (item 22)

**`controller_node.cpp`**, in `on_configure()`, inside the `cancel_service_` lambda: replace the
check at line 415 (`if (!cancel_client_ || !cancel_client_->action_server_is_ready())`) with:

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

The `respond(...)` body and `async_cancel_all_goals(...)` below it (line 422 on) stay as they
are. `wait_for_action_server()` returns at once when the server is already known.

**Tests**, in **`test_controller_node.cpp`**, after `CancelledActionHaltsAndAcceptsNextGoal`
(ends line 602), before `SinusoidalGoalBlendsFromMeasuredPosition`. There are none for this
service yet. Each creates a `CancelAllGoals` client on a client node in the test's namespace
(`node_options(ns)`) and spins with `spin_until()`:

- `CancelAllGoalsRightAfterActivate`: configure, activate, call the service as soon as
  `wait_for_service()` returns. Expect `NO_ACTIVE_GOALS`, not `SERVER_UNAVAILABLE`. (Discovery
  is usually fast, so this can pass before the fix too; it guards against a regression.)
- `CancelAllGoalsCancelsARunningGoal`: start a regular goal as `CancelledActionHaltsAndAcceptsNextGoal`
  does, call the service. Expect `REQUEST_ACCEPTED`, `goals_canceling` 1, and the goal result
  `CANCELED`.

## 4. `return_to_upright` replies (item 24)

**`rig_interfaces/srv/ReturnToUpright.srv`**: add `uint8 SOFT_STOP = 7` after
`uint8 POSITION_INVALID = 6`, above the blank line before `uint8 status`.

**`controller_node.cpp`**, four places:

1. Line 705, top of the private section:
   `enum class ReturnStatus { Success, Timeout, Fault, PositionInvalid, Stopped, SoftStop };`

2. `move_to_upright()`, in the loop, the soft-stop branch (line 1179): `return ReturnStatus::Stopped;`
   becomes `return ReturnStatus::SoftStop;`.

3. `move_to_upright()`, after the loop (lines 1208-1209): tell a fault from a lifecycle stop. A
   driver fault calls `handle_driver_fault()`, which sets `stop_execution_` before the loop
   sees `faulted()`:

   ```cpp
    actuator_->halt();
    // A driver fault deactivates the node, which also ends this loop.
    return actuator_->faulted() ? ReturnStatus::Fault : ReturnStatus::Stopped;
   ```

   `stop_execution()` joins the worker before `release_resources()` resets `actuator_`, so
   `actuator_` is valid here and in the reply.

4. In `on_configure()`, the `return_to_upright` service, in the worker lambda passed to
   `start_execution()` (line 503):

   - Above `ReturnStatus status = ReturnStatus::Fault;` (line 505) add `std::string failure;`.
   - First line of `catch (const std::exception & error)` (line 508): `failure = error.what();`,
     before the existing `RCLCPP_ERROR`.
   - First line of `catch (...)` (line 515): `failure = "unknown exception";`.
   - In the `switch (status)`, `case ReturnStatus::Fault:` (line 546): replace the message
     `actuator_->fault_reason()` with
     `failure.empty() ? actuator_->fault_reason() : "Return to upright failed: " + failure`.
   - After `case ReturnStatus::Stopped:` (ends line 555), add:

     ```cpp
              case ReturnStatus::SoftStop:
                respond(
                    ReturnToUpright::Response::SOFT_STOP, final_position,
                    "Soft stop engaged; actuator halted and remains enabled");
                break;
     ```

     The message is the one a goal gets (`execute_goal`, line 1250).

**`rig_web/scripts/web_server.py`**, `RETURN_STATUS` (lines 91-94): add `7: 'SOFT_STOP'` after
`6: 'POSITION_INVALID'`, so `call_return_to_upright` (line 380) shows the name, not `7`.

**Tests** (**`test_controller_node.cpp`**, with `ReturnToUprightRig`), after
`ReturnToUprightReportsFaultDuringMove` (ends line 1009), each starting at 0.30 like it does and
triggering from a thread after 200 ms:

- **`FakeActuator`** first: add `std::atomic<bool> throw_on_position{false};` after
  `reject_writes` (line 120), and make `actual_position_m()` (line 73) throw
  `std::runtime_error("fake position read failed")` when it is set (`<stdexcept>` include).
- `ReturnToUprightReportsExceptionText`: the thread sets `throw_on_position`. Expect
  `ACTUATOR_FAULT` and a message containing `fake position read failed`.
- `ReturnToUprightReportsFaultThatDeactivates`: the thread sets `fault` and calls
  `fake->fault_callback_("fake fault")` (as `DriverFaultReleasesResourcesAndAllowsReconfigure`,
  line 1047, does). Expect `ACTUATOR_FAULT`, `fake fault`. Today it is `CONTROLLER_INACTIVE`.
- `ReturnToUprightReportsSoftStop`: the thread publishes `true` on
  `soft_stop_releaser().publishers.at(ns)`. Expect `SOFT_STOP`, `halt_count` 1, the node still
  `PRIMARY_STATE_ACTIVE`.

`ReturnToUprightReportsFaultDuringMove` (fault seen by the loop) stays as it is.

## 5. Start blend over whole periods (item 25)

**`wave_trajectory.hpp`**: add `#include <limits>` to the includes (lines 3-4). Replace the class
comment at line 22 and the `configure()` declaration (lines 26-28), and add `startup_duration()`
after `required_maximum()` (line 31):

```cpp
// Regular wave with a quintic blend from the start position over whole periods: one, or more
// when the start is far from the wave, so the blend itself stays under blend_max_velocity.
class WaveTrajectory
{
public:
  // wave_height is the target wave height H (crest to trough).
  void configure(
    const WavemakerGeometry & geometry, double wave_height, double period,
    double start_position,
    double blend_max_velocity = std::numeric_limits<double>::infinity());

  double required_minimum() const;
  double required_maximum() const;
  double startup_duration() const {return startup_duration_;}
```

The default keeps today's single period, so existing callers and tests are unchanged.

**`wave_trajectory.cpp`**:

- `configure()` signature (lines 41-43): add `double blend_max_velocity`.
- `configure()`, last line (64): replace `startup_duration_ = period;` with:

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

  `required_minimum()` / `required_maximum()` depend only on `amplitude_`, `upright_position_`
  and `upright_is_minimum_`, all set above it. `kQuinticPeakVelocityFactor` (1.875) comes from
  `wave_math.hpp`, already included (line 9).
- `peak_velocity()` (line 83) and `peak_acceleration()` (line 113): the 10 000 samples assume one
  period. Rename `kBlendSamples` to `kBlendSamplesPerPeriod` and loop over
  `const int samples = kBlendSamplesPerPeriod * blend_periods();` (in both the loop bound and
  the `startup_duration_ * i / samples` divisor), with a private helper in the header next to
  `wave()`:
  `int blend_periods() const {return static_cast<int>(std::lround(startup_duration_ * omega_ / (2.0 * M_PI)));}`
  The comment above the constant in `peak_velocity()` (lines 79-82) stays right per period.
- `sample()`, comment at line 128: "only because startup_duration_ equals the period" becomes
  "only because startup_duration_ is a whole number of periods". The code is right for any whole
  number (`wave()` is periodic).

**`controller_node.cpp`**, in the regular-wave branch of goal preparation, the
`trajectory_.configure(...)` call (lines 863-866): add `return_max_velocity_mps_` after
`start_position`, for a wavemaker and the oscillator alike:

```cpp
      goal.height, goal.period, start_position, return_max_velocity_mps_);
```

Example, ladertanken: paddle at +0.14 m, 0.7 s, 0.026 m (amplitude 15 mm): D = 0.155 m,
1.875 × 0.155 / (0.7 × 0.1) = 4.2 → 5 periods (3.5 s) instead of 0.7 s. Near upright (D = 15 mm)
n = 1.

**Tests:**

- **`test_wave_trajectory.cpp`**, after `PeakVelocityBoundsEverySample` (ends line 156):
  - `BlendStaysOnePeriodNearUpright`: start at upright with a finite blend speed:
    `startup_duration()` == T.
  - `BlendUsesWholePeriodsFromFarStart`: the example above: 5 T, and over the blend
    |velocity| ≤ amplitude × ω + return speed (sampled).
  - `PhaseContinuousAfterLongBlend`: position and velocity just before and after 5 T agree to
    1e-9 m.
  - `PeakVelocityBoundsEverySampleWithLongBlend`: the loop of `PeakVelocityBoundsEverySample`
    with a blend speed that gives five periods, sampled up to `startup_duration() + 2 T`.
- **`test_controller_node.cpp`**:
  - `speed_limited_node()` (line 286) and `goal_accepted_with_speed_limit()` (line 339): add a
    last parameter `double start_position = 0.25` and set `actuator->actual_position` from it in
    the factory. Existing calls are unchanged.
  - `RegularWaveFarFromUprightIsAccepted`, after `RegularWaveAboveDriveSpeedIsRejected` (ends
    line 1145): start 0.0 (travel end), piston 1 m water, T = 1 s, H = 0.01 m, drive maximum
    0.08 m/s, return speed 0.05 (set in `speed_limited_node()`). Today's one-period blend needs
    about 0.47 m/s; ten periods stay under 0.08. Expect accepted; confirm the numbers with
    `peak_velocity()` while writing the test.
  - `RegularWaveAboveDriveSpeedIsRejected` still passes (the wave itself is too fast).

## 6. Documentation

- **`README.md`:**
  - *Service `return_to_upright`*, lines 644-645: add `SOFT_STOP` to the reply statuses, then one
    line on the three changed cases: an exception gives `ACTUATOR_FAULT` with its text; a fault
    gives `ACTUATOR_FAULT` even when it deactivates the node; a soft stop gives `SOFT_STOP`.
  - *How a motion runs*, *Start blend*, lines 731-732: "over one period" becomes "over one
    period, or more whole periods when the paddle starts far from the wave, so the blend stays
    under `return_to_upright_max_velocity_mps`".
  - *Tests*, line 782: `test_indradrive_actuator`: "unit conversion, parameter checks, and
    `halt()` against a fake MGate".
- **`TODO.md`:** remove items 20 (line 27), 21 (30), 22 (33), 24 (38) and 25 (41) with their
  description lines.

## Out of scope

- The LinMot driver's `last_error()` (one thread only; see section 1).
- A blend for pregenerated goals: it already blends at the return speed (`prepare_goal`).
- Item 36 (soft stop subscription not served while `on_activate` blocks): separate.
