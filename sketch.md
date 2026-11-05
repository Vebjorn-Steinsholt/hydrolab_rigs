# Sketch: the remaining software items (TODO items 29, 30, 31, 32, 36)

The previous sketch (controller and driver fixes, items 20, 21, 22, 24, 25) is done and in the
git history (`git show ca87b35:sketch.md`).

Line numbers are as of commit `05e2994`. Each change names the function or block it goes in, so
it can be found again once earlier edits shift the lines.

## Problem

The open items in TODO.md that need code and no rig to start:

- **36.** The controller logs `No Soft stop message received for over 200 ms; treating as
  stopped` right after every activation that blocks (homing, enabling the drive), although the
  soft stop publishes at 50 Hz. The TODO entry suspected the callback group.
- **29.** `IndraDrive::disable()` waits up to `step_timeout_ms` (10 s) for standstill and again
  for torque off: up to 20 s inside `on_deactivate`, `on_cleanup`, `on_error` or `on_shutdown`.
- **30.** `travel_minimum_m` / `travel_maximum_m` are declared only when `drive_motion` is
  `linear` at construction.
- **31.** The bridge's `height`, `period` and `stop` subscriptions use depth 10, not
  `command_qos`; the `stop` topic and the `stop` service share a name.
- **32.** The unused `driver_address_` member, two commented-out members, and the placeholder
  license in the `package.xml` files.

Not in this sketch:
- **43** is a probe measurement on the rig.
- **19** waits for the drive scaling to be confirmed on a linear rig.
- The planned features have their own steps in TODO.md.

## Item 36: the cause, found 2026-11-05

The soft stop subscription is not in its own callback group. `on_configure` builds
`rclcpp::SubscriptionOptions options` with `options.callback_group = soft_stop_callback_group_`
(`controller_node.cpp:607-608`), but the `create_subscription(...)` call (610-624) ends at the
lambda's `});` and never passes `options`. So the subscription sits in the node's default
callback group. The lifecycle `change_state` service is in the same group, and that group is
mutually exclusive, so the subscription is not served while any transition callback runs. The
fault timer that runs the liveness check (`fault_timer_`, also in the default group) runs first
after the transition and finds the last message older than 200 ms.

How it was found (the experiments are not committed):

1. A minimal Jazzy lifecycle node with the subscription in its own group got every message
   during a 2 s blocking `on_activate`, with 2, 4 or 20 executor threads.
2. The real `ControllerNode` in the gtest, with transitions sent through `change_state` as the
   lifecycle manager does and an actuator whose `start()` sleeps 2 s, logged the timeout every
   time. A log line in the subscription callback showed no messages during the activation, and
   the queued ones arriving right after.
3. gdb during the activation showed one executor thread in `on_activate`, two waiting for the
   executor's lock, and one idle in `rcl_wait` while messages arrived. So the subscription was not
   in any group that could be taken from.
4. Removing the action server, the action client, both services or the fault timer changed
   nothing. Passing `options` to `create_subscription` stopped the log in 3 runs out of 3.

This explains every observation in the TODO entry made during or right after a blocking
activation: 2026-10-16, 10-22, 10-23, including `Soft stop connected` appearing only when the 21 s
homing ended. Two observations are not explained by it:
- 2026-10-26: the log came minutes after a failed activation.
- 2026-11-05 09:32: the log came mid-session, followed 5 s later by `Soft stop triggered`.

The default group can be busy at other times too: parameter services, the bond, `get_state`.
Check both cases on the rig after the fix (section 5).

## Decisions (2026-11-05)

1. **36: pass `options`; keep the 200 ms timeout.** Do not change the timeout or skip the check:
   the cause is the group, not the Pi's timing. The callback only touches atomics, so it is safe
   in its own mutually exclusive group, concurrent with the fault timer.
2. **29: a separate `disable_timeout_ms` for `disable()`, default 3000.** It is used for each of
   the two waits, so the worst case is 6 s instead of 20 s. That is less than `run.sh`'s
   `sigterm_timeout:=20`, so a `systemctl stop` no longer races the escalation to SIGTERM.
   `enable()` keeps `step_timeout_ms` (power-up and homing can take long). The 3 s default is
   a guess: from 240 rpm the drive's own deceleration should stop it in well under a second.
   Measure it on the rig (section 5). If the drive does not reach standstill in time, torque
   off goes out anyway, as today.
3. **30: always declare both.** For an angular drive they are declared and ignored (the limits
   come from `min_position_deg` / `max_position_deg`). `play_dat`'s
   cross-check asks for declared parameters only and `rig_limits` ignores `travel_*` for angular
   drives, so it is not affected.
4. **31: `command_qos` everywhere, and rename the service to `stop_wave`.** The QoS change is
   for consistency only: `10` and `QoS(10).reliable()` are the same profile (keep last 10,
   reliable, volatile). The service is used only from the terminal and `test_bridge_node`; the
   web page publishes the `stop` topic. The rename breaks a terminal command, so it goes in the
   README. There is no rename table for services.
5. **32: remove the dead code; the license is your decision.** The `driver_address` parameter
   stays declared in the constructor (line 92): `IndraDriveActuator` reads it
   (`indradrive_actuator.cpp:62`). Every `package.xml` (seven) has a `TODO` license. Pick one
   for all of them, e.g. `Apache-2.0` (the ROS default) or `Proprietary` if the code stays at
   NTNU.

## Files

| File | Where | Change | Section |
|---|---|---|---|
| `rig_controller/src/controller_node.cpp` | `create_subscription` for `soft_stop` in `on_configure()` (610-624) | Pass `options` | 1 |
| `rig_controller/test/test_controller_node.cpp` | After `CancelAllGoalsCancelsARunningGoal`; includes | `SlowStartActuator`, log capture, `SoftStopServedDuringBlockingActivate` | 1 |
| `drivers/mgate5101_driver/include/mgate/indradrive.h` | `DriveConfig` (line 70) | `disable_timeout_ms = 3000` | 2 |
| `drivers/mgate5101_driver/src/indradrive.cpp` | `IndraDrive::disable()` (264-275) | Use `disable_timeout_ms` | 2 |
| `rig_controller/src/indradrive_actuator.cpp` | `declare_parameter_if_missing` (55); config (122-129); `stop()` (215-224) | Parameter, check, log of the duration | 2 |
| `rig_controller/test/test_indradrive_actuator.cpp` | `FakeMGate` status word (213-217); after `HaltAfterStopDoesNothing` | `stuck` flag; `StopGivesUpAfterDisableTimeout` | 2 |
| `wavemaker_bringup/config/wavemakers.yaml` | After `step_timeout_ms` (29) | `disable_timeout_ms: 3000` | 2 |
| `rig_controller/src/controller_node.cpp` | Constructor (106-109) | Declare `travel_*` always | 3 |
| `rig_controller/test/test_controller_node.cpp` | After `AngularDriveLimitsOverrideWavemakerMeterBounds` | `TravelLimitsDeclaredForAngularDrive` | 3 |
| `rig_controller/src/bridge_node.cpp` | Subscriptions (92-100); `stop_service_` (107-108) | `command_qos`; `"stop_wave"` | 4 |
| `rig_controller/test/test_bridge_node.cpp` | `stop_service` client (151) | `"stop_wave"` | 4 |
| `rig_controller/src/controller_node.cpp` | `on_configure()` (188); members (1397, 1458-1459) | Remove `driver_address_` and the commented-out members | 4 |
| `*/package.xml` (7 files) | Line 8 | `<license>` | 4 |
| `README.md` | Driver parameters table (146); bridge table (743) and text (778); travel row (111) | `disable_timeout_ms`, `stop_wave`, travel note | 6 |
| `TODO.md` | Items 29, 30, 31, 32, 36 | Remove | 6 |
| `rpi/README.md` | *Rig test* (164) | Checks for 29 and 36 | 5 |

## 1. Soft stop subscription in its own group (item 36)

**`controller_node.cpp`**, in `on_configure()`, the `soft_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(`
call: change its last line from `});` to:

```cpp
      }, options);
```

`options` is the `rclcpp::SubscriptionOptions` set two statements above (lines 607-608). Nothing
else changes; `release_resources()` already resets the group and the subscription.

**Test**, in **`test_controller_node.cpp`**, after `CancelAllGoalsCancelsARunningGoal`. It is
the experiment, kept: it fails before the fix (every run) and passes after it.

- `SlowStartActuator` in the anonymous namespace: an `Actuator` whose `start()` sleeps 1 s, with
  trivial overrides for the rest (as in `FakeActuator`: positions 0.25, speeds unlimited).
- Log capture: install a handler with `rcutils_logging_set_output_handler()` that sets an
  `std::atomic<bool>` when a message contains `No Soft stop message received`, then calls the
  previous handler (from `rcutils_logging_get_output_handler()`); restore it at the end of the
  test. Include `<rcutils/logging.h>`.
- `SoftStopServedDuringBlockingActivate`:
  1. Create the node with the parameters of `node_options(ns)` but without
     `with_soft_stop_released()`. That helper sets `soft_stop_timeout_s` to 1e6, which hides the
     bug; this test needs the default 0.2.
  2. Spin it in a `MultiThreadedExecutor` on its own thread.
  3. A second node publishes `false` on `ns + "/soft_stop"` every 50 ms from a thread.
  4. Configure and activate through a `lifecycle_msgs::srv::ChangeState` client on
     `controller/change_state`, so the callbacks run in the executor as the lifecycle manager's
     do. `trigger_transition()` runs them in the test thread and does not show the bug.
  5. Expect, 300 ms after the activate reply, that the timeout message was not logged.
  6. Stop the publisher, deactivate, cancel and join the executor.

## 2. A shorter timeout for `disable()` (item 29)

**`indradrive.h`**, `DriveConfig`, after `step_timeout_ms` (line 70):

```cpp
    int disable_timeout_ms = 3000;  // each of the two waits in disable(): standstill, torque off
```

**`indradrive.cpp`**, `IndraDrive::disable()`: in both `wait_for(...)` calls (lines 269 and
273), `cfg_.step_timeout_ms` becomes `cfg_.disable_timeout_ms`. The comment above the function
gets: "Bounded by 2 x disable_timeout_ms; torque off goes out even if standstill times out."

**`indradrive_actuator.cpp`**:

- After `declare_parameter_if_missing(node, "step_timeout_ms", 10000);` (line 55):
  `declare_parameter_if_missing(node, "disable_timeout_ms", 3000);`
- After `drive_config.step_timeout_ms = ...` (line 122):
  `drive_config.disable_timeout_ms = node.get_parameter("disable_timeout_ms").as_int();`, and
  add `drive_config.disable_timeout_ms <= 0` to the check below it.
- `stop()` (215-224): time `indradrive_->disable()` with `std::chrono::steady_clock`. Log the
  duration at info level ("Drive disabled in %.2f s"), and at warn level with `last_error()`
  when `disable()` returns false. This is for the rig measurement in section 5. `stop()` has
  no logger today: keep the `rclcpp::Logger` passed to `start()` in a member.

**`wavemakers.yaml`**, ladertanken block, after `step_timeout_ms: 10000` (line 29):
`disable_timeout_ms: 3000`.

**Test**, in **`test_indradrive_actuator.cpp`**:

- `FakeMGate`: add `std::atomic<bool> stuck{false};`. When it is set, the status word (built at
  lines 213-217) keeps ready `AF` and clears standstill (bit 3), so neither wait in
  `disable()` succeeds.
- `StopGivesUpAfterDisableTimeout`, after `HaltAfterStopDoesNothing`:
  1. Set `disable_timeout_ms` to 200.
  2. Call `start()`, then set `fake.stuck`, then time `stop()`.
  3. Expect it to return in under 1 s (two 200 ms waits, plus the poll interval and slack;
     with `step_timeout_ms` it would take 20 s).
  4. Expect the control word at the fake to be Operating mode only (torque off sent anyway).

## 3. Travel limits always declared (item 30)

**`controller_node.cpp`**, constructor, lines 106-109: drop the `if (... == "linear") { ... }`
and keep the two declarations, with a comment:

```cpp
    // Linear drives only; an angular drive takes its limits from min/max_position_deg.
    // Declared always, so drive_motion can change before configure.
    declare_parameter<double>("travel_minimum_m", 0.0);
    declare_parameter<double>("travel_maximum_m", 1.0);
```

`on_configure()` reads them only in the linear branch (lines 226-227), so nothing else changes.

**Test**, in **`test_controller_node.cpp`**, after
`AngularDriveLimitsOverrideWavemakerMeterBounds`: `TravelLimitsDeclaredForAngularDrive`.
1. Build a node with `angular_node_options()`.
2. Expect `has_parameter("travel_minimum_m")` and `has_parameter("travel_maximum_m")`.
3. Set `drive_motion` to `linear`, `travel_minimum_m` 0.2 and `travel_maximum_m` 0.3 with
   `set_parameter`.
4. Configure. Expect `PRIMARY_STATE_INACTIVE`; today `on_configure` throws
   `ParameterNotDeclaredException`.

## 4. Bridge interface and clean-up (items 31, 32)

**`bridge_node.cpp`**:

- Lines 92-100: in the `height`, `period` and `stop` subscriptions, the `10` becomes
  `command_qos` (declared at line 81). This doesn't change behaviour.
- Line 108: the service name `"stop"` becomes `"stop_wave"`. The comment above the service:
  "Topic `stop` from the web page; service `stop_wave` from the terminal."

**`test_bridge_node.cpp`**, line 151: `create_client<std_srvs::srv::Trigger>("stop_wave")`.

**`controller_node.cpp`**:

- Remove `driver_address_ = get_parameter("driver_address").as_string();` (line 188) and the
  member `std::string driver_address_;` (line 1397). Keep `declare_parameter<std::string>("driver_address", "")`
  (line 92).
- Remove the two commented-out members at the end of `ControllerNode` (lines 1458-1459,
  `// std::unique_ptr<WavemakerDriver> driver_;` and `// rclcpp::TimerBase::SharedPtr command_timer_;`)
  and the blank line before them.

**`package.xml`**, line 8 in all seven: `rig_controller`, `rig_interfaces`, `rig_web`,
`wavemaker_bringup`, `oscillator_bringup`, `drivers/mgate5101_driver`,
`drivers/linmot_serial_driver`. Use the license you choose (decision 5). If it is
`Apache-2.0`, also add a `LICENSE` file at the repository root.

## 5. Rig check (ladertanken, after merging)

Added to `rpi/README.md`, *Rig test*:

1. **36.** Start the service. The journal has `Soft stop connected` during the activation, and
   no `No Soft stop message received` after `Managed nodes are active`. Stop and start twice
   more.
2. **36, idle.** Leave the page open without motion for 30 min. Note any
   `No Soft stop message received` with its time, and whether `soft_stop` logged anything around
   it (the 2026-11-05 09:32 case).
3. **29.** Start the fastest lab wave (1/15 at 1.3 s, 17.1 cm, about 170 rpm), then
   `systemctl stop` mid-wave. The journal's `Drive disabled in N s` gives the standstill time
   from full speed. If it is above 2 s, raise `disable_timeout_ms` to 2 × the measured value.
4. **31.** `ros2 service call $NS/stop_wave std_srvs/srv/Trigger` stops a running wave; Stop on
   the page still works.

## 6. Documentation

- **`README.md`:**
  - Driver parameters, line 146: add a row: `disable_timeout_ms` (3000), "timeout for each of
    the two waits when the drive is switched off (standstill, torque off); bounds every
    transition that leaves active".
  - Line 111, travel row: add "Declared for every drive; ignored for an angular drive."
  - Bridge table, line 743, and the text at line 778: `stop` (service) becomes `stop_wave`.
- **`TODO.md`:** remove items 29, 30, 31, 32 and 36.

## Out of scope

- 43 (probe measurement) and 19 (linear drives).
- The LinMot's `disable()`: it has its own `step_timeout_ms` of 2000 and was not part of item 29.
- Moving the fault timer out of the default group. After the fix it delays only the liveness
  check and the fault handling until a transition ends. Revisit if the rig check shows a fault
  handled late.
