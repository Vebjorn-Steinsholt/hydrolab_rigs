# Sketch: the open TODO items that had no sketch

What to code (or decide) for each open item in [TODO.md](TODO.md) that had no design yet, and
why. Items 29, 30, 31, 32 and 36 are in [controller_fixes_sketch.md](controller_fixes_sketch.md),
54 in [oscillator_centre_sketch.md](oscillator_centre_sketch.md), and the long-term part of 43 in
[gain_estimation_sketch.md](gain_estimation_sketch.md).

**Status (2026-11-18): not started**, except 57 (code and tests done 2026-11-18). Checked
against `master` at `9ac7b46`. Each change names the function or block it goes in (item 62), and
line numbers are hints only.

**No sketch, on purpose:**

| Item | Why |
|---|---|
| 43 | A rig session; its plan is in the TODO entry itself. |
| 46 | Drive tuning with the drive's owner, from about 2026-11-23. Options (1) and (2) are listed in the entry. |
| 50 | A rig check, postponed to 2027. The steps are in the entry. |
| 51 | Needs the experimenters' numbers. The formula and what to set are in the entry. |
| 53 | Only if the gateway's resends become a problem. Remedies are ordered in the entry. |

## Order

1. **59, 58** before the next rig session. They tell you which code ran, and keep a broken
   commit from reaching the Pi.
2. **56** together with 36 (controller_fixes_sketch section 1). Both touch the soft stop
   subscription, and 56 reuses 36's log capture.
3. **55, 45** together. Both are about reading a drive fault from the journal.
4. **61** after 29, 30, 31, 32, 36 and 56 are merged, so the move doesn't conflict with them.
5. The rest when convenient.

## 59. Record which code played each run

**Problem.** `play_dat` writes nothing to disk, and the controller doesn't log its version. On
2026-11-17 the late starts came from one version of `play_dat`, but no record of the session
says which version was on the Pi.

**Decisions.**
1. `play_dat` writes a record next to the motion file:
   `<file stem>_played_<YYYYmmdd-HHMMSS>.txt`. It is a new file each time, so a replay doesn't
   overwrite the first record.
2. The record holds the commit, the command, the namespace and limits used, and one line per
   goal: run index, kind, the wall time the goal was sent (`time.time()`, ISO with ms), and the
   result. The send times are what the timeline in `transfer_function` (88363ba) estimates, so
   they can be compared with them later.
3. **`-dirty` warns, it doesn't refuse.** A Pi with a local edit can still be needed in a
   session. The warning goes on the first line of the record and of the terminal output.
4. `run.sh` logs the commit to the journal before `exec`. Then the controller's version is in
   the journal as well.

**Changes.**
- `rig_controller/src/play_dat.py`:
  - In `main()`, after `print_table()` and not for `--dry-run`: open the record and write the
    header. Use `motion_files.write_record`'s format (`generated`, `command`, `folder`,
    `commit`), so `transfer_function` and `play_dat` records read the same way. Either reuse
    `source_commit()` from `motion_files`, or move it to `rig_limits.py`, which `play_dat`
    already imports, if importing `motion_files` (numpy, scipy) slows the start.
  - In `play()`, append one line per goal when it is sent and when its result arrives. Flush
    after each line, so a Ctrl-C keeps what was played.
- `rpi/run.sh`, before `exec`:

  ```bash
  echo "hydrolab_rigs $(git -C "$HOME/ros_ws/src/hydrolab_rigs" describe --always --dirty --exclude='*' 2>/dev/null || echo unknown)"
  ```

  It only logs; nothing fails if git is missing.

**Tests** (`test_play_dat.py`). With the fake controller:
- A play writes the record with `commit` and one line per goal.
- `--dry-run` writes nothing.
- Two plays give two records.
- A Ctrl-C after the first goal leaves a record with that goal in it.

**Docs.** README *Check and play*: where the record is and what it holds. rpi/README
*Troubleshooting*: `journalctl -u wavemaker@ladertanken | grep hydrolab_rigs` shows the version
that ran.

## 58. Run all tests automatically

**Problem.** `colcon test` runs the gtests but not the Python tests. Those run only by hand, and
the Pis deploy `master` with `git pull`.

**Decisions.**
1. **A pre-push hook first, CI second.** The hook works today on the lab PC, which has the
   workspace sourced. CI needs a ROS image and `rosdep`, so it is more work for the same check.
2. The hook runs only the Python tests, about a minute. `colcon test` stays a manual step before
   a rig session, because it needs a build first.
3. The Python tests are also registered in CMake, so `colcon test` runs everything when you do
   run it.

**Changes.**
- `rig_controller/CMakeLists.txt` and `rig_web/CMakeLists.txt`, inside `if(BUILD_TESTING)`:

  ```cmake
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(python_tests test TIMEOUT 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ```

  Plus `<test_depend>ament_cmake_pytest</test_depend>` in both `package.xml`. The tests load
  the scripts from `src/` and `scripts/` by path (as `test_play_dat.py` does), so they test the
  source tree, which is what the hook needs too.
- `.githooks/pre-push` (new, executable):

  ```bash
  #!/usr/bin/env bash
  set -e
  cd "$(git rev-parse --show-toplevel)"
  python3 -m pytest -q rig_controller/test rig_web/test
  ```

  Enable it once per clone with `git config core.hooksPath .githooks`. Put that line in README
  *Tests* (LAB_PERSONNEL.md *Tests*). `git push --no-verify` skips it when needed.
- **Later, CI:** `.github/workflows/test.yml` on `ros:jazzy-ros-base`, running
  `rosdep install --from-paths . -y`, `colcon build`, then `colcon test` and
  `colcon test-result --verbose`. Do this only if the hook isn't enough, for example once
  someone else pushes too.

## 56. Log when soft stop messages resume

**Problem.** A gap is logged when it starts but not when it ends, so the journal can't show how
long it was.

**Decision.** Log the recovery in the subscription callback, not in
`check_soft_stop_liveness()`. The callback already does
`soft_stop_last_msg_ns_.exchange(steady_now_ns())`, and its return value is the previous
message's time, so the gap is known exactly there. After 36's fix the callback runs in its own
group, so it fires on time. Log only gaps longer than `soft_stop_timeout_ns_`, which are the
ones `check_soft_stop_liveness()` reported.

**Change.** `controller_node.cpp`, the `soft_stop` subscription's lambda in `on_configure()`:

```cpp
const int64_t now = steady_now_ns();
const int64_t previous = soft_stop_last_msg_ns_.exchange(now);
if (previous == 0) {
  RCLCPP_INFO(get_logger(), "Soft stop connected");
} else if (now - previous > soft_stop_timeout_ns_) {
  RCLCPP_WARN(get_logger(), "Soft stop messages resumed after %.0f ms", (now - previous) / 1e6);
}
```

**Test** (`test_controller_node.cpp`, reusing 36's log capture). Publish at 50 ms, pause
400 ms, then resume. Expect exactly one `resumed after` line, with a value between 400 and
600 ms. With no pause, expect none.

## 55. Units in the `Drive status` line

**Problem.** `IndraDrive::describe()` prints `deg` and `rpm`, but the vertical oscillator's
values are mm and 10 mm/min.

**Decision.** Put the labels in `IndraDriveConfig` (`position_unit = "deg"`,
`velocity_unit = "rpm"`), not in `describe()`'s signature. `IndraDriveActuator` sets them where
it fills `drive_config` from `drive_motion`. That way `describe()`'s callers
(`IndraDrive::enable()`'s error text, `mgate_example`) stay unchanged and get the right units
too. Use `"mm"` and `"10 mm/min"` for linear drives, as decided 2026-11-12. Change labels only,
not values: `position_deg()`/`velocity_rpm()` already return tag units.

**Changes.**
- `drivers/mgate5101_driver/include/mgate/indradrive.h`, `IndraDriveConfig`: the two
  `std::string` fields.
- `drivers/mgate5101_driver/src/indradrive.cpp`, `describe()`: `%s` for the two units, from
  `cfg_`.
- `rig_controller/src/indradrive_actuator.cpp`, where `drive_config` is filled: set them when
  `drive_motion` is `linear`.

**Test** (`test_indradrive_actuator.cpp`). A linear actuator's `IndraDriveActuator::status()`, which
returns `describe()`, contains `mm` and `10 mm/min` and not `deg`. The angular default still
says `deg` and `rpm`.

## 45. A drive fault logs a burst of lifecycle errors

**Decision: option (b), plus one line from the controller.** Keep today's path: deactivate,
`ERROR`, then `on_error` releases. The unit test `DriverFaultReleasesResourcesAndAllowsReconfigure`
covers it, and it releases the drive even with no lifecycle manager. Option (a) would make
releasing the drive depend on the manager. The burst then stays, but it is announced.

**Changes.**
- `controller_node.cpp`, `process_driver_fault()`: after `Driver fault: %s`, log once:
  `The lifecycle manager will now report this node as down; those errors follow from this fault.`
- `rpi/README.md`, *Troubleshooting*: a row "many `ERROR` lines, `CRITICAL FAILURE ... IS DOWN`
  → search for `Driver fault:` first; the rest follows from it", with the F2026 example.

**Test.** Extend `DriverFaultReleasesResourcesAndAllowsReconfigure`: the follow-up line is
logged once, after `Driver fault:`. It uses 36's log capture.

## 49. The page's heartbeat margin

**Decision: from the journal, no code unless the data says so.** Since 2026-11-11 the server
logs every gap (`No page acknowledged for ...`, `_check_idle()` area of `web_server.py`).
Before the next change:

```bash
journalctl -u wavemaker@ladertanken -u oscillator@ladertanken --since 2026-11-11 | grep 'No page acknowledged'
```

- No gaps over 0.5 s in normal sessions: keep 1.5 s and close the item.
- Gaps of 0.5–1.5 s that didn't stop a wave: raise `ACK_TIMEOUT_S` to 1.0 s (2.0 s in total).
  Change the comment by it to say why.
- Waves stopped by gaps while someone was watching: find the cause first (browser, Wi-Fi), as
  on 2026-11-11.

## 19. Linear drives (lilletanken, mclab): set their units

**What to set, when each drive is reachable.** From the drive's scaling parameters in
IndraWorks:
- `drive_m_per_unit` from S-0-0076 (position data scaling) and S-0-0123 (feed constant).
- `drive_velocity_per_unit_per_s` from S-0-0044 (velocity data scaling).
- `drive_direction` from S-0-0055 (polarity).
- `max_velocity_rpm` from the axis' allowed speed in the drive's units, as done for the
  vertical oscillator (TODO 51, COMMISSIONING.md *The oscillator drive*).

Then delete the stale `min_position_m`/`max_position_m` keys. Check with
`play_dat <file> <rig> --dry-run` that the printed limits are plausible, before any move. Do *Rig-agnostic configuration* (Planned features) first, so the
two rigs don't each copy a launch file.

## 61. Split `controller_node.cpp`

**Problem.** 1483 lines. `on_configure()` alone is about 490 lines, mostly reading and checking
parameters. The test includes the `.cpp` with `main` renamed
(`#define main controller_node_main`).

**Decision: extract the two pieces with their own logic, not a mechanical split.**
1. **`controller_config.{hpp,cpp}`.** A `ControllerConfig` struct and
   `ControllerConfig read_config(rclcpp_lifecycle::LifecycleNode &)`, holding everything
   `on_configure()` reads and validates, and throwing `std::invalid_argument` with today's
   messages. `on_configure()` already catches that and returns `FAILURE`, so the behaviour stays
   the same. `on_configure()` shrinks to "read config, build the actuator, create the
   interfaces". The parameter checks can then be tested without a lifecycle.
2. **`soft_stop_monitor.{hpp,cpp}`.** The subscription, its callback group, the liveness check
   and the logs from 36 and 56, behind `bool active() const`, `void check()` and
   `void reset()`. That is the safety-relevant part, and it becomes small enough to review on
   its own.
3. The goal execution stays in the node. It shares too much state (`goal_mutex_`, the worker)
   to move without changing behaviour.

**Changes.** The new files are added to the `controller_node` executable and the
`test_controller_node` target in `rig_controller/CMakeLists.txt`. The test keeps including
`controller_node.cpp`.

**Test.** All existing gtests pass unchanged. New: `test_controller_config.cpp`, one case per
rejected parameter, moved from `test_controller_node.cpp` where they exist there today.

## 60. The MGate driver's `test_driver` (corrected)

**Correction (2026-11-18).** The TODO entry said the Modbus layer is covered only by
`test_mapping`. That's wrong: `FakeMGate` (`rig_controller/test/fake_mgate.hpp`) is a real
Modbus/TCP server, so `MGateDriver` is tested end-to-end through `test_indradrive_actuator` and
`test_controller_node`. What only `test_driver` checks: registering a tag outside the area
throws, writing an input tag throws, and tags that straddle two registers decode correctly.

**Decision.** Move those three checks into `tests/test_mapping.cpp`, which needs no server for
the first two. For the straddling tags, check `process_image`'s decode directly if
`test_mapping` doesn't already. Then delete `tests/test_driver.cpp` and `tests/stub_server.py`
and the comment about them in `drivers/mgate5101_driver/CMakeLists.txt`. Keep `probe_map.py`
if it's still used against the real gateway. The priority is low.

## 52. `mgate_example`: step size, speed and limits; install it

**Problem.** `test_move()` has `kStepDeg = 5.0` and `kRpm = 10.0` fixed. On the vertical
oscillator that is 5 mm at 1.7 mm/s. The limits are `IndraDriveConfig`'s −550…500, so the
drive's own ±211 mm is the only backstop. The program is also not installed.

**Decisions.**
1. Add `--step`, `--speed`, `--min` and `--max`, all in tag units (deg/rpm or mm/(10 mm/min), as
   the drive reports).
   - `--step` defaults to 5 and may be negative (a move down first).
   - `--speed` defaults to 10 and is capped by `max_velocity_rpm` (50, the commissioning cap;
     `move_to()` already rejects anything faster).
2. **`--min`/`--max` are required with `--move`.** A missing limit is what makes −550…500 the
   only check today. The program refuses before enabling when `start + step` is outside them.
3. Install it (`install(TARGETS mgate_example DESTINATION lib/${PROJECT_NAME})`), so
   `ros2 run mgate5101_driver mgate_example ...` works.

**Changes.** `drivers/mgate5101_driver/examples/example_main.cpp`:
- `main()`'s option loop gains the four options. Parse them as doubles; there is a
  `parse_int` already, so add a `parse_double` next to it.
- `usage()` lists them.
- `test_move(IndraDrive&, double step, double speed)` replaces the constants. `dcfg.min_position`
  and `dcfg.max_position` come from `--min`/`--max`, so `move_to()` enforces them too.

Also `drivers/mgate5101_driver/CMakeLists.txt`: the `install` line.

**Test.** No unit test, because the program talks to a gateway. Check by hand:
- `--move` without `--min` is refused.
- A step outside the limits is refused before `enabling...`.
- `--step 50 --min -60 --max 60` on the vertical oscillator moves 50 mm and back.

## 62–65. Housekeeping

- **62. Sketches name functions.** No code. Say it in the TODO intro: "Sketches name the
  function or block each change goes in; line numbers are hints with their commit." This
  sketch follows it.
- **63. Docs entry point.** In README *Contents*, a table with one line per doc and who it's
  for:
  - README: users;
  - STARTUP: before each session;
  - COMMISSIONING: rig history and checks;
  - LAB_PERSONNEL: configuration and internals;
  - OSCILLATOR: the sloshing rig;
  - rpi/README and windows/README: the computers;
  - TODO and `*_sketch.md`: open work.

  Moving the sketches to `docs/sketches/` is optional. Their links from TODO.md and git
  history (`git show <commit>:<path>`) would need the new path.
- **64. Leftovers.**
  - `rmdir carriage_bringup/launch carriage_bringup` (untracked, empty).
  - Add `.pytest_cache/` to `.gitignore`.
  - Drop the `.gitignore` entries `OPPSTART.md` and `drivers/mgate5101_driver/compile_commands.json`
    (neither file exists).
- **65. Lint the Python.**
  - `ruff check rig_controller/src rig_controller/test rig_web` with a `ruff.toml`:
    line length 100, the style the files already use, and rules `E,F,W,I,B`.
  - Fix or silence the first run's findings in one commit, then add it to the pre-push hook (58).
  - `mypy` is not worth it yet: only some functions are typed.
