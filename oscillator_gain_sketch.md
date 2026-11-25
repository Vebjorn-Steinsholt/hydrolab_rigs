# Sketch: a stroke gain table for the oscillators

What to code so an oscillator reaches the stroke that was asked for at every period, and why.
The wavemaker already has a gain table; the oscillator sends the stroke unchanged. Three parts:
rename the gain parameters to rig-neutral names (part A), apply the table to oscillators
(part B), and measure the table properly with a new tool, `stroke_gain` (part C).

**Status (2026-11-25): not started.** None of the changes below are in the code. Measurements:
[COMMISSIONING.md](COMMISSIONING.md), Test 5, Test 6 and the lag item in *Open*.

Line numbers are as of `wave-probe` on 2026-11-25 (`824088e`).

## Problem

The vertical oscillator's slide moves less than asked at short periods (naked load, no water):

| Period | 3.0 s | 2.5 s | 2.0 s | 1.5 s | 1.2 s | 1.0 s | 0.8 s | 0.6 s |
|---|---|---|---|---|---|---|---|---|
| Stroke reached | 99.3 % | 98.8 % | 97.3 % | 95.9 % | 92.8 % | 89.5 % | 83.9 % | 72.4 % |

1.0, 0.8, 0.6 and 2.0 s: the bag `~/bags/margin_1.2` (2026-11-25). 3.0, 2.5, 1.5 and 1.2 s:
the endurance test (2026-11-20, catman), which gave 98.3 % at 2.0 s. Two methods and two days:
good enough to show the shape, not to set the table (part C).

- **The loss is in the drive**, between the target it receives (S-0-0258) and its own command
  (S-0-0047); the slide follows S-0-0047 exactly (oscilloscope, 2026-11-25). Neither the jerk
  limit (Test 5) nor the speed cap (Test 6) changes it.
- **It is linear:** at 1 s the stroke is 89.5 % for 10, 20, 30 and 40 mm. One factor per period
  corrects every stroke.
- **It is repeatable:** within 0.01 mm over the 5 h endurance test, and within 0.3 % between the
  three recordings of 2026-11-25.

A drive setting that removes the smoothing would make part B unnecessary; ask the drive's owner
first (lag item in *Open*). Part A is worth doing either way; part C measures what part B needs.

## What changes for the user

**Wavemaker yaml and launch:** the gain parameters lose the `wavemaker_` prefix. Old names stop
the controller at configure with a message naming the new one, so a stale yaml is never ignored
silently.

| Before | After |
|---|---|
| `wavemaker_transfer_gain` | `transfer_gain` |
| `wavemaker_transfer_gain_periods` | `transfer_gain_periods` |
| `wavemaker_transfer_gain_values` | `transfer_gain_values` |
| `ros2 launch … wavemaker_transfer_gain:=0.92` | `ros2 launch … transfer_gain:=0.92` |

**Oscillator yaml:** the vertical controller gets a table, from part C:

```yaml
    # Stroke reached / stroke asked per period (measured YYYY-MM-DD, stroke_gain). The command
    # is the goal / this, as the wavemaker's. To correct: new = old * measured / target.
    transfer_gain_periods: [0.5, 0.6, …, 3.0]
    transfer_gain_values:  [ … ]
    min_period_s: 0.5                 # the table's shortest period: nothing runs uncorrected
```

A goal of 10 mm at 0.6 s from the page commands about 13.8 mm and the slide moves 10 mm. Files
from `generate_oscillations` are corrected the same way. Without a table (the sloshing
oscillator) nothing changes.

## Decisions

1. **Convention: the gain divides,** and is the fraction reached (0.895 at 1 s), as the
   wavemaker's (`wave_trajectory.cpp:54`, `generate_waves.paddle_amplitude`). One correction
   rule for both rigs: new = old × measured / target.
2. **Full rename to `transfer_gain`, `transfer_gain_periods`, `transfer_gain_values`,** in one
   commit with no aliases. The old names are declared only to refuse them (change A2): a renamed
   parameter that ROS ignores without a word would silently drop the wavemaker's gain table and
   make every wave 5-15 % wrong. This also settles that part of TODO "Rig-neutral renames"
   (line 398).
3. **Oscillators use only the table**, not the single `transfer_gain`. The loss depends strongly
   on the period; one number would be wrong almost everywhere.
4. **No goal runs uncorrected:** set `min_period_s` to the table's shortest period, so the
   controller rejects shorter periods instead of using the end value. Measure down to the
   shortest period the experiments use. Above the longest period the end value is used (the
   loss is under 1 % there).
5. **Stroke only, not timing.** The ~170 ms lag stays. Correcting it would mean starting the
   motion earlier, which only matters if the motion must be in sync with something else.
6. **The limits apply to what the drive is sent, the corrected stroke; reject, never clip.**
   - Rejecting keeps both promises: the slide moves what was asked, and nothing goes past the
     speed and travel limits. Clipping the correction instead would quietly deliver less than
     asked, which is what the table exists to stop.
   - The rejection must say why. Today it reads like the goal is too fast; with a table it
     should name the goal and the command: "10.0 mm at 0.60 s needs 13.8 mm commanded (gain
     0.724): peak 0.072 m/s above …". `generate_oscillations` skips such a run with the same
     reason, and its run table shows the gain.
   - The limits must grow for the experiments: their largest motion, 55.7 mm amplitude at
     0.5 s (0.70 m/s, 8.8 m/s² at the slide), needs about 1.08 m/s and 13.5 m/s² commanded with
     the table (gain ≈ 0.65 at 0.5 s, extrapolated). Today's 0.45 m/s rejects it even
     uncorrected. Raising `max_velocity_rpm` needs a speed test first (see *Open*).
   - `peak_velocity()` and the travel checks already use the corrected amplitude, so only the
     messages change (B4).

## Part A: rename (no behaviour change)

1. `controller_node.cpp`: declarations (lines 97-101), `get_parameter` (196-198), checks and
   messages (271-306), the member (1430). Search for `wavemaker_transfer_gain`; 16 hits.
2. `controller_node.cpp`: declare the three old names (default: not set) and fail configure if
   any is set: "wavemaker_transfer_gain_periods was renamed to transfer_gain_periods".
3. `wavemakers.yaml` (lines 51-63, 79, 94) and its comments.
4. Launch files: `lab.launch.py` (42, 67), `bridge.launch.py` (42, 55), `wavemakers.launch.py`
   (32-38, 126): the argument becomes `transfer_gain`. Check what `ros2 launch` does with the
   old argument; if it ignores it, say so in LAB_PERSONNEL.
5. `generate_waves.py` (44, 86-88) and the docstring of `play_dat.py` (12).
6. Tests: `test_controller_node.cpp` (11 hits), `test_generate_waves.py` (142). New: an old name
   set fails configure, with the new name in the message.
7. Docs: LAB_PERSONNEL (71, 167, 171, 180), COMMISSIONING (the instructions, not the dated
   log), TODO (156; 398 marks the gain renamed), `gain_estimation_sketch.md` (17), README.

## Part B: the table for oscillators

### Controller (C++)

1. `controller_node.cpp:868`: look up the table for both rigs, not only `rig_ == "wavemaker"`.
   For an oscillator, start from 1.0 instead of `transfer_gain_` (decision 3).
2. `controller_node.cpp:271-298`: move the table checks (equal length, at least 2 points,
   finite and > 0, strictly ascending) before the wavemaker branch so they run for both rigs.
   `transfer_gain` and `water_depth` stay wavemaker-only. For an oscillator with a table, warn
   at configure if `min_period_s` is below the table's first period (decision 4).
3. `wave_trajectory.cpp:45-47`: for an oscillator, `amplitude_ = wave_height / 2.0 /
   geometry.transfer_gain`. Update the comment ("no dispersion relation or flap scaling; the
   gain divides, as the wavemaker's").
4. The speed, acceleration and travel rejections: when the gain is not 1, add the goal, the
   commanded stroke and the gain to the message (decision 6). Goals with `positions` (from
   `play_dat`) are not touched, so files are not corrected twice.

### `generate_oscillations` (Python)

5. Load the table from the oscillator's yaml block (as `generate_waves.load_wavemaker`), empty
   if absent.
6. Move `generate_waves.transfer_gain()` (line 96) to `motion_files` and use it from both, so
   files and live goals interpolate the same way (`np.interp`, nearest end value outside).
7. `regular_signal()` (line 153): `amplitude = spec.stroke_m / 2.0 / gain(period_s)`.
8. `irregular_signal()` (line 135): divide each component, `a / gain(1 / f)`, before
   `synthesize`, as `generate_waves.paddle_amplitudes`. `sig_stroke_m` stays the slide's
   significant stroke, not the command's.
9. Run table: a `gain` column (regular: the run's; irregular: at `tp_s`), as `generate_waves`
   line 312. `amplitude_m` becomes the commanded amplitude; `stroke_m` stays the goal. A skipped
   run's reason names the commanded stroke (decision 6).
10. A period below the table's first, or an irregular band (0.5-3 × fp) reaching below it: skip
    the run if `min_period_s` would reject it, else warn. `<output>.txt` records the table, or
    "none".

### Docs

11. README, oscillator sections: the table, its convention, that the checks see the corrected
    stroke, and how to re-measure it (part C). `oscillator.yaml`: the comment above.

### Tests

- `test_wave_trajectory.cpp`: oscillator with gain 0.5 → twice the amplitude; gain 1.0 →
  unchanged; peak velocity scales with it.
- `test_controller_node.cpp`: an oscillator with a table interpolates (0.7 s → between 0.6 and
  0.8 s values); an oscillator without a table is unchanged; a bad table fails configure for an
  oscillator too; a goal that fits uncorrected but not corrected is rejected, and the message
  names the commanded stroke; a `positions` goal is not scaled.
- `test_generate_oscillations.py`: regular amplitude = stroke / 2 / gain; an irregular run's
  spectrum divided per component (compare with `synthesize_direct`); no table → identical files
  to today; the `gain` column; a run skipped on its corrected speed, with the reason.
- `test_generate_waves.py`: unchanged results after moving `transfer_gain()`.

## Part C: measuring the table

### The tool: `stroke_gain`

A `rig_controller` script, from the bag and the motion file's run table (the prototype,
`baglag.py`, did this for one file on 2026-11-25):

```bash
ros2 run rig_controller stroke_gain gain_sweep.dat ~/bags/gain_sweep_1 ~/bags/gain_sweep_2 -o gain_sweep
```

- **Input:** the `.dat` and its run table (`<file>.csv`), one or more bags of that file played
  (topics `<ns>/oscillator_setpoint` and `<ns>/oscillator_position`).
- **Per run:** find the run in the bag (the setpoint against the file, as `transfer_function`
  aligns probe and command); skip the ramps and one period; fit harmonics 1-9 at the run's
  period to setpoint and position; stroke = fundamental, gain = position / setpoint, lag from
  the phase, distortion = harmonics 2-9 / fundamental.
- **Time base:** the controller's 50 Hz cycle, not the PC's receive time (its jitter is up to
  18 ms). Refuse a run with a gap in the messages; count samples against the run's duration.
- **Output:** `<o>.csv` with one row per run and bag; `<o>.txt` with the table per period
  (mean, spread, number of runs), the yaml lines to paste, and the commit and bags it came from.
  Flag a period whose runs spread more than 0.5 %, and strokes whose gains differ more than 1 %
  at one period (the loss would not be linear).
- Tests with a synthetic bag: a known gain and lag come back; a gap is refused; repeats are
  averaged.

### The test (COMMISSIONING, when it is run)

**Session file:** regular runs, 20 periods, 2 ramp periods, 10 s pauses. The experiments use
0.5-1.5 s (asked 2026-11-25), so dense there, with 2.0 s as a margin: 0.5, 0.55, 0.6, 0.65,
0.7, 0.75, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5, 2.0 s.

- 10 mm at every period: the table.
- 30 mm at 2.0, 1.0 and 0.6 s: the loss is the same at another stroke (linearity).
- Peak speed at most 0.16 m/s (30 mm at 0.6 s), 0.22 m/s with the table in step 6; jerk
  warnings below 0.6 s only (10 mm at 0.5 s: 9.9 m/s³, a warning).

**Steps:**
1. No table in the yaml, `positioning_velocity_margin` 1.2, S-0-0193 3000.
2. Play the file twice, each with its own bag (*Procedures*, step 6).
3. Once, catman `6047 Position` with the first play: `oscillator_position` is the drive's
   encoder, catman is the slide (on 2026-11-25 they agreed within 0.3 %).
4. `stroke_gain` on both bags. Pass: the two plays within 0.5 % per period; 10 and 30 mm within
   1 %.
5. With the experiment's load and water: play once more. If any period differs by more than 1 %
   from the naked load, the table comes from the loaded play; if not, the naked one stands and
   the result says so.
6. Paste the table and `min_period_s` into the yaml, restart, and play the file again. Pass:
   every run within ±1 % of its goal, the lag unchanged (the gain fixes the stroke only).

## Open

- The experiments use 0.5-1.5 s; their largest motion is 55.7 mm amplitude at 0.5 s (0.70 m/s,
  8.8 m/s², 110 m/s³ at the slide; 2026-11-25). With the table: ≈1.08 m/s, 13.5 m/s²,
  170 m/s³ commanded. Before that can run:
  - The ball nut's maximum speed from its datasheet (≈1.0 m/s assumed from d·n).
  - The moving mass against the motor's peak torque (1 Nm ≈ 280 N at 20 mm/rev).
  - Whether S-0-0193 rounds such a motion off (Test 5 only reached 6 m/s³).
  - A stepwise speed test at 0.5 s from 0.45 m/s up, with accelerometer and drive current.
  - If the drive's owner removes the smoothing in 0x001B, no table is needed and the rig only
    has to reach 0.70 m/s and 8.8 m/s²: ask that first.
- Does the experiment's load or water change the table? Part C step 5 answers it.
- The sloshing oscillator (LinMot): the same table would work if it loses stroke too; not
  measured.
- Should the timing be corrected too (decision 5)? Ask the experimenters.
- Ask the drive's owner what in 0x001B smooths the target first; if a setting removes it, part B
  is not needed (part A still is).
