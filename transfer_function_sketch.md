# Sketch: transfer gain from an irregular run

What to code to get the wavemaker's transfer gain from a single irregular run, and why. The
measurements and their history are in [TODO.md](TODO.md) item 43 and
[rpi/README.md](rpi/README.md), *`generate_waves` on the rig*.

Line numbers are as of `feat-transfer-function` on 2026-11-13. The program is written
([transfer_function.py](rig_controller/src/transfer_function.py)) and passes its synthetic
tests; what is left is in *Still to do*.

## Problem

Today each regular run gives one point of `wavemaker_transfer_gain_values`. The table at 1.5 s
rests on two runs that disagree (0.75 and 0.78). Each run costs a pause for the tank to settle,
and its measuring window is cut short by the beach reflection after about 21 s. An irregular run
has energy at every frequency in its band at the same time. From one run (run 3 of `item38.dat`)
we can get the gain across about 0.8–2.4 s, with an error bar at each period.

**What changes for the user.** A new program in `rig_controller`, next to `generate_waves` and
`play_dat`, run on the motion file and a catman record. Not a change on the rig:

```bash
ros2 run rig_controller transfer_function item38.dat ladertanken --run 3 \
    --record item38_catman.csv --probe WP1 --probe-distance-m 3.0 \
    --stroke Stroke --stroke-height-m 1.20 --depth-m 0.98 -o item38_run3_tf --plot
```

`--record` is the catman CSV export, with the probe and the flap's stroke as columns. The tool
finds where the file starts in the record by itself, prints the gain per period next to the
yaml's table, the drive's response and the water's, and writes `item38_run3_tf.csv` (the rows),
`.txt` (what it was made from) and `.png`. Without `--stroke` it gives the command-to-wave gain
only.

### Decisions

1. **Self-contained in this repo.** Everything it needs is on the command line or in files this
   repo already writes: the `.dat` and its run table from `generate_waves`, the wavemaker from
   `wavemakers.yaml`, and one catman record (the probes and the flap's stroke). No session
   files and no other analysis code.
2. **The record as catman's ASCII export** (decided 2026-11-12, changed 2026-11-13). catman
   has no CSV export; its ASCII export starts `HBM_CATMAN_DATAFILE_40`, has a header of about 36
   lines (names, units, `T0`, `dt =5 ms`, calibration and filters), tabs, decimal commas, and a
   time column rounded to 0.01 s. `read_record` recognises it by its first line, takes the names
   and `dt` from the header and drops the time column. A CSV with time first still works (the
   tests use it). Channels are chosen by name (`--probe WP1`, `--stroke WM_Position`).
3. **The run from the run table, not detected.** `<file>.csv` already has each run's `start_s`,
   `end_s`, `kind` and, for irregular runs, `peak_period_s`, `f_min_hz` and `f_max_hz`
   (`WAVE_COLUMNS`, [generate_waves.py:316](rig_controller/src/generate_waves.py#L316)). Use
   those for the window and the band. A `.dat` without a run table is refused.
4. **H1 estimator, with the command as input.** `G = S_x,η / S_x,x`. The command is exact and
   the noise is on the probe, which is the case H1 is unbiased for. With a measured paddle
   position as input (later), H1 is biased low. Report H2 = `S_η,η / S_η,x` as well; the true
   value lies between them.
5. **Gain = |G| / |G_lin|, compared with linear theory at gain 1.0.** The command already has
   the gain table applied (`paddle_amplitudes`,
   [generate_waves.py:269](rig_controller/src/generate_waves.py#L269)), but that cancels out: G
   is measured against the x that was actually sent. So the result is the new table value
   directly, not a correction to multiply the old one by.
6. **The theory is imported from `generate_waves`, not copied** (`wavenumber_depth`,
   `stroke_ratio`, `load_wavemaker`). This way the analysis uses the same physics as the
   controller and the generated files.
7. **`--depth-m` is required.** No default from the yaml: depth is one of the open confounders
   in item 43, so the user must state the measured one.
8. **Only trusted periods are reported.** That means inside the run's band (`f_min_hz` to
   `f_max_hz` from the run table), below 1.5 fp (`F_TOP_FP`), and with coherence ≥ 0.8. At the
   band edges there's too little energy. Around 2 fp the peak's second-order waves land: in
   `item38` run 3 the gain swung between 0.45 and 0.9 below 0.75 s with coherence still above
   0.8, so coherence alone doesn't catch them (2026-11-13). Those periods are left out, not
   corrected.
9. **The gain table is not changed by the tool.** It prints values, and they go into
   `wavemakers.yaml` by hand after the comparison with the regular runs, as item 43 says ("Leave
   the gain table alone until then").
10. **Regular runs through the same tool.** `--run 0` on a regular run reports G at its one
    period (1 / `period_s`). The regular points to compare with come from the same code, not
    from numbers typed in.
11. **The ramp length is recorded in the run table** (decided 2026-11-12). Today it isn't:
    `n_ramp` is in the test matrix only, and the window can't skip the ramps without it.
    `generate_waves` and `generate_oscillations` write `ramp_s` to `<file>.csv`: the length of
    each ramp in seconds, as `apply_ramps` applied it (whole samples), so it needs no period to
    convert and means the same for regular and irregular runs. Both generators, so the two run
    tables stay alike and the oscillator's lag (TODO item 28) can go through the same tool
    later. A run table without `ramp_s` (files made before the change, such as `item38.csv`) is
    refused, with the hint to generate the file again from its test matrix. The seeds are fixed,
    so the motion is the same, and it also gives `item38` its missing `.txt`.
12. **The offset is found from the record** (decided 2026-11-12). The trigger channel isn't
    played yet (TODO *Trigger output*), so catman can't record it. The match covers the whole
    file (all runs and the rests between them): the pattern of runs and rests fixes one offset
    for the whole record, and each run's offset is that plus its `start_s`.
    - **With the stroke** (the normal case, decision 15): cross-correlate the command with the
      stroke, waveform against waveform. Between them there is only the drive's lag (about
      0.1–0.2 s), no travel time, so the offset lands within that lag.
    - **Without it** (fallback): match the envelope of the command against the probe's envelope,
      minus the travel time at the peak period; right to about 1 s. Envelopes, not the waves:
      matching the waves would remove the travel time, which is real and belongs in G's phase.
    - `--offset-s` overrides both, for records the match can't place (for example a single run
      played with `--runs`).

    Either way the offset's error is the drive's delay (or more), so the record alone can't give
    the drive's absolute delay. That needs the bag or a played trigger (*Later*).
13. **Reflections: accept the ripple and smooth over it in the first pass** (decided
    2026-11-12). The beach reflection adds a ripple to |G| that repeats every 1/τ_r in
    frequency, where τ_r is how much later the reflection reaches the probe than the incident
    wave. In `item38` it arrived about 21 s into the run at 1.5 s, about 19 s after the incident
    wave (3 m at c_g ≈ 1.3 m/s), so 1/τ_r ≈ 0.05 Hz. Average the gain over a moving band of
    `--smooth-hz` (default 0.05) to cancel the ripple, and say so in the output. Separating the
    incident wave with three probes comes later.

    Within a segment of length T only the part `1 − τ_r/T` of the reflection lines up with its
    command, so the raw ripple shows at about `r · (1 − τ_r/T)`: ±5 % rather than ±10 % for
    r = 0.1 in 40 s segments. The rest becomes noise (lower coherence, a larger ±). The smoothing
    cancels the ripple whatever its size; what is left is about r²/4 (0.25 % at r = 0.1). So
    keep 40 s segments for the gain, and use `--segment-s 100` only to read r from the raw
    ripple (2026-11-13, from the synthetic test).
14. **The program lives in `rig_controller`** (decided 2026-11-12), next to `play_dat` and
    `generate_waves`, whose code it imports. It doesn't touch the controller.
15. **The flap's stroke from catman splits the chain** (decided 2026-11-12). A draw-wire
    displacement sensor on the top of the flap, positive towards the beach, recorded by catman
    with the probes: same clock, same rate, and a separate sensor, not the drive's own feedback.
    With it the tool reports three transfer functions:
    - `G_cmd = S_x,η / S_x,x`, command → wave: the gain table (decision 5).
    - `G_drive = S_x,s / S_x,x`, command → flap: drive and linkage. The "paddle 0.572 of the
      command" in TODO 43 is this at one period; part of it may be geometry (the top moves more
      than the attachment point), so compare both at the waterline.
    - `G_water = S_s,η / S_s,s`, flap → wave: the hydrodynamics against linear theory. Stroke
      and probe are sampled together, so its phase is exact, independent of the offset.

    The sensor's height is a command-line argument, `--stroke-height-m` (above the tank floor),
    not a `wavemakers.yaml` parameter: it belongs to this measurement setup, not to the
    wavemaker. Required with `--stroke`. The stroke is scaled to the waterline like the
    attachment point: `x_wl = s · (h − hinge) / (z_sensor − hinge)`. The draw wire's zero is
    arbitrary, so its mean is removed (`detrend='constant'`); a small angle is assumed (the wire
    measures the top's horizontal travel).
    `G_water` uses the measured stroke as input, so it has noise on both sides: report H1 and H2
    (decision 4).

16. **The travel to the probe is removed before the estimate** (decided 2026-11-13, found by
    the synthetic test). A wave takes `distance / c_g` to reach the probe: about 2 s at 0.8 Hz
    and 7 s at 1.8 Hz, at 3 m. Within one Welch segment, that first part of the probe signal
    belongs to the command of the segment before, so |H1| comes out low by about
    `travel / T`: 18 % at 1.8 Hz in 40 s segments, growing towards short periods, where the
    gain table matters. The test gave water 0.68 for 0.83 at 1.8 Hz.
    - **The fix:** advance η by linear theory's travel, separately at each frequency
      (`undo_travel`: `e^{+ikd}`, an all-pass filter). |G| isn't touched; only the arrival
      time is. The remaining bias is `(error in the travel) / T`, a fraction of a percent.
    - **Not chosen:** longer segments (the bias stays `travel / T`, and there are fewer averages,
      so a larger ±); one shift by the travel at the peak period (dispersion leaves short periods
      seconds late).
    - **What it relies on:** `--probe-distance-m` and `--depth-m`. An error in either shows as a
      slope in `G_water`'s residual phase, which the tool reports (*Reading the output*).
    - **Its consequences:** the residual phase is `angle(G)` minus π/2 (linear theory: the wave
      follows the paddle's velocity, so it leads the displacement by 90°; `item38` showed
      1.45 rad before this was subtracted), brought to 0 by the
      nearest 2π (`np.unwrap` keeps whatever multiple the first bin starts at). The window skips
      only the ramps, since the waves are aligned.
    - **The check on real data:** a regular run doesn't have this bias (a steady wave looks the
      same whichever segment it is compared in), so its gain must match the irregular run's at
      the same period (*Check on the data*, step 3).

### Where the code differs from the first sketch

Why the code does a few things differently from what the first sketch said:

- `match_quality` compares against the next-best *local* peak (`find_peaks`). "The largest
  value more than one rest away" would pass for the stroke, but for envelopes the main peak is
  as wide as a run (hundreds of seconds). A value one rest away is then still about 0.9 of the
  peak, and every envelope match would be refused.
- `linear_ratio` takes `ok`, so `band_mask` comes before it in `analyse`. On all of
  Welch's bins it would crash: they go up to 25 Hz, where kh is about 2500, and `math.sinh` in
  `stroke_ratio` raises `OverflowError`.
- `analyse` refuses a record shorter than the file without `--offset-s`. scipy's `'valid'`
  mode swaps the inputs then, so the match wouldn't crash, but it would place a part of the
  file's run pattern, not the whole (decision 12). `cut_window` refuses a run outside the record
  at the offset, which `--offset-s` can give. Python's slices would wrap a negative start and
  clip a late end without an error.
- `frf` computes the coherence from the spectra it already has, which is the same as
  `signal.coherence`, and counts `n_avg` exactly as `welch` splits the window.
- Windows are cut as `[skip:n - skip]`, not `[skip:-skip]`. With `skip = 0`, `[0:-0]` is empty.
- The ± columns are absolute (gain × eps / √n_smooth), not relative.
- A regular run is never smoothed (`n_smooth = 1`): it has only one bin.
- `check_still` compares against the run's own std (`STILL_RATIO`, with `STILL_FLOOR_M` as a
  floor), not a fixed `STILL_STD_M`. See *Reading the output*.
- `analyse` holds what was `main`'s steps 1–9, so one `try` catches every `ValueError`.
- The travel is removed from η before the estimate (decision 16), not subtracted from the
  phase afterwards, and the window skips only the ramps.

### Still to do

1. **Find which reading is off: the command, the draw wire or WP1** (*Check on the data*,
   step 7). Inclinometer test, Monday 2026-11-16.
2. **Commit** `transfer_function.py`, `test_transfer_function.py`, `rpi/README.md` and this
   sketch (uncommitted on `master` since 2026-11-13).
3. **The next irregular session**, after step 1: one 10-minute irregular run at the tank's
   current depth (±1–2 % on the gain), and a mild regular run at 1.1 s.

The tests are done:
[test_transfer_function.py](rig_controller/test/test_transfer_function.py), 18 tests on a
synthetic session with a known drive (0.9, a 77 ms lag, 100 ms delay) and water (0.83 × theory,
3 m), at 200 Hz with 30 s of still water before the file. They cover the offset three ways, the
recovered drive, water and gain within their ±, the flat residual phase, the chain check, the
regular run, a reflection (19 s, 0.1) smoothed away, and every refusal. If one fails, the error
is in the code, not the tank.

### Reading the output

- **`gain`** is |G_cmd| over theory: the number for the gain table. **`drive`** is |G_drive|
  and **`water`** is |G_water| over theory, both at the waterline and both 1.0 when ideal (only
  with `--stroke`). The ± columns are absolute: the value × eps / √n_smooth.
- **Drive below 1 and falling with frequency** is the drive lagging (the oscillator showed 97 %
  at 2 s and 90 % at 1 s unloaded, TODO 46).
- **Water about 1 and drive about the table's gain** means the table is correcting the drive,
  not the water. Worth knowing: the drive's response may change with load; the water's doesn't.
- **`chain`**, the largest difference between G_cmd and drive × water: a few % at most. A large
  one means the stroke and the probe don't belong together (wrong column, wrong scale).
- **`delay`** (G_cmd's residual phase as a straight line) is the drive's delay plus the
  offset's error, so it's no measure of the drive. It checks the offset: well under 1 s without
  the stroke, and about the drive's lag (0.1–0.2 s) or less with it.
- **`water delay`** is free of the offset (stroke and probe are sampled together): 0 when the
  distance and the depth are right. A slope is an error in `--probe-distance-m` or `--depth-m`
  (decision 16).
- **Curvature in a residual phase** is a lag; a straight line is a pure delay.
- **Coherence** below 0.8 removes a bin (decision 8): at about 2 fp from second-order waves, and
  at the band's edges from too little energy. The bins at the edges also have the largest ±.
- **The stillness warning** compares the probe's std in the 10 s before the run with 10 % of the
  run's, with a 1 mm floor. Set the floor from a still stretch of a real record.
- **Segments**: `--segment-s 40` gives df = 0.025 Hz and about 14 averages on a 300 s run.
  Longer segments resolve finer and show the reflection's ripple fully (decision 13), with a
  larger ±.

### Later, not in the first pass

- With three probes, separate the incident and reflected waves (Mansard & Funke 1980) and use
  the incident wave as η, in place of the smoothing (decision 13).
- The drive's absolute delay, which the record can't give (decision 12): `--bag <bag>` with
  `wavemaker_setpoint` and `wavemaker_position`, published in the same control cycle
  ([controller_node.cpp:1344-1350](rig_controller/src/controller_node.cpp#L1344-L1350)). Pair
  them by message order, not receive time (the messages are `Float64`, no stamp, and the bags
  arrive in bursts, COMMISSIONING.md, speed tests). Or a played trigger recorded in catman (TODO
  *Trigger output*).
- Read catman `.bin` directly, which saves the export step.

### Check on the data

Done 2026-11-13 on `item38` (`item38.ASC`, catman ASCII; run table regenerated as
`item38_new`), and on a 4 s regular run made for the draw wire (`drawwire.dat`, 2026-11-13,
depth 1.0 m, catman `test38.ASC` and a bag of the setpoint and position).

1. **Regenerated `item38`: identical** to the played `.dat` (`cmp` on the Pi).
2. **Run 3: works.** Offset 26.3 s from the draw wire (`wave_analysis`: 26.4 s), 25.9 s from the
   envelopes. Coherence 0.91–1.00 over 0.8–1.9 s. `water delay` 0.03 s, so the 3.0 m distance
   and the 0.98 m depth are right. `chain` 1 %.
3. **Regular runs: agree with `wave_analysis`** to the third decimal: run 0 (1.0 s) 0.823, run 1
   (1.5 s, the whole run with the reflection) 0.870, run 2 (1.3 s) 0.798; `wave_analysis`'s
   measured/target × table gives 0.824, 0.870, 0.798. Against run 3: 1.0 s within 2 %; 1.3 s
   5 % lower, but run 2 is very steep (10 % second harmonic).
4. **The gain** (run 3, depth 0.98 m): 0.78–0.84 over 0.8–1.8 s (0.81 ± 0.01 at 1.0 s, 0.84 at
   1.2–1.3 s, 0.83 ± 0.06 at 1.5 s); the table has 0.70–0.80, with 0.71 at 1.1 s. So the table
   is 3–15 % low and the waves come out too high, as every earlier measurement found (H/target
   1.03–1.12, Hm0 0.0347 for 0.030). At 1.5 s the ± is too wide to choose between 0.75 and 0.78;
   both look low. **Not put into `wavemakers.yaml`** (decision 9) until step 7 is settled.
5. **The drive follows its setpoint:** the bag of the 4 s run gives `wavemaker_position` /
   `wavemaker_setpoint` = 0.999, 0.10 s behind. Both go through `actuator_lead_m_per_degree`
   (0.00053 m/deg), so this can't check that constant.
6. **The draw wire (`WM_Position`, 1.375 m above the floor) is linear but off in scale.** Its
   ratio to the command is 0.567–0.574 in all five runs, at amplitudes from ±0.015 to ±0.137 m
   and periods from 0.8 to 4 s, with less than 1 % bend at the extremes. So the arm and its
   attachment are linear. Geometry predicts 1.062 ((1.375 − 0.09) / (1.30 − 0.09)); the reading
   is 0.54 of that. `WM_Position` counts positive away from the beach (`--stroke-scale -1`).
   Pulled by hand 30 cm, the wire read 0.368 m (rough: 1.1–1.35 × the pull), so it doesn't
   read half, as first suspected.
7. **Open: the command, the draw wire or WP1.** With the wire's own scale, the flap moves about
   0.40–0.50 of the command. That would put the error in `actuator_lead_m_per_degree` (about
   2.3 cm per 100 deg, not 5.3), but then the flap would make about 1.9 × linear theory's wave
   per metre of real stroke, which a flap doesn't do. So one of three is off: the command (the
   constant), the draw wire (not measuring the flap's horizontal travel 1:1), or WP1 (its scale
   making the waves look larger; the October table, also from WP1, would hide that). **Test,
   Monday 2026-11-16:** an inclinometer on a rigid part of the flap at upright, then at a
   position held by Stop during `drawwire.dat`, with catman and the bag recording. At 0.10 m at
   the attachment the controller predicts 4.7 deg of tilt, a flap moving 0.44 of the command
   2.1 deg. Compare the tilt from the inclinometer (B − A), from the last setpoint
   (setpoint / 1.21 m) and from the draw wire (`WM_Position` change / 1.285 m): the two that
   agree are right. If WP1 is the suspect, film a ruler on the tank wall at WP1 during a
   regular run.

## Limits of the first pass

- The drive and water split (`drive`, `water`) can't be used until step 7 of *Check on the
  data* is settled. The gain doesn't depend on it.
- The drive's absolute delay isn't measured (decision 12); the drive's response is, apart from
  that straight line in its phase.
- Run 3 is a low wave (Hs 0.03 m), so it measures the linear gain only. Steepness and depth
  effects need regular runs and the GP step.
- `undo_travel` uses linear theory's dispersion. For steep waves the real speed is a little
  higher; at Hs 0.03 m that's negligible.
