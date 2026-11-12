# Sketch: transfer gain from an irregular run

What to code to get the wavemaker's transfer gain from a single irregular run, and why. The
measurements and their history are in [TODO.md](TODO.md) item 43 and
[rpi/README.md](rpi/README.md), *`generate_waves` on the rig*.

Line numbers are as of `master` on 2026-11-12.

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
2. **The record as a CSV export** (decided 2026-11-12): catman's ASCII export, a time column in s
   and one column per channel, chosen by name (`--probe WP1`, `--stroke ...`). One small,
   testable reader with no dependency on the catman format.
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
   `f_max_hz` from the run table) and with coherence ≥ 0.8. Low coherence at about 2 fp comes
   from second-order waves, and at the band edges there's too little energy. Those periods are
   left out, not corrected.
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
    the drive's absolute delay. That needs the bag or a played trigger (Change 11).
13. **Reflections: accept the ripple and smooth over it in the first pass** (decided
    2026-11-12). The beach reflection adds a ripple to |G| that repeats every 1/τ_r in
    frequency, where τ_r is how much later the reflection reaches the probe than the incident
    wave. In `item38` it arrived about 21 s into the run at 1.5 s, about 19 s after the incident
    wave (3 m at c_g ≈ 1.3 m/s), so 1/τ_r ≈ 0.05 Hz. Average the gain over a moving band of
    `--smooth-hz` (default 0.05) to cancel the ripple, and say so in the output. Separating the
    incident wave with three probes comes later.
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

### Changes

1. **`rig_controller/CMakeLists.txt`.** Install the program as the others are, and install
   `generate_waves.py` as a module too, since the new program imports it (today only
   `play_dat`, `rig_limits` and `motion_files` are, line 84):

   ```cmake
   # Transfer function and gain from a run and a catman record (see the script's docstring).
   install(PROGRAMS src/transfer_function.py
     DESTINATION lib/${PROJECT_NAME}
     RENAME transfer_function
   )

   install(FILES src/play_dat.py src/rig_limits.py src/motion_files.py src/generate_waves.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

2. **`generate_waves.py` and `generate_oscillations.py`: `ramp_s` in the run table** (decision
   11). In both generators' `regular_signal` and `irregular_signal`, keep the sample count that
   goes to `apply_ramps` and record it; add the column after `kind` in `WAVE_COLUMNS` and
   `OSCILLATION_COLUMNS`:

   ```python
   n_ramp_samples = round(spec.n_ramp * spec.tp_s / dt)      # spec.period_s for regular
   x = apply_ramps(x, n_ramp_samples)                        # shape='cosine' in oscillations
   row = {'kind': 'irregular', 'ramp_s': round(n_ramp_samples * dt, 6), ...}

   WAVE_COLUMNS = ['run', 'start_s', 'end_s', 'kind', 'ramp_s', ...]
   ```

   `play_dat` ignores columns it doesn't use, so existing playback is unchanged.

3. **`rig_controller/src/transfer_function.py`: inputs.** The command with `play_dat`'s readers
   (`read_motion`, `select_channel`, `read_run_table`); don't write a second parser. Keep the
   whole file's x for the offset (Change 4), and cut the run with its `start_s`/`end_s`:

   ```python
   motion = play_dat.read_motion(args.file)
   x_file = play_dat.select_channel(motion, 'position')    # m at the attachment, from upright
   dt = motion.sample_interval_s
   row = next(r for r in play_dat.read_run_table(args.file) if int(r['run']) == args.run)
   i_start, i_end = round(float(row['start_s']) / dt), round(float(row['end_s']) / dt)
   x = x_file[i_start:i_end]
   ```

   The record CSV: check that the time column is uniform and give its rate. Decimate each
   channel to the file's rate (200 Hz → 50 Hz) with an anti-alias filter, not by taking every
   4th sample:

   ```python
   t, channels = read_record_csv(args.record)              # {name: values}, by column header
   q = round(dt / (t[1] - t[0]))                           # 4 for 200 Hz against 0.02 s
   def at_file_rate(y):
       return signal.decimate(y, q, ftype='fir', zero_phase=True)
   eta_all = at_file_rate(channels[args.probe]) * args.probe_scale      # m
   s_all = at_file_rate(channels[args.stroke]) * args.stroke_scale if args.stroke else None
   ```

   Check the units of both channels (mm or m: `--probe-scale`, `--stroke-scale`) and refuse a
   column name that isn't in the header, listing the ones that are.

4. **`transfer_function.py`: the offset** (decision 12). With the stroke, the waveforms
   themselves:

   ```python
   lags = signal.correlation_lags(len(s_all), len(x_file), mode='valid')
   c = signal.correlate(s_all - s_all.mean(), x_file - x_file.mean(), mode='valid')
   file_offset_s = lags[np.argmax(c)] * dt          # includes the drive's lag, ~0.1-0.2 s
   ```

   A negative peak means the stroke's sign is the opposite of the command's: refuse, and say
   which way the command's + points (check it against "positive towards the beach").

   Without the stroke, envelopes smoothed over a few peak periods, so the carrier and its phase
   don't take part; then the lag where they match best:

   ```python
   def envelope(y, dt, smooth_s):
       return ndimage.uniform_filter1d(np.abs(signal.hilbert(y - y.mean())), round(smooth_s / dt))

   env_x = envelope(x_file, dt, 3 * tp_s)
   env_eta = envelope(eta_all, dt, 3 * tp_s)
   lags = signal.correlation_lags(len(env_eta), len(env_x), mode='valid')
   c = signal.correlate(env_eta - env_eta.mean(), env_x - env_x.mean(), mode='valid')
   travel_s = args.probe_distance_m / c_g(1 / tp_s, args.depth_m)   # at the peak
   file_offset_s = lags[np.argmax(c)] * dt - travel_s
   ```

   - The record must cover the whole file; refuse it if it's shorter, unless
     `--offset-s` is given.
   - Print the offset and the match's quality (the peak against the next-best peak more than
     one rest away). Refuse an unclear match and suggest `--offset-s`.
   - The match needs to be right to about 1 s, against 40 s segments. An error well under a
     segment doesn't change |G|, only the phase's straight-line slope (Change 9).

5. **`transfer_function.py`: the window.** Cut the run from the record and skip its start:

   ```python
   i0 = round((file_offset_s + float(row['start_s'])) / dt)
   eta = eta_all[i0:i0 + len(x)]
   s = s_all[i0:i0 + len(x)] if s_all is not None else None    # same samples as eta
   # skip the ramps, and the start until the slowest component has reached the probe
   skip = round((float(row['ramp_s']) + args.probe_distance_m / c_g(f_max, args.depth_m)) / dt)
   x, eta = x[skip:-skip], eta[skip:-skip]                     # and s
   ```

   Warn if the probe isn't still before the run (its std over the 10 s before the run above a
   few mm).

6. **`transfer_function.py`: the estimate.** Use the same window, segment length and overlap for
   all three spectra:

   ```python
   kw = dict(fs=1 / dt, window='hann', nperseg=round(args.segment_s / dt), detrend='constant')
   f, sxx = signal.welch(x, **kw)
   _, sxy = signal.csd(x, eta, **kw)
   _, coh = signal.coherence(x, eta, **kw)
   G = sxy / sxx
   n_avg = 2 * len(x) // kw['nperseg'] - 1                     # 50 % overlap
   eps = np.sqrt(1 - coh) / (np.sqrt(coh) * np.sqrt(2 * n_avg))  # Bendat & Piersol, ± on |G|
   ```

   `--segment-s 40` (default) gives df = 0.025 Hz and about 14 averages on a 300 s run. Longer
   segments give finer resolution but a larger eps. For a regular run, set the segment to a whole
   number of periods so its frequency falls on a bin.

   Put the three pairs through one function, `frf(u, y, kw) -> f, G_h1, G_h2, coh, eps`, so they
   share every setting.

7. **`transfer_function.py`: theory, the gain and smoothing.** This is the inverse of
   `paddle_amplitude` ([generate_waves.py:148](rig_controller/src/generate_waves.py#L148))
   without the gain:

   ```python
   wm = gw.load_wavemaker(args.wavemaker, args.config or gw.default_config_path())
   wm.water_depth = args.depth_m                 # the flap's hinge term uses it too
   kh = np.array([gw.wavenumber_depth(1 / fi, args.depth_m) for fi in f])
   g_lin_wl = np.array([gw.stroke_ratio(k, wm) for k in kh])     # a / x at the waterline
   g_lin = g_lin_wl.copy()                                        # a / x at the attachment
   if wm.type == 'flap':
       g_lin *= (args.depth_m - wm.hinge_height) / (wm.attachment_height - wm.hinge_height)
   gain = np.abs(G) / g_lin
   ok = (f >= float(row['f_min_hz'])) & (f <= float(row['f_max_hz'])) & (coh >= 0.8)
   # decision 13: average over the reflection's ripple
   n_smooth = max(1, round(args.smooth_hz / (f[1] - f[0])))
   gain_s = ndimage.uniform_filter1d(gain, n_smooth)   # only over ok bins: mask, then filter
   ```

   - The ± on the smoothed gain is eps / sqrt(n_smooth): the bins are about independent.
   - With `--smooth-hz 0` the raw gain is reported, to see the ripple.
   - Warn if `args.depth_m` differs from the yaml's, as `generate_waves` does for the gain table
     ([generate_waves.py:389](rig_controller/src/generate_waves.py#L389)).

8. **`transfer_function.py`: drive and water** (decision 15), with `--stroke`:

   ```python
   z, hinge = args.stroke_height_m, wm.hinge_height
   s_wl = s * (args.depth_m - hinge) / (z - hinge)                    # stroke at the waterline
   x_wl = x * (args.depth_m - hinge) / (wm.attachment_height - hinge)  # command at the waterline
   _, G_drive, _, coh_d, eps_d = frf(x_wl, s_wl, kw)       # 1.0 = the flap follows the command
   _, G_water, G_water_h2, coh_w, eps_w = frf(s_wl, eta, kw)
   gain_water = np.abs(G_water) / g_lin_wl                 # 1.0 = linear theory
   ```

   - `G_cmd` ≈ `G_drive` × `G_water` (at the waterline). Check it and print the largest
     difference in the band: a large one means the stroke and the probe don't belong together
     (wrong column, wrong scale).
   - `|G_drive|` below 1 and falling with frequency is the drive lagging (the oscillator showed
     97 % at 2 s and 90 % at 1 s unloaded, TODO 46). Its phase, after the straight line of the
     offset error is removed, shows the lag's shape but not the absolute delay (decision 12).
   - `gain_water` is the number to set against theory. If it is about 1 and `|G_drive|` is
     about the table's gain, the table is correcting the drive, not the water. Worth knowing,
     since the drive's response may change with load and the water's does not.

9. **`transfer_function.py`: the residual phase.** Subtract the travel phase predicted by
   theory:

   ```python
   k = kh / args.depth_m
   res = np.unwrap(np.angle(G[ok]) + k[ok] * args.probe_distance_m)
   delay_s = -np.polyfit(2 * np.pi * f[ok], res, 1)[0]
   ```

   - A straight line means a pure delay. Curvature means the drive is lagging.
   - For `G_cmd`, `delay_s` is the drive's delay plus the offset's error, so it's no measure of
     the drive. It checks the offset: well under 1 s without the stroke, and about the drive's
     lag (0.1–0.2 s) or less with it. The drive's absolute delay needs the bag (Change 11).
   - For `G_water` (with the stroke) the same residual is free of the offset: stroke and probe
     are sampled together. A flat line through 0 means the waves travel as linear theory says;
     a slope is an error in `--probe-distance-m` (or the depth); its size tells which.

10. **`transfer_function.py`: the output.** Print one row per table period inside the band, and
   write all trusted bins to `<output>.csv`, raw and smoothed:

   ```
   T [s]  gain   ±     coh   table (yaml)   drive  ±     water  ±     coh
   1.00   0.771  0.02  0.96  0.755          0.81   0.01  0.95   0.02  0.95
   1.50   ...
   offset 352.4 s (from the stroke, match 4.1× the next), delay 0.21 s, 14 averages,
   smoothed over 0.05 Hz, depth 0.98 m (yaml 1.0 m), stroke sensor at 1.20 m,
   G_cmd vs drive × water: largest difference 2 % in the band
   ```

   - `gain` is `G_cmd` (the table's number); `drive` and `water` are `|G_drive|` and
     `gain_water`, both 1.0 when ideal, at the waterline. The drive and water columns only with
     `--stroke`.
   - `<output>.txt`: the command line, the commit (`motion_files.source_commit`), the
     wavemaker, depth, record file and its columns, distance, stroke height, the offset and how
     it was found, and the smoothing, as `generate_waves` records what a file was made from.
   - `<output>.png` with `--plot`, against period: gain ± eps, raw and smoothed, with the table;
     drive and water (with `--stroke`); coherence with the 0.8 line; and the residual phases.

11. **Later, not in the first pass.**
    - With three probes, separate the incident and reflected waves (Mansard & Funke 1980) and
      use the incident wave as η, in place of the smoothing (decision 13).
    - The drive's absolute delay, which the record can't give (decision 12): `--bag <bag>` with
      `wavemaker_setpoint` and `wavemaker_position`, published in the same control cycle
      ([controller_node.cpp:1344-1350](rig_controller/src/controller_node.cpp#L1344-L1350)).
      Pair them by message order, not receive time (the messages are `Float64`, no stamp, and
      the bags arrive in bursts, COMMISSIONING.md, speed tests). Or a played trigger recorded in
      catman (TODO *Trigger output*).
    - Read catman `.bin` directly, which saves the export step.

### Tests and docs (Claude)

- `rig_controller/test/test_transfer_function.py`, registered like `test_generate_waves.py`.
  Synthetic data: make a `.dat` and run table with `generate_waves` (a regular run, a rest, an
  irregular run), then make the record CSV at 200 Hz, with 30 s of still water before the file:
  - a stroke column: x filtered by a known drive (gain 0.9 at the waterline, a first-order lag
    of 77 ms plus 100 ms delay, as the oscillator measured), scaled to a sensor at 1.20 m, in mm;
  - a probe column: that stroke filtered by a known water (0.83 × `g_lin_wl` and the travel
    phase to 3 m), plus noise at the probe's level.

  The tool must:
  - find the offset from the stroke to within the drive's lag, and without `--stroke` (probe
    envelope) to within 1 s;
  - recover drive 0.9 and water 0.83 within eps, and gain 0.75 (their product) for `G_cmd`;
  - give a flat residual phase for `G_water` and a straight one for `G_cmd`.

  If this fails, the error is in the code, not the tank.
- The stroke column with its sign flipped: refused, saying the stroke and the command point
  opposite ways.
- The same with one regular run: one row, at its period, gain 0.75.
- Reflection: add a copy of η, delayed by 19 s and scaled by 0.1. The smoothed gain stays within
  eps of 0.75; the raw gain shows the ripple at 1/19 Hz.
- Refusals: no `--depth-m`; no run table; a run table without `ramp_s`; a run number not in it;
  a record CSV with a non-uniform time column or a rate that isn't a whole multiple of the
  file's; a column name not in its header (the message lists those that are); `--stroke`
  without `--stroke-height-m`; a record shorter than the file without `--offset-s`; an unclear
  match.
- `test_generate_waves.py`: `ramp_s` in `test_regular_signal` (3 periods of 1.0 s → 3.0) and
  `test_irregular_signal` (5 × 1.2 s → 6.0, whole samples), and in the written run table's
  header. `test_generate_oscillations.py`: the same, and its header check against
  `OSCILLATION_COLUMNS`.
- `README.md`: `ramp_s` in the run-table columns, for waves and oscillations; the program in the
  `rig_controller` table and a short section after *Irregular waves*. `rpi/README.md`,
  *`generate_waves` on the rig*: one paragraph on running it after a session: record the whole
  file in catman with the probes and the flap's draw wire, with still water before the first
  run; note the draw wire's height above the floor; export as CSV.

### Check on the data

1. Generate `item38` again from `matrix.csv` (decision 11); the `.dat` is identical to the
   played one (compare them).
2. Run 3 of `item38.dat` (Hs 0.03 m, Tp 1.18 s, WP1 at 3 m, depth 0.98 m): the offset is found,
   and coherence ≥ 0.8 over most of 0.8–2.4 s.
3. Runs 0 and 2 through the same tool: at 1.0 s and 1.3 s they agree with run 3 within eps plus
   the table's ~6 %.
4. At 1.5 s, see which value run 3 supports, 0.75 or 0.78. Run 1 is steeper (1/40), so a
   difference beyond eps is the steepness effect, not a reason to discard the result.
5. `delay_s` is well under 1 s. More means the offset is wrong: check it against `play_dat`'s
   `run N` log time and give `--offset-s`.
6. With the stroke: `G_cmd` ≈ `G_drive` × `G_water` within a few %; `G_water`'s residual phase
   flat. At 1.5 s, `G_drive` at the waterline set against the "0.572 of the command" from
   `Calib20oct.BIN`: if they differ by about the ratio of the sensor's and the attachment's
   heights above the hinge, the 0.572 was mostly geometry.

## Limits of the first pass

- The drive and water split needs the draw-wire channel in the record. `item38.bin` should
  have it (2026-11-12); confirm its channel name and the sensor's height above the floor when
  exporting the CSV. Without either, `item38` gives only `G_cmd`.
- The drive's absolute delay isn't measured (decision 12); the drive's response is, apart from
  that straight line in its phase.
- Run 3 is a low wave (Hs 0.03 m), so it measures the linear gain only. Steepness and depth
  effects need regular runs and the GP step.
- All code above is sketch lines; none of it has been run.
