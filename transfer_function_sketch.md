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
`play_dat`, run on the motion file and a probe record. Not a change on the rig:

```bash
ros2 run rig_controller transfer_function item38.dat ladertanken --run 3 \
    --probe item38_WP1.csv --probe-distance-m 3.0 --depth-m 0.98 --offset-s 352.4 \
    -o item38_run3_tf --plot
```

It prints the gain per period next to the yaml's table, and writes `item38_run3_tf.csv`
(the rows), `.txt` (what it was made from) and `.png`.

### Decisions

1. **Self-contained in this repo.** Everything it needs is on the command line or in files this
   repo already writes: the `.dat` and its run table from `generate_waves`, the wavemaker from
   `wavemakers.yaml`, and one probe record. No session files and no other analysis code.
2. **The probe as a CSV export** (recommended): two columns, time in s and elevation, from
   catman's ASCII export. One small, testable reader with no dependency on the catman format.
   Alternative, later: read catman's `.bin` directly, which saves the export step but means
   reimplementing a binary format.
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

### Changes

1. **`rig_controller/CMakeLists.txt`.** Install the program as the others are, and install
   `generate_waves.py` as a module too, since the new program imports it (today only
   `play_dat`, `rig_limits` and `motion_files` are, line 84):

   ```cmake
   # Transfer function and gain from a run and a probe record (see the script's docstring).
   install(PROGRAMS src/transfer_function.py
     DESTINATION lib/${PROJECT_NAME}
     RENAME transfer_function
   )

   install(FILES src/play_dat.py src/rig_limits.py src/motion_files.py src/generate_waves.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

2. **`rig_controller/src/transfer_function.py`: inputs.** The command with `play_dat`'s readers
   (`read_motion`, `select_channel`, `read_run_table`); don't write a second parser. Cut the run
   with its `start_s`/`end_s`:

   ```python
   motion = play_dat.read_motion(args.file)
   x = play_dat.select_channel(motion, 'position')         # m at the attachment, from upright
   dt = motion.sample_interval_s
   row = next(r for r in play_dat.read_run_table(args.file) if int(r['run']) == args.run)
   x = x[round(float(row['start_s']) / dt):round(float(row['end_s']) / dt)]
   ```

   The probe CSV: check that the time column is uniform and give its rate. Decimate to the
   file's rate (200 Hz → 50 Hz) with an anti-alias filter, not by taking every 4th sample:

   ```python
   t, eta = read_probe_csv(args.probe)                     # s, m (--probe-scale for mm)
   q = round(dt / (t[1] - t[0]))                           # 4 for 200 Hz against 0.02 s
   eta = signal.decimate(eta, q, ftype='fir', zero_phase=True)
   ```

3. **`transfer_function.py`: the window.** `--offset-s` is where the run starts in the probe
   record: the time of `play_dat`'s `run N (...)` log line minus the start of the recording.

   ```python
   i0 = round(args.offset_s / dt)
   eta = eta[i0:i0 + len(x)]
   # skip the ramps, and the start until the slowest component has reached the probe
   skip = round((n_ramp_s + args.probe_distance_m / c_g(f_max, args.depth_m)) / dt)
   x, eta = x[skip:-skip], eta[skip:-skip]
   ```

   The offset only needs to be right to about 1 s, against 40 s segments. Don't refine it by
   cross-correlation: the travel time to the probe is real and belongs in G's phase. The ramp
   length isn't in the run table: take it from the file's ramps (where |x| reaches its level),
   or add `n_ramp` to `WAVE_COLUMNS` in `generate_waves`. Warn if the probe isn't still before
   the run (its std over the 10 s before `offset_s` above a few mm).

4. **`transfer_function.py`: the estimate.** Use the same window, segment length and overlap for
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

5. **`transfer_function.py`: theory and the gain.** This is the inverse of `paddle_amplitude`
   ([generate_waves.py:148](rig_controller/src/generate_waves.py#L148)) without the gain:

   ```python
   wm = gw.load_wavemaker(args.wavemaker, args.config or gw.default_config_path())
   wm.water_depth = args.depth_m                 # the flap's hinge term uses it too
   kh = np.array([gw.wavenumber_depth(1 / fi, args.depth_m) for fi in f])
   g_lin = np.array([gw.stroke_ratio(k, wm) for k in kh])        # a / x at the waterline
   if wm.type == 'flap':
       g_lin *= (args.depth_m - wm.hinge_height) / (wm.attachment_height - wm.hinge_height)
   gain = np.abs(G) / g_lin
   ok = (f >= float(row['f_min_hz'])) & (f <= float(row['f_max_hz'])) & (coh >= 0.8)
   ```

   Warn if `args.depth_m` differs from the yaml's, as `generate_waves` does for the gain table
   ([generate_waves.py:389](rig_controller/src/generate_waves.py#L389)).

6. **`transfer_function.py`: the residual phase.** Subtract the travel phase predicted by
   theory:

   ```python
   k = kh / args.depth_m
   res = np.unwrap(np.angle(G[ok]) + k[ok] * args.probe_distance_m)
   delay_s = -np.polyfit(2 * np.pi * f[ok], res, 1)[0]
   ```

   A straight line means a pure delay: the drive's delay plus any error in `--offset-s`.
   Curvature means the drive is lagging.

7. **`transfer_function.py`: the output.** Print one row per table period inside the band, and
   write all trusted bins to `<output>.csv`:

   ```
   T [s]  gain   ±     coh   table (yaml)
   1.00   0.771  0.02  0.96  0.755
   1.50   ...
   delay 0.21 s, 14 averages, depth 0.98 m (yaml 1.0 m)
   ```

   `<output>.txt`: the command line, the commit (`motion_files.source_commit`), the wavemaker,
   depth, probe file, distance and offset, as `generate_waves` records what a file was made from.
   `<output>.png` with `--plot`: three panels against period: gain ± eps with the table; coherence
   with the 0.8 line; and the residual phase.

8. **Later, not in the first pass.**
   - With three probes, separate the incident and reflected waves (Mansard & Funke 1980) and use
     the incident wave as η. This removes the beach ripple in |G|, which repeats every
     `c_g / (2 L_beach)` in frequency.
   - `--paddle <bag>` for `G_drive` (command → paddle) and `G_water` (paddle → probe)
     separately. Use the message stamps, not receive times: the bags arrive in bursts with about
     20 ms jitter (COMMISSIONING.md, speed tests), and that adds phase noise that grows with
     frequency.
   - Read catman `.bin` directly (decision 2).

### Tests and docs (Claude)

- `rig_controller/test/test_transfer_function.py`, registered like `test_generate_waves.py`.
  Synthetic data: make a `.dat` and run table with `generate_waves` (one irregular run), then make
  the probe CSV by filtering x with a known G (gain 0.75 × `g_lin`, a 2 s delay, noise at the
  probe's level), at 200 Hz with an offset. The tool must recover 0.75 within eps across the band,
  and `delay_s` must come out at 2 s ± one sample. If this fails, the error is in the code, not
  the tank.
- The same with one regular run: one row, at its period, gain 0.75.
- Refusals: no `--depth-m`; no run table; a run number not in it; a probe CSV with a non-uniform
  time column or a rate that isn't a whole multiple of the file's.
- `README.md`: the program in the `rig_controller` table and a short section after *Irregular
  waves*. `rpi/README.md`, *`generate_waves` on the rig*: one paragraph on running it after a
  session, and that `--offset-s` comes from `play_dat`'s `run N` log line.

### Check on the data

1. Run 3 of `item38.dat` (Hs 0.03 m, Tp 1.18 s, WP1 at 3 m, depth 0.98 m): coherence ≥ 0.8
   over most of 0.8–2.4 s.
2. Runs 0 and 2 through the same tool: at 1.0 s and 1.3 s they agree with run 3 within eps plus
   the table's ~6 %.
3. At 1.5 s, see which value run 3 supports, 0.75 or 0.78. Run 1 is steeper (1/40), so a
   difference beyond eps is the steepness effect, not a reason to discard the result.
4. `delay_s` is in the hundreds of ms (drive plus offset error), not seconds. Seconds means
   `--offset-s` is wrong.

## Limits of the first pass

- `item38` has no paddle data, so this gives only the command-to-probe gain, not the drive and
  water split.
- Run 3 is a low wave (Hs 0.03 m), so it measures the linear gain only. Steepness and depth
  effects need regular runs and the GP step.
- All code above is sketch lines; none of it has been run.
