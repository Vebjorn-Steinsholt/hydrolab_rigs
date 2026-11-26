# Sketch: estimating the transfer function from every run (long term)

What to code so the wavemaker's transfer gain is estimated from all the runs it plays, not just
from occasional calibrations, and why. This is a long-term plan. Stage 0 works with what exists
today. Stage 1 needs a wave probe that publishes on a ROS topic:
[ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md).

**Status (2026-11-18): not started.** None of the changes below are in the code. The run's
`.txt` already records depth, kind and probe distance, as free text (change 1). Builds on
`transfer_function` (done; its sketch: `git show c6bba03:transfer_function_sketch.md`) and on
[TODO.md](TODO.md) item 43.

Line numbers are as of `master` on 2026-11-18 (`9ac7b46`).

## Problem

The gain table in `wavemakers.yaml` (`wavemaker_transfer_gain_periods/values`, lines 62-63)
gives one value per period, each from a single run:

- The 2026-11-13 values come from item38 run 3 alone, with ±0.05-0.10 at 1.4, 1.5 and 1.8 s.
  The one exception is 0.8 s. It was set to 0.99 by hand on 2026-11-17, from the regular checks,
  because item38 run 3 gave 1.08.
- Before the 2026-11-13 recalibration, two runs at 1.5 s disagreed (0.75 against 0.78), and
  there was no way to combine them.
- Each new measurement replaces the old value, by hand, instead of adding to it.
- Nothing notices when the tank drifts away from the table. The WP1 channel scaled 0.03981
  instead of 0.0229 m/V was only found by checking by hand (COMMISSIONING.md line 327).

Every run played with a probe in the tank is a measurement of the same function. Pooling them
gives one smooth curve with an error bar at each period, which tightens with every run and flags
a run that disagrees with it.

**What changes for the user.**

Stage 0, offline, from today's `transfer_function` outputs:

```bash
ros2 run rig_controller gain_model ladertanken --add item38_run3_tf.csv --depth-m 0.98
ros2 run rig_controller gain_model ladertanken --table
```

```text
T [s]  model  ±      runs  bins  yaml   model-yaml
1.00   1.112  0.008  4     31    1.11   +0.2 %
1.50   1.162  0.021  3     18    1.15   +1.0 %
...
item 2026-11-20_r2 (1.5 s): 1.31, 5.8 σ from the model: NOT added (--accept to add it)
```

Stages 1 and 2 do the same live, per goal, from the probe topic. They add to the same model and
print the same table.

### Decisions

1. **Model the gain over theory, as a smooth function of frequency.** The modelled quantity is
   g(f) = |G_cmd(f)| / Biésel(f, depth). That is `transfer_function`'s `gain` column (its
   decision 5), and the yaml's convention. Dividing by theory at each run's measured depth lets
   runs at 0.98 m and at 1.0 m feed the same curve. g is written as B-splines with unknown
   weights:

   ```text
   g(f) = Σ θ_j · B_j(f)     cubic B-splines, knots every 0.1 Hz over 0.4–1.8 Hz (~12 weights)
   ```

   A single table point at a period gets help from its neighbours, and the curve has no jumps
   over 0.05 s like the ones that made item 33 suspicious.
2. **Store the measurements and refit, not just the recursive state** (recommended). Each run
   adds rows (f, g, σ, run id, date, depth, geometry) to a file. The model is a weighted
   least-squares refit over all rows, which is milliseconds for a few thousand rows. It gives the
   same θ as recursive least squares with the same weights. It can be audited: you can see which
   runs made a value. One bad run is dropped by removing its rows. RLS is kept only for the live
   fit inside a regular goal (stage 1).
3. **Forgetting by age, reset by geometry.**
   - A row's weight is `λ^(runs since)`, the RLS forgetting factor. Start with λ = 0.97, a
     memory of about 30 runs, so slow drift is followed.
   - Rows are tagged with a hash of the geometry from the yaml (`actuator_lead_m_per_degree`,
     `flap_attachment_height`, `hinge_height`, type). Only rows with the current hash are
     pooled. A recalibration like 2026-11-13, which changed every gain by 1.38, starts a fresh
     model by itself.
   - The hash is of the geometry `transfer_function` used to analyse the run, not the geometry
     at recording. A run recorded before a recalibration still counts once it is analysed again
     with the new geometry, as item38 run 3 was on 2026-11-13.
4. **The yaml table is the prior.** It enters as one pseudo-measurement per table period, with
   σ 5 %. With no data the model returns the yaml. Where there is little data (0.6 s, 2.0 s) it
   stays near the yaml. Where many runs agree, the data wins. A second-difference penalty on θ
   (a P-spline) keeps the curve smooth between knots.
5. **Use the raw bins, with honest errors.**
   - From irregular runs, use `gain_raw` with σ = g · ε (Bendat & Piersol, `frf` at
     `transfer_function.py:399`), not the smoothed `gain`. Smoothed bins are correlated, so their
     errors would be counted several times.
   - Hann-windowed Welch bins also leak into their neighbours. Take every second bin, or inflate
     σ by √1.5.
   - Regular runs give one row at their period, from the same columns.
6. **A floor on each run's error.** Probe scaling, depth and position errors are common to a
   whole run and don't average down with more bins. Each run gets an extra σ of 1 %, shared by
   all its rows. Without this, ten irregular runs would claim ±0.1 % while being 5 % off from one
   probe scaling. Implement it by giving each run one extra parameter for its common offset, in
   log g, with a 1 % prior.
7. **New runs that disagree are flagged, not silently pooled.** A run whose mean residual
   against the model is above 3σ (combined) is not added unless `--accept` is passed. It prints
   which bins disagree. That catches channel scaling, a wrong depth or a moved probe on the first
   run.
8. **The yaml is never written automatically.** `--table` prints model values at the yaml's
   periods, with ±, the number of runs and the difference to the yaml. A person copies them, as
   with `transfer_function` (its decision 9).
9. **Drive and water kept apart where the data has both.** `transfer_function` already gives
   `drive` (command → draw wire) and `water` (draw wire → wave) when `--stroke` is used. Each
   gets its own model in the same file, and the command → wave model is fitted to `gain`. If the
   gain moves and `drive` moves with it, the drive changed. If only `water` moves, look at the
   probe, the depth or reflections.

## Changes

1. **`rig_controller/src/transfer_function.py`.** In `gain_cols` (line 659), also write the
   raw error, and the run's facts that the model needs into the CSV, or into the `.txt` that
   goes with it. Take the error from `frf_.eps`, the Bendat & Piersol error, and not from the
   timeline's `eps` (clock drift), which shares the name:

   ```python
   return {f'{name}_raw': g[ok], name: g_s[ok],
           f'{name}_raw_err': (g * frf_.eps)[ok],
           f'{name}_err': (g_s * smooth(frf_.eps, ok, n_smooth) / np.sqrt(n_smooth))[ok]}
   ```

   The `.txt` from `write_record` (line 495) already has the depth, kind and probe distance, as
   free-text lines (`water depth`, `motion file`, `record`, lines 688-692). Add the geometry
   hash. Then either give these facts fixed `key value` lines that `gain_model` can parse, or
   write them as columns in the CSV. `gain_model` reads them from there, so `--depth-m` above is
   only a fallback.

2. **`rig_controller/src/gain_model.py` (new).** The model file is
   `~/wave_sessions/gain_model_<wavemaker>.csv`, one row per measurement:

   ```text
   run_id, date, geometry, depth_m, kind, quantity, f_hz, g, sigma
   2026-11-05_item38_r3, 2026-11-05, 3f9a1c, 0.98, irregular, gain, 0.833, 1.137, 0.021
   ```

   The fit:

   ```python
   def fit(rows, prior, knots, lam, n_runs):
       B = bspline_design(rows.f_hz, knots)                 # n x m
       R = run_indicator(rows.run_id)                       # n x r, one offset per run (decision 6)
       A = np.hstack([B, R * rows.g[:, None]])
       w = lam ** (n_runs - 1 - rows.run_index) / rows.sigma**2
       # prior: yaml table at 5 %, run offsets at 1 %, second differences of theta
       A, y, w = stack(A, rows.g, w, prior_rows(prior, knots), offset_rows(0.01), penalty_rows(knots))
       Aw = A * w[:, None]
       cov = np.linalg.inv(A.T @ Aw)
       theta = cov @ (Aw.T @ y)
       return theta[:B.shape[1]], cov[:B.shape[1], :B.shape[1]]
   ```

   The value and error at a period:

   ```python
   b = bspline_design([1 / T], knots)[0]
   g, sd = b @ theta, np.sqrt(b @ cov @ b)
   ```

   The 3σ check (decision 7) compares the new run's rows against a fit without them. Use
   `scipy.interpolate.BSpline.design_matrix` for `bspline_design`. scipy is already used by
   `transfer_function`.

   Options: `--add <tf.csv>...`, `--accept`, `--remove <run_id>`, `--table`, `--plot` (model
   ± band, each run's points, the yaml points), `--lambda`, `--quantity gain|drive|water`.

3. **Stage 1: `rig_controller/src/probe_gain.py` (new; needs a probe on a topic).** A node that,
   for each goal, collects the commanded setpoint and the probe's η, and produces the same rows
   as `transfer_function`. It adds them with `gain_model`'s functions, flags included.
   - **The command** comes from `<rig>_setpoint`, published at `controller_node.cpp:1344`. It is
     `std_msgs/Float64` with no stamp, so add a stamped `rig_interfaces/msg/Setpoint.msg`
     (header, setpoint, position) published next to it in the same loop.
   - **The goal** comes from a new latched topic `<rig>_goal`, published by the controller,
     since it is the only node that sees every goal, whoever sent it. Today no topic carries
     the goal: the web page's `period` and `height` are unstamped `Float64`s and reach only the
     bridge, and `play_dat` sends only `positions` and `sample_interval`
     (`play_dat.py:338`), so a file's periods exist only in its run table.
     - **`rig_interfaces/msg/Goal.msg` (new):** `std_msgs/Header header` (stamped when
       `execute_goal` starts, `controller_node.cpp:1235`), `string kind` (`regular` or
       `positions`), `float64 height`, `float64 period`, `float64 sample_interval`,
       `uint32 n_positions`, `string source`.
     - **`MoveRig.action` gains `string source`** in the goal, left empty by old clients.
       `play_dat` fills it with the file and run (`check3.dat run 2`), and the bridge with
       `web`. The controller copies it into `Goal.msg` unchanged.
     - **Published once per goal,** next to the setpoint publishers
       (`controller_node.cpp:596`), with QoS reliable, transient local and depth 1, so a node
       started mid-goal still gets the goal that is running.
     - **`probe_gain` takes ω** from `period` for a regular goal. For a `positions` goal it
       reads the run table row named by `source`: `period_s` for a regular run inside a
       file, `peak_period_s` and the band for an irregular one. A `positions` goal with an
       empty `source` is logged and skipped.
     - Considered and not chosen: reading the run table for file goals and the bridge's
       `<rig>_state` text for web goals. That needs no controller change, but it is two
       sources, and parsing a text string breaks when its wording changes.
   - **Regular goals:** the three-parameter sine fit at ω gives H. Use RLS, so a live value can
     be logged during the run. g = gain_used × H_meas / H_target.
     - **Linear, at the commanded ω:** η ≈ a sin ωt + b cos ωt + c, solved by least squares
       (`np.linalg.lstsq`, or RLS live); H = 2√(a² + b²). Not `scipy.optimize.curve_fit` with ω
       free: that is non-linear, needs a start guess and can stop at a wrong minimum, and ω is
       known anyway.
     - **Dropouts are left out, not filled:** the fit takes only the finite samples at their
       stamps, so no grid and no interpolation are needed. The FFT path can't do that.
     - **Add 2ω and 3ω terms** (sin and cos each). H stays the ω term. The ratio
       √(a₂² + b₂²) / √(a² + b²) is logged per goal as the run's nonlinearity, the amplitude
       dependence that [drive_correction_sketch.md](drive_correction_sketch.md) names under
       *Problem*.
     - **The fit rms** in *Rejects* is the rms of the residual after all terms, over the
       ω-term amplitude H/2. Above 5 %, the wave isn't a clean regular wave at the probe
       (reflections, a cross mode, or the probe losing echoes).
   - **File goals (irregular):** at the end of the clean window, the node calls
     `transfer_function.frf` on the setpoint and η, resampled onto one time base.

   **The probe** is the HC-SR04 on the RPi4 ([ultrasonic_probe_sketch.md](ultrasonic_probe_sketch.md)).
   Before each goal the node calls `/wavemakers/<pi>/zero_measurement` with `duration` =
   `STILL_S`, then reads η from `/wavemakers/<pi>/surface_elevation` (the probe sketch's decision
   7; [wave_probe_node_sketch.md](wave_probe_node_sketch.md), decision 9). A refused zero
   (water not still, or too many dropouts) rejects the goal, as `check_still` does.

   **Clocks.** The probe's stamps come from the RPi4, kept to the RPi5 by chrony (the probe
   sketch's decision 6), and need not agree exactly. A constant offset only rotates
   the phase of S_uy, so |H1| and the gain are unchanged, as long as the offset is small against
   a segment (`segment_s`). The node still logs the offset it finds by cross-correlation, as
   `offset_from_envelopes` does, and refuses a goal when the offset jumps between goals.

   **The window** is the same as `transfer_function` uses: after the ramp and the front's travel
   `x / c_g`, and before the beach reflection `(2 L − x) / c_g` (COMMISSIONING.md line 447). That
   needs `beach_distance_m` as a parameter. Reuse `group_velocity` (`transfer_function.py:130`),
   moved to `motion_files.py` so a node doesn't import an analysis script.

   **Rejects** follow `transfer_function`: `check_still` before the goal, coherence below
   `COHERENCE_MIN`, more than 10 % probe dropouts (`nan` on `surface_elevation`, or gaps over 3
   samples), and a fit rms above 5 % for regular goals.

4. **Stage 2: drive tracking live (no probe needed).** The same node fits setpoint → encoder
   position from the stamped `Setpoint.msg`. Both come from the same loop and the same clock, so
   phase is valid too. This is `G_drive` as the servo sees it.

   **Pitfall:** the encoder doesn't see the geometry. On 2026-11-13 the flap turned only 0.78
   of what the old lead predicted, and the encoder could not have shown that. Only the draw
   wire (`WM_Position`) or the inclinometer can. So this model is for servo tracking (TODO 46 on
   the oscillator, the paddle on the wavemakers), not a replacement for decision 9's `drive`.
   Publishing the draw wire on a topic would close that gap.

5. **`wavemaker_bringup/launch/wavemakers.launch.py`.** Add a `probe_gain:=off|on` argument
   (default `off`) and `PROBE_GAIN=` in `/etc/default/wavemaker`, the same way `WATER_DEPTH` is
   handled. The node is a pure listener: it sends nothing to the controller.

## Tests and docs

- `test_gain_model.py`:
  - Synthetic runs drawn from a known g(f) with their σ: the model recovers g within its ±
    over 0.8–1.6 s, and the ± is honest (about 68 % of true values inside 1σ over many seeds).
  - With no rows, the model equals the yaml table at its periods.
  - A run with its probe scaled 5 % off is flagged (> 3σ) after 3 good runs.
  - The run-offset term keeps ten copies of one biased run from claiming better than 1 %.
  - A row with another geometry hash is ignored.
  - `--remove` gives the same model as never adding the run.
- `test_probe_gain.py`:
  - A synthetic sine with noise and dropouts gives the same g as `transfer_function` on the
    same data.
  - The RLS with λ = 1 equals the batch fit.
  - A constant clock offset of 0.3 s leaves the irregular gain unchanged to 0.1 %.
- README: a section *The gain model*. In [TODO.md](TODO.md), point at this sketch from
  *Planned features* (done 2026-11-14).

## Check on the data / rig

1. **Stage 0 on what exists.** Add item38 run 3 (2026-11-05), then the 2026-11-17 check
   (`check2`), then 2026-11-18 (`check3`, TODO 43), all at 1.0 m. Analyse each one with the
   timeline from `88363ba`. Leave out 2026-11-13: it needed `--offset-s` and a hand-corrected WP1.
   Expected:
   - The model agrees with the table to its scatter at 0.9–1.3 s and 1.6 s.
   - It narrows the ±0.05–0.10 at 1.4, 1.5 and 1.8 s.
   - At 0.8 s, item38 run 3 (1.08) and the checks (about 0.99-1.02) disagree. Decide by hand
     which goes in the table. Expect the 3σ flag from decision 7 when the regular runs are
     added after item38 run 3.

   Item38 run 3 must carry the new geometry's hash, because it was analysed with it. Analyse it
   once more with the old geometry and check that those rows are ignored.
2. **Repeatability.** With the sessions from check 1, add the runs one at a time. Each table
   value's ± should shrink about as 1/√runs until it reaches the 1 % floor, and not below.
3. **Stage 1 against catman.** Record the probe topic and WP1 side by side. Pass if
   `probe_gain`'s rows agree with `transfer_function`'s on the same run within their ±.
4. **A deliberate fault.** Scale the probe's channel 5 % off for one run. `gain_model` must flag
   it and leave the model unchanged.

## Open

- Whether the HC-SR04 passes the probe sketch's check 3 against WP1. If not, its fallback is
  the probe amplifier's ±10 V output through an ADC.
- `beach_distance_m`: from the drawings, or from the 21 s arrival at 1.5 s.
- Knot spacing and λ: start at 0.1 Hz and 0.97, and revisit after stage 0 on real data.
- Whether the yaml table should later be generated from the model, as the model's values at the
  table periods, with a person still committing it.

## Later, not in the first pass

- **Phase and delay.** Model the residual phase (`phase_res_rad`) the same way. Needs a probe
  and the Pi on a common clock (PTP or chrony) for the live stage.
- **A time-domain model.** An FIR or ARX model from setpoint to η, fitted with RLS over lagged
  inputs, gives the complex G and the nonlinear terms (2ω) in one model. Worth it only if the
  frequency-domain model leaves systematic residuals.
- **Amplitude dependence.** Add steepness H/L as a second variable, g(f, H/L), once there are
  runs across steepness at the same period. No session has planned these yet. TODO 43's
  2026-11-18 check runs everything at 1/60. A series at one period (for example 1.5 s at 1/60,
  1/40 and 1/28) would be the first such data.
- **Incident and reflected waves** separated with 2–3 probes (Goda–Suzuki or Mansard–Funke).
  This lifts the reflection window and gives longer, cleaner rows.
- **Using the model to correct goals,** with a bounded factor between goals. The first version
  of this sketch had it. Only after the model has been trusted for a while.
