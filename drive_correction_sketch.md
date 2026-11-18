# Sketch: correcting a run's drive signal from its recording (iterative)

What to code so that one irregular run can be played, measured at the probe and played again
with a corrected drive signal, until the wave at the probe matches its target. Why it is done
this way and not with machine learning: linear physics plus 2–4 iterations gets within a few
percent with no training data, and stays auditable.

**Status (2026-11-18): not started.** Builds on `transfer_function` (its steps 1–5 put the probe
on the file's time base) and on `generate_waves` (each irregular run can be rebuilt exactly from
its run-table row). Independent of [gain_estimation_sketch.md](gain_estimation_sketch.md), but
uses its model as Ĥ once that exists (decision 3).

Line numbers are as of `master` on 2026-11-18 (`bb73ba1`).

## Problem

The gain table fixes the mean gain per period. What is left in an irregular run is not fixed by
any table:

- **Shape across the band.** The table is linear between points 0.1 s or more apart, and the
  measured `gain_raw` scatters ±0.05–0.10 at 1.4, 1.5 and 1.8 s. The spectrum at the probe is
  off by that much in narrow bands.
- **Phase.** The table has no phase. The residual phase (`phase_res_rad`) is measured but not
  used, so a time series at the probe drifts from its target. This matters most for a focused
  wave or for reproducing a given record.
- **Nonlinearity.** At high steepness the gain depends on amplitude, so a linear table can't be
  right for every sea state at the same time.
- **The tank.** Reflections and the probe's own position are in the measured wave and in no
  model.

An experiment that needs "this spectrum at the model" or "this time series at the probe" can only
get it by measuring and correcting that run.

**What changes for the user.**

```bash
# iteration 0: as today
ros2 run rig_controller generate_waves ladertanken matrix.csv -o sea
ros2 run rig_controller play_dat sea.dat ladertanken          # catman records WP1

# correct run 0 from its record; writes sea_it1.dat, .csv, .npz, .txt, .png
ros2 run rig_controller correct_drive sea.dat ladertanken --run 0 \
    --record sea.ASC --probe WP1 --probe-distance-m 3.0 --depth-m 1.0 -o sea_it1
ros2 run rig_controller play_dat sea_it1.dat ladertanken      # record again

# the next iteration starts from sea_it1
ros2 run rig_controller correct_drive sea_it1.dat ladertanken --run 0 \
    --record sea_it1.ASC --probe WP1 --probe-distance-m 3.0 --depth-m 1.0 -o sea_it2
```

```text
iteration 1 of run 0 (irregular, Hs 0.030 m, Tp 1.20 s, seed 1)
  Hs at probe     0.0291 m   target 0.0300   -3.0 %
  band error      rms 6.1 %  (0.05 Hz bands, 0.42-1.25 Hz)   worst +11 % at 1.15 Hz
  time series     rms 14 %   of the target's std, after a 0.034 s delay is removed
  correction      largest |dX/X| 18 % (cap 30 %), beta 0.7
  limits          stroke peak 0.112 m of 0.150, velocity 0.31 m/s of 0.50: OK
  repeatability   not known: play iteration 0 twice once (Check 2)
```

### Decisions

1. **The target is the wave that `generate_waves` meant, unless `--target` is given.**
   - **Default: no conventions to agree on.** The target spectrum at the probe, with the travel
     undone, is `T_n = Ĥ(f_n) · X0_n`. X0 is iteration 0's command, rebuilt from the run-table
     row (`hs_m`, `peak_period_s`, `gamma`, `seed`, `ramp_s`, `f_min_hz`, `f_max_hz`) with
     `irregular_signal` (`generate_waves.py:278`). Ĥ is the yaml's model (decision 3).
     This is the same as `a_n` with phase `φ_n + π/2` (theory: the wave leads the stroke by
     90°, `residual_delay`, `transfer_function.py:456`). Writing it through Ĥ keeps the sign
     and the flap's attachment factor in one place.
   - **`--target <csv>`: a given record.** Columns `t_s, eta_m`, at the probe, for example a
     run from an earlier session to reproduce. It is resampled onto the run's grid and
     travel-undone like the measurement. Only bins inside the band are matched (decision 5).
2. **Correct the complex spectrum on the synthesis grid. Don't re-synthesise from new random
   phases.** The run is `synthesize(n, P, …)` on bins `f_n = n / duration`
   (`motion_files.py:98`, `:144`), then `apply_ramps` (`:71`). The measured wave, read on the
   file's time base over exactly the run's samples (`on_file_time`, `transfer_function.py:320`)
   and with the travel undone (`undo_travel`, `:445`), has its FFT on the same bins. The update
   is per bin:

   ```text
   E_n     = T_n − Y_n · e^{iωτ}          error, with the run's best-fit delay τ removed (decision 4)
   P_{i+1} = P_i + β · E_n / Ĥ(f_n)       β = 0.7; |P_{i+1} − P_i| ≤ 0.3 |P_i| per bin
   ```

   This is iterative learning control with the model's inverse. It converges in every bin where
   `|1 − β H/Ĥ| < 1`. That means Ĥ only needs to be within a factor of about 2 of the real H,
   and the table is within 10 %. With Ĥ within 20 % and β = 0.7, each iteration leaves at most
   44 % of the error. The ramps are applied to the target too, so ramp leakage appears in both
   and cancels to first order.
3. **Ĥ is the yaml's model: smooth, with no per-run noise.** That is
   `Ĥ(f) = transfer_gain(T) · g_lin(f) · e^{iπ/2}`, with `g_lin` from `linear_ratio`
   (`transfer_function.py:137`), at the measured depth. Dividing by the measured `Y/X` per bin
   would be faster, but one realisation's bin ratio is noisy where `|Y|` is small, and it chases
   noise. When the gain model exists, Ĥ is its curve, and later its phase.
4. **Remove a pure delay before correcting phase.** The timeline anchors on the stroke or the
   probe (`analyse`, step 4), so a constant lag is absorbed into `a_k`. `play_dat` also starts
   each goal ±20 ms late (TODO 43). Correcting that would chase timing noise between plays. τ is
   fitted by maximising `|Σ T̄_n Y_n e^{iω_n τ}|`, then reported. Absolute timing to the trigger
   is "Open" below.
5. **Correct only the first-order band, and only where there is energy.** The bins are
   `band_mask`'s (`transfer_function.py:415`): `f_min_hz` up to `F_TOP_FP · fp`. On top of that,
   only bins where `|T_n|² ≥ 1 %` of the peak bin's. Bound harmonics near 2 fp are a response to
   the band, not to their own bins, and dividing them by Ĥ would make them worse. Bins outside
   keep `P_i` unchanged.
6. **Each iteration is a file: `.dat`, run table, `.npz`, `.txt`.**
   - The `.npz` holds `P_i` (complex, per bin) and `n` for every corrected run, so the next
     iteration doesn't have to rebuild it from a ramped, float32 file.
   - The run table keeps every column and adds `iteration` and `corrected_from` (the previous
     file and its commit).
   - Runs that aren't corrected are copied unchanged.
   - The `.txt` is `write_record`'s, with the lines printed above.
   - A file never overwrites its source.
7. **The limits are checked like any generated file.** The corrected run goes through
   `check_run` and `motion_files.assemble` (`motion_files.py:167`, `:202`). The output is
   refused, not clipped, if the stroke or the velocity is over the yaml's limits. The error
   names the bins that grew most. Clipping would put energy back into every bin.
8. **Stop by criteria, at most 4 iterations by default.** Stop when the band error is down to
   the repeatability (Check 2), or when it rises from the last iteration. A rise means the tank,
   not the drive, sets what's left, or Ĥ is too far off: lower β. `--max-iteration` only warns;
   it doesn't refuse.
9. **Regular runs are not iterated.** A regular run has one bin, and its correction is the gain
   table's job. `correct_drive --run <regular>` prints `H_target / H_meas` and the table entry
   it implies, as `transfer_function` does, and writes nothing.

## Changes

1. **`rig_controller/src/transfer_function.py`.** Split `analyse` (line 554):
   - Steps 1–5 go into a function `measured_run(args, config) -> MeasuredRun`. It returns
     `wm`, `row`, `table`, `dt`, the run's command `x` (full run, ramps included), the probe `η`
     on file time with the travel undone over the same samples, `a_k`, `eps` and the info lines.
   - `analyse` calls it, then works as now.
   - Keep the window cut (`cut_window`) in `analyse`. `correct_drive` wants the full run, so
     `measured_run` returns it uncut, plus `skip`.

2. **`rig_controller/src/generate_waves.py`.** Split `irregular_signal` (line 278) so the
   complex spectrum is available:

   ```python
   def irregular_spectrum(wm, spec, dt):
       """(n, f, P, n_samples, n_ramp_samples): P_n = p_n e^{i phi_n}, before synthesis."""

   def irregular_signal(wm, spec, dt):
       n, f, P, n_samples, n_ramp = irregular_spectrum(wm, spec, dt)
       x = apply_ramps(synthesize(n, np.abs(P), np.angle(P), n_samples), n_ramp)
       ...  # row as now
   ```

   Also add `spec_from_row(row) -> IrregularSpec`, the inverse of the run-table fields, so
   iteration 0 is rebuilt from the `.csv` alone. `generate_waves` output must not change: the
   existing tests guard that.

3. **`rig_controller/src/correct_drive.py` (new).**
   - The same CLI as `transfer_function`, plus `--target`, `--beta` (0.7), `--cap` (0.3),
     `--max-iteration` (4) and `--runs` (more than one run from the same record).
   - The core is pure functions, so the tests need no files:

   ```python
   def model_h(wm, f, depth_m) -> np.ndarray:                  # decision 3
   def target_spectrum(wm, P0, n, f, depth_m) -> np.ndarray:  # decision 1, default
   def fit_delay(T, Y, f) -> float:                           # decision 4
   def correct(P, T, Y, H, ok, beta, cap) -> np.ndarray:      # decision 2, returns P_{i+1}
   def metrics(T, Y, f, dt, n_samples) -> dict:               # Hs, band error, rms after τ
   ```

   - Main: `measured_run` → load `P_i` from `<file>.npz`, or rebuild it from the row
     (iteration 0) → FFT of η over the run's samples →
     `correct` → re-synthesise, `apply_ramps` → `assemble` all runs → write.
   - Before using a rebuilt `P_0`, check that it reproduces the run in the `.dat` within
     float32 rounding. If not, refuse: the yaml or the code changed since the file was made.
   - The plot: target and measured spectra (0.05 Hz bands) per iteration, the time series over
     60 s at the peak of the run, and `|P_{i+1}/P_i|` per bin.
   - Register it in `CMakeLists.txt` with the other Python tools.

## Tests and docs

- `test_correct_drive.py`, all synthetic, with a fake tank: `Y = H_true · X + noise`,
  `H_true = Ĥ · (1 + 0.15 sin(...)) · e^{i(small phase ripple)}`, plus a delay:
  - The band error falls below 1 % within 4 iterations at β = 0.7, and shrinks each iteration.
  - A 0.3 s delay is found by `fit_delay` to within one sample and isn't corrected into P.
  - With `H_true = 2.5 Ĥ` (outside decision 2's bound), the error rises at iteration 2, and
    the tool says to lower β.
  - Bins outside the band and below 1 % energy are bitwise unchanged.
  - The cap holds: no bin changes by more than 30 % per iteration.
  - A correction that would exceed the stroke limit is refused, not clipped.
  - A quadratic term in the fake tank (`Y += c · (x²)'s spectrum`) still converges in the band,
    and the 2 fp bins are not touched.
- `test_generate_waves.py`: `irregular_signal` gives the same samples as before the split, and
  `spec_from_row` → `irregular_spectrum` reproduces a generated run.
- `test_transfer_function.py`: `analyse` prints and writes the same as before `measured_run`.
- README: a section *Correcting a run* under *Generating motion files*, after *Check and play*.
  In [TODO.md](TODO.md), list this sketch with the others.

## Check on the data / rig

1. **Offline on what exists.** Run `correct_drive` on 2026-11-17 `check2` run 0 and
   2026-11-18 `check3` run 0 (irregular, Hs 0.03 m, Tp 1.2 s). Don't play the result; just
   check that:
   - the rebuilt P_0 matches the file;
   - the reported band error is close to `transfer_function`'s scatter of `gain_raw` about the
     table for the same run;
   - the corrected file passes `play_dat --dry-run`.
2. **Repeatability (sets the stopping criterion).** Play iteration 0 twice, with the same
   pause before each. The band error and time-series rms between the two plays is the floor
   that no iteration can get below. Pass: below 3 % band error. Above that, the probe or the
   tank's state between runs dominates (seiching, `check_still` warnings), and that has to be
   fixed first.
3. **Iterate.** Play iterations 1–3 of run 0, each recorded. Pass:
   - The band error and Hs error fall each iteration and reach the floor from Check 2 by
     iteration 3.
   - τ stays within ±20 ms of iteration 0.
   - The largest `|dX/X|` shrinks each iteration.
4. **Transfer to another seed.** Apply iteration 3's per-bin factor `P_3 / P_0` to seed 2 (same
   Hs, Tp). If its band error is near the floor too, the correction is a property of the tank
   and belongs in the gain model. If not, it is specific to the realisation.

## Open

- **Absolute timing.** Is the `.dat`'s trigger channel recorded by catman? If it is, τ can be
  measured against the trigger instead of fitted away. A focused wave then focuses at the
  intended time, not only in the intended shape.
- **Where the target is.** The probe at 3.0 m is not where the model sits. Either the probe goes
  to the model's position for the iterations, or the target is moved with linear travel. The
  first is better: linear travel is exactly what the iteration corrects.
- **β and the cap:** start at 0.7 and 30 %, and revisit after Check 3.
- **Pause between runs:** whether 60 s is enough for Check 2's floor at Tp 1.2 s.

## Later, not in the first pass

- **Focused (transient) waves.** A new test-matrix kind, `focused` (focus time, focus
  position, spectrum). The same iteration works once there is a trigger reference (Open).
- **Second-order correction.** Remove spurious long waves and free harmonics with second-order
  wavemaker theory (Schäffer 1996). Worth it at steepness above about 1/30. Needs the gain
  model's linear part first.
- **Incident waves only.** With 2–3 probes (Mansard–Funke), match the incident wave and leave
  reflections out of the error. This pairs with the same item in the gain-model sketch.
- **Feeding the gain model.** Every iteration's `Y/X` is a measurement. Add each iteration's
  run to `gain_model` like any other run, tagged with its iteration.
