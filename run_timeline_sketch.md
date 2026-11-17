# Sketch: per-run offsets and clock drift in `transfer_function`

What to code so `transfer_function` finds where each run is in the catman record by itself,
also in long files with several runs, and why.

**Status (2026-11-17): changes 1–4 done, not yet committed; `play_dat` not yet on the Pi.**
Found on the 2026-11-17 check (`~/wave_sessions/2026-11-17_check`, TODO.md item 43). Builds on
`transfer_function` (its sketch: `git show c6bba03:transfer_function_sketch.md`).

## As built: where the code differs from the decisions below

1. **The envelope anchor (decision 6)** matches the envelopes over the whole first run, not
   `COARSE_S` of it: 120 s inside the irregular run was too flat to find (1.0× the next, at
   933 s). The envelopes find the run and give the quality; their slopes (the ramps) then fix
   the lag within one smoothing length. On a regular run the envelopes' peak alone is a few per
   mille high over a second and sat 0.6 s early, outside the walk's window. The envelope is of
   the travel-undone probe, so no `travel_s` is subtracted, and the command's envelope is taken
   on the whole file, then cut.
2. **The walk on the probe (decision 6)** runs on the travel-undone probe turned back by linear
   theory's 90° lead (`lag_90`). Without that the lag sits T/4 off the stroke's, and the
   jump between runs of different periods comes close to the window.
3. **`--offset-s` without `--drift-ppm` (decision 7)** fits this run's pieces with a free
   intercept and the `DRIFT_MIN_S` rule. With the intercept held at X, an X that is off by the
   drive's lag (0.17 s) reads as about 900 ppm. If that walk fails, the tool warns and takes 0.
4. **eps = 0 cuts at whole samples** (`on_file_time`), so `--offset-s X --drift-ppm 0` gives
   byte for byte the CSV of the version before (checked against `4285c3d`).
5. **Pieces are at most `CHUNK_S`** (`ceil`), as decision 3 says; the sketch's code used `round`.
6. **`play_dat` (change 2)**: `Run.start_sample` was there already. The timing is
   `goal_time_s`, tested without ROS. After runs skipped with `--runs`, the gap is played from
   the previous result as before. On `fake_controller`, goals went out within 8 ms of their
   time in the file.
7. **Tests**: late goals of 60 ms, not 50 (three file samples, so the truth is exact). The
   bias test runs at 600 ppm: at 250 ppm the single offset loses only ~0.9 % at 1 Hz. The
   late-goal refusal uses 0.55 Tp: at 0.6 Tp the window's edge sits on the side lobe one
   period away. "Exactly today's numbers" is tested as a quarter sample in `--offset-s`
   changing nothing.

**Check 1 (2026-11-17), done:** `runs` 31.716 31.947 32.005 32.064 32.143 32.204, `late` 0 62
102 142 202 243 ms, drift +255 ppm (rms 0.5 ms). Run 0: gain 1.019 (0.8 s), 1.106 (1.0 s),
drive 0.98–1.00, coherence ≥ 0.96. Regular runs within 0.07–0.29 % of `run1`–`run5`. Without
the stroke: +258 ppm, the same gains within 0.1 %. Checks 2–4 are still open.

Line numbers are as of `master` on 2026-11-17 (`4285c3d`).

## Problem

`transfer_function` assumes one offset for the whole file: file time t is at record time
`offset + t`. `offset_from_stroke` (`transfer_function.py:154`) cross-correlates the command up
to the last run's end against the draw wire, and steps 4–5 of `analyse` (lines 420–449) cut every
run at that one offset.

On 2026-11-17 (`check2.dat`: a 600 s irregular run, then regular runs at 0.8, 1.0, 1.1, 1.3 and
1.6 s, 60 s pauses), this refused the session with *"the stroke matches the command upside down"*,
although the sign was right. Two things move the run away from a single offset:

1. **catman's clock against the Pi's: +253 ppm.** In ten 60 s pieces of the irregular run, the
   lag rises from 31.727 s to 31.861 s. It's a straight line to ±2 ms. The bag shows the
   setpoints following the file with a constant lag on the Pi's own clock (the controller samples
   the goal by steady-clock time, `controller_node.cpp:1271`), so the drift is between the two
   clocks, not in the playback.
2. **Each goal starts late, and the delays add up.** `play_dat` sends each run as a goal of its
   own and starts the next pause after the previous result arrives (`play_dat.py:308-319`). In
   the bag, each regular run's setpoint is about 49 ms later against the file than the run before
   it: +48, +97, +146, +195, +244 ms. The drive's lag (position against setpoint) stays at
   94–97 ms throughout.

Over the file the two add up to 0.49 s, about 0.4 Tp. The one match smears, and a negative peak
0.68 s away (r = −0.78) beats the true positive one (r = +0.67).

What each error costs:

| Error | Effect |
|---|---|
| A constant offset error, small against a segment | Nothing on \|G\| (it rotates the phase only); the residual phase and `delay` take it |
| An offset off by a period or more | The window shifts; ramp samples come in; the stillness check looks at the wrong place |
| Drift inside a run (±0.075 s over run 0's 588 s window) | The phase turns between segments, so averaging shrinks \|S_xy\|: G_cmd and drive come out low, coherence drops |

The last row is measured. Run 0 analysed as today (`--offset-s 31.80`) against a prototype
that resamples the record by 253 ppm (`--offset-s 31.7165` on the resampled record):

| T [s] | gain today | gain, drift removed | regular run | drive today | drive, removed | coh today → removed |
|---|---|---|---|---|---|---|
| 0.8 | 0.971 | 1.021 | 0.989 | 0.938 | 0.983 | 0.90 → 1.00 |
| 1.0 | 1.069 | 1.107 | 1.109 | 0.957 | 0.991 | 0.93 → 1.00 |
| 1.3 | 1.115 | 1.133 | 1.127 | 0.979 | 0.997 | 0.96 → 1.00 |
| 1.6 | 1.116 | 1.132 | 1.129 | 0.983 | 0.999 | 0.96 → 0.99 |

The ± on the gain falls from 0.020–0.030 to 0.003–0.009. The water column (stroke → wave, both
on catman's clock) doesn't change, which is the check that the bias came from the command side.

**What changes for the user.** The command from item 43 works as written, without `--offset-s`:

```bash
ros2 run rig_controller transfer_function check2.dat ladertanken --run 3 \
    --record check2.ASC --probe WP1 --probe-distance-m 3.0 \
    --stroke WM_Position --stroke-scale -1.342 --stroke-height-m 1.375 \
    --depth-m 1.0 -o run3 --plot
```

```text
offset       32.064 s at the run's start (from the stroke; anchor 1.3x the next, run 0)
runs         at each start [s]: 31.717 31.947 32.005 32.064 32.144 32.205
late         goal start after the file, drift removed [ms]: 0 63 104 143 203 245
drift        +253 ppm (catman's clock against the Pi's), 10 pieces in 1 run, rms 1.2 ms
```

### Decisions

1. **The timeline model.** For a file time t inside run k:

   ```text
   record time = a_k + t + ε · (t − start_k)
   ```

   `a_k` is per run: the record time of the run's start, minus `start_k`, so it means what
   `--offset-s` means today. It holds the goal's late start and the drive's lag. `ε` is one per
   file: the two clocks. The drive's lag stays inside `a_k`, as today. Its change with period
   (about ±15 ms between 0.8 and 1.6 s on 2026-11-17) is far below what matters for |G|.

2. **Anchor on the first run only.** The whole-file match stays, but on the file from sample 0 to
   `min(end of the first run, its start + COARSE_S)`, with `COARSE_S = 120 s`. Within 120 s,
   drift moves the lag by at most 0.12 s, even at 1000 ppm, and no goal boundary falls inside
   the match. The sign check and `match_quality` stay here, unchanged. The sign is decided once,
   where the match is clean.

3. **Walk the runs in time order, in pieces.** Each run's inner part (ramps off) is cut into
   pieces of at most `CHUNK_S = 60 s`; a run shorter than that is one piece. Each piece's lag is
   searched only within ±`SEARCH_TP = 0.4` periods of the previous piece's lag. Between two
   pieces of a run the drift adds at most 60 ms (at 1000 ppm), and between runs it adds a goal's
   latency (49 ms) plus the pause's drift (15 ms). Both are far inside 0.4 T (0.24 s at 0.6 s).
   The window is narrower than T/2, so a peak one period away (regular) or a flipped one half a
   period away can't win. A peak on the window's edge means the prediction was off. That's
   refused with the piece named, and `--offset-s` is the way out.

4. **Fit ε with one intercept per run.** Least squares on all pieces, `lag = a_k + ε·(t_mid −
   start_k)`. The clock is shared by the file, so every run with at least two pieces adds to ε.
   With less than `DRIFT_MIN_S = 180 s` of such runs (regular-only files), ε = 0, printed as
   *not measured*. Regular runs of 15–25 s drift less than 10 ms over the window, so that's fine.
   Refuse when |ε| > `DRIFT_MAX_PPM = 1000` (a dropped catman block, or the wrong record). Warn
   when the rms residual is above `DRIFT_RES_MAX_S = 0.01`: something moved the timeline
   inside a run.

5. **Resample only the run being analysed.** The record is decimated to the file's rate as now
   (`record_channel`). Then the run's window, plus `STILL_S` before it, is read at the record
   times decision 1 gives, with `np.interp`. Linear interpolation at 50 Hz loses less than
   0.05 % amplitude at 1.8 s. `undo_travel` stays on the whole probe series before this, as now:
   the travel time (about 2 s) changes by 0.5 ms at 253 ppm.

6. **Without the stroke, the same walk on the probe.** The anchor is `offset_from_envelopes`
   on the first run, cut as in decision 2. The walk and the ε fit run on the travel-undone probe
   (`undo_travel` already gives it). The water's phase biases each period's lag by a fraction of
   T, but it is constant within an irregular run, so ε is unbiased. For |G| that bias doesn't
   matter (the first row of the table above). This keeps the probe-only path as good as the
   stroke one for the gain.

7. **`--offset-s` stays exact, and `--drift-ppm` is new.** `--offset-s X` sets `a_k = X` for
   the run being analysed, with no anchor and no walk, as today. ε then comes from that run's own
   pieces, with `a_k` held at X, when the run has at least two pieces. Otherwise it's 0.
   `--drift-ppm E` sets ε and skips its fit. `--offset-s X --drift-ppm 0` is exactly today's
   behaviour, as a way out.

8. **What's reported.** In `info` (so in the `.txt` too): the run's `a_k` and how it was found;
   `a_k` for every run walked; each run's late start, `a_k − a_0 − ε·(start_k − start_0)`, which
   should match the bag's setpoint steps; and ε with its piece count and rms. The CSV doesn't
   change.

## Changes

1. **`rig_controller/src/transfer_function.py`.**

   Constants, next to `MATCH_MIN` (line 30):

   ```python
   COARSE_S = 120.0         # the anchor matches at most this much of the first run
   CHUNK_S = 60.0           # the walk: pieces of a run at most this long
   SEARCH_TP = 0.4          # each piece is searched within +-this many periods of the last
   DRIFT_MIN_S = 180.0      # fit the clock drift only from at least this much of long runs
   DRIFT_MAX_PPM = 1000.0   # refuse a drift above this
   DRIFT_RES_MAX_S = 0.01   # warn when the pieces scatter more than this about the fit
   ```

   A lag in a narrow window, with sub-sample interpolation:

   ```python
   def local_lag(x_file, y_all, a, n, guess_s, w, dt) -> tuple[float, float, bool]:
       """
       Return (lag_s, r, at_edge) of y_all against x_file[a:a + n], within guess_s +- w samples.

       r is the normalised correlation at the peak; at_edge says the peak is on the window's
       edge (the prediction was off by more than the window).
       """
       xs = x_file[a:a + n]
       lo = max(0, a + round(guess_s / dt) - w)
       hi = min(len(y_all) - n, a + round(guess_s / dt) + w)
       if hi < lo:
           raise ValueError('the run is not inside the record at this offset')
       seg = y_all[lo:hi + n]
       c = signal.correlate(seg - seg.mean(), xs - xs.mean(), mode='valid')
       i = int(np.argmax(c))
       at_edge = i in (0, len(c) - 1)
       d = 0.0
       if not at_edge:                                   # parabola through the peak
           y0, y1, y2 = c[i - 1], c[i], c[i + 1]
           d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
       r = c[i] / (n * xs.std() * y_all[lo + i:lo + i + n].std())
       return (lo + i + d - a) * dt, r, at_edge
   ```

   The anchor (replaces `offset_from_stroke`; `offset_from_envelopes` gets the same cut):

   ```python
   def anchor_from_stroke(x_file, s_all, dt, first, min_sep) -> tuple[float, float]:
       """(lag_s, match quality) of the first run: the whole-record match, on <= COARSE_S of it."""
       end_s = min(float(first['end_s']), float(first['start_s']) + COARSE_S)
       x0 = x_file[:round(end_s / dt) + 1]
       # ... the body of today's offset_from_stroke on x0, sign check included
   ```

   The walk and the fit:

   ```python
   @dataclass
   class Timeline:
       a: dict[int, float]        # run number -> lag at its start [s]
       eps: float                 # drift; 0 if not measured
       eps_pieces: int            # pieces in runs of >= 2 pieces (0: not measured)
       rms_s: float


   def pieces(row, dt) -> list[tuple[int, int]]:
       """(first sample, length) of each piece of the run's inner part (ramps off)."""
       ramp = round(float(row['ramp_s']) / dt)
       a, b = round(float(row['start_s']) / dt) + ramp, round(float(row['end_s']) / dt) - ramp
       n = max(1, round((b - a) * dt / CHUNK_S))
       edges = np.linspace(a, b, n + 1).round().astype(int)
       return [(int(p), int(q - p)) for p, q in zip(edges, edges[1:])]


   def walk(x_file, y_all, dt, table, anchor_s) -> Timeline:
       """
       Lag of every run in the record, piece by piece from the anchor (decisions 3 and 4).

       Each piece is searched within SEARCH_TP periods of the last; play_dat starts each run
       as a goal of its own, ~50 ms later than the file says (2026-11-17), and catman's clock
       drifts against the Pi's (+253 ppm), so one offset for the whole file doesn't hold.
       """
       rows, lag = [], anchor_s                        # (run, t_mid - start, lag)
       for r in (r for r in table if r['end_s']):
           tp = float(r['peak_period_s'] or r['period_s'])
           w = round(SEARCH_TP * tp / dt)
           for p, n in pieces(r, dt):
               if p + round(lag / dt) + n > len(y_all):
                   break                               # the record stops before this piece
               lag, _, edge = local_lag(x_file, y_all, p, n, lag, w, dt)
               if edge:
                   raise ValueError(f"run {r['run']}, piece at {p * dt:.0f} s: the lag left "
                                    f'the +-{SEARCH_TP} T window; give --offset-s')
               rows.append((int(r['run']), (p + n / 2) * dt - float(r['start_s']), lag))
       return fit_timeline(rows, dt)


   def fit_timeline(rows, dt) -> Timeline:
       """lag = a_k + eps * (t - start_k): one intercept per run, one slope (decision 4)."""
       runs = sorted({k for k, _, _ in rows})
       count = {k: sum(1 for kk, _, _ in rows if kk == k) for k in runs}
       long_s = sum(CHUNK_S for k, _, _ in rows if count[k] >= 2)
       A = np.zeros((len(rows), len(runs) + 1))
       y = np.array([lag for _, _, lag in rows])
       for i, (k, t, _) in enumerate(rows):
           A[i, runs.index(k)] = 1.0
           A[i, -1] = t
       if long_s < DRIFT_MIN_S:
           A = A[:, :-1]                               # eps not measured: 0
       sol, *_ = np.linalg.lstsq(A, y, rcond=None)
       eps = float(sol[-1]) if long_s >= DRIFT_MIN_S else 0.0
       res = y - A @ sol
       if abs(eps) * 1e6 > DRIFT_MAX_PPM:
           raise ValueError(f'drift {eps * 1e6:+.0f} ppm, above {DRIFT_MAX_PPM:.0f}: '
                            'the wrong record, or a gap in it; give --offset-s and --drift-ppm')
       return Timeline(a={k: float(sol[j]) for j, k in enumerate(runs)}, eps=eps,
                       eps_pieces=sum(c for c in count.values() if c >= 2) if eps else 0,
                       rms_s=float(np.sqrt(np.mean(res ** 2))))
   ```

   Reading a run on the file's time base (decision 5):

   ```python
   def on_file_time(y_all, dt, a_s, eps, start_s, i_from, i_to) -> np.ndarray:
       """y_all at the record times of file samples i_from..i_to of a run starting at start_s."""
       t = np.arange(i_from, i_to) * dt
       t_rec = a_s + t + eps * (t - start_s)
       y = np.interp(t_rec / dt, np.arange(len(y_all)), y_all, left=np.nan, right=np.nan)
       if np.isnan(y).any():
           raise ValueError('the run is not inside the record at this offset')
       return y
   ```

   In `analyse`, step 4 (lines 420–440) becomes:

   ```python
   first = next(r for r in table if r['end_s'])
   y_walk = s_all if s_all is not None else eta_travel     # decision 6
   if args.offset_s is not None:
       a_k, how = args.offset_s, '--offset-s'
       tl = own_drift(x_file, y_walk, dt, row, a_k)        # decision 7: this run's pieces only
   else:
       anchor_s, quality = (anchor_from_stroke(x_file, s_all, dt, first, min_sep)
                            if s_all is not None else
                            anchor_from_envelopes(x_file, eta_all, dt, tp_s, travel_s,
                                                  first, min_sep))
       # ... MATCH_MIN check on quality, as now
       tl = walk(x_file, y_walk, dt, table, anchor_s)
       a_k, how = tl.a[args.run], ...
   eps = args.drift_ppm * 1e-6 if args.drift_ppm is not None else tl.eps
   ```

   `own_drift` is `walk` over this run's pieces only, starting from `a_k`. Its fit keeps the
   intercept at `a_k` and solves for the slope alone: `eps = Σ t·(lag − a_k) / Σ t²` over the
   pieces, when there are at least two. Otherwise eps is 0.

   `eta_travel = undo_travel(eta_all, ...)` moves up from step 5 (line 444), so the walk and the
   cut use the same series. Step 5 cuts on the file's time base:

   ```python
   a, b = round(float(row['start_s']) / dt), round(float(row['end_s']) / dt)
   n_before = round(STILL_S / dt)
   eta_run = on_file_time(eta_travel, dt, a_k, eps, float(row['start_s']), a - n_before, b)
   s_run = (on_file_time(s_all, dt, a_k, eps, float(row['start_s']), a - n_before, b)
            if s_all is not None else None)
   i0 = n_before                                   # the run's start in eta_run
   eta, s = cut_window(i0, len(x), skip, eta_run, s_run)
   check_still(eta_run, i0, dt, eta.std())
   ```

   `check_still` and `cut_window` don't change. The `len(eta_all) < len(x_match)` check (line
   429) goes: the walk stops at the record's end (the `break` in `walk`), and `on_file_time`
   refuses a run outside the record.

   New argument, next to `--offset-s` (line 546):

   ```python
   parser.add_argument('--drift-ppm', type=float,
                       help="catman's clock against the Pi's; skips its fit (0: none)")
   ```

   `info` (lines 497–519): replace the `offset` line with the four lines under *What changes
   for the user*.

2. **`rig_controller/src/play_dat.py` (separate, optional).** Schedule each goal on the file's
   timeline instead of after the previous result, so the latency doesn't add up:

   ```python
   t0 = None
   for run in runs:
       if t0 is not None:                        # the file's time of this run, on the Pi's clock
           pause_end_s = t0 + run.start_s
           if pause_end_s < time.monotonic():
               node.get_logger().warning(f'run {run.index}: '
                                         f'{time.monotonic() - pause_end_s:.2f} s late')
           while time.monotonic() < pause_end_s:
               rclpy.spin_once(node, timeout_sec=0.01)
       ...
       future = client.send_goal_async(goal)
       if t0 is None:
           t0 = time.monotonic() - run.start_s
   ```

   `Run` needs `start_s` (the run's first sample × dt), next to `gap_before_s` (line 81). The
   first-run pause stays as it is. This takes about 49 ms per run off the regular runs' a_k. It
   doesn't remove the need for decision 3: old records keep the latency, the latency still jitters
   by a few ms, and the clock drift is unrelated to it. The 0.1 s `spin_once` timeout becomes
   0.01 s, so the goal goes out within 10 ms of its time.

3. **README.md, *Transfer gain from a run*.** Say that the offset is found per run, what the
   `runs`, `late` and `drift` lines mean, and when to give `--drift-ppm`. Drop the advice to put
   an irregular run first for the offset. It still helps the anchor in a file of regular runs
   only (see *Open*).

4. **TODO.md.** Point at this sketch under item 43.

## Tests and docs

In `rig_controller/test/test_transfer_function.py`, `write_record` (line 89) gets two
arguments. The playback is built on the Pi's clock and then read at catman's:

```python
def write_record(path, x_file, dt, wm, *, ..., clock_ppm=0.0, late_s=0.0, table=None):
    # late_s: run k starts k * late_s after the file says (a goal's latency, play_dat)
    # clock_ppm: catman sample j is at Pi time j / RATE_HZ / (1 + clock_ppm * 1e-6)
```

`late_s` inserts `round(k * late_s / dt)` rest samples before run k, using the run table's
`start_s`. `clock_ppm` evaluates the 200 Hz series at `j / (1 + ε)` with `np.interp`. The module
fixture's matrix gets a third row, a regular run at 0.9 s, so a walk crosses two goal boundaries.

- `test_drift_and_late_goals`: `clock_ppm=250`, `late_s=0.05`. With no `--offset-s`, every
  run's a_k is within 5 ms of the truth, the printed drift is 250 ± 5 ppm, and the late starts
  are 0, 50, 100 ± 5 ms.
- `test_drift_does_not_bias_the_gain`: the irregular run's gain and drive with
  `clock_ppm=250` agree with those from the clean record within 0.5 %. The same run analysed with
  `--offset-s <true a_k> --drift-ppm 0` comes out at least 2 % low at 1.0 s, so the test shows the
  bias it removes.
- `test_clean_record_unchanged`: the clean record gives today's offset within one sample, and a
  drift of 0 ± 5 ppm.
- `test_flipped_stroke_is_refused` (line 262): unchanged; it must still pass, now via the anchor.
- `test_flipped_stroke_with_drift_is_refused`: `stroke_sign=-1, clock_ppm=250, late_s=0.05`.
- `test_regular_only_file`: a file of two regular runs gives *not measured* and no error.
- `test_late_goal_beyond_the_window_is_refused`: `late_s = 0.6 × T` gives the edge error naming
  run 1, and the same call with `--offset-s` passes.
- `test_drift_from_the_probe`: without `--stroke`, `clock_ppm=250` gives 250 ± 20 ppm.
- `test_offset_and_drift_given`: `--offset-s X --drift-ppm 0` gives exactly today's numbers
  (compare the CSV to a run on the clean record with the old code path).
- `test_drift_above_the_limit_is_refused`: `clock_ppm=2000`.
- `test_play_dat.py` (if change 2 is made): with a fake server that answers 50 ms late, the
  third goal is sent within 15 ms of `t0 + start_s`.

## Check on the data / rig

1. **2026-11-17, without `--offset-s`.** All six runs of `check2.dat` analyse. Expected a_k
   31.717, 31.947, 32.005, 32.064, 32.144, 32.205 s (±5 ms), late starts 0, 63, 104, 143, 203,
   245 ms, and drift 253 ± 5 ppm. The regular runs' gains are within 0.3 % of today's
   (`run1`–`run5.txt` in the session folder; they were cut at the mid-run lag, about 2 ms off).
   Run 0 gives the prototype's column above: gain 1.107 at 1.0 s, drive 0.99–1.00, coherence
   ≥ 0.99.
2. **2026-11-13.** `item38` run 3 (one run, 120 s) and the regular check (`--offset-s 16.04`
   before) give the same gains as in COMMISSIONING.md to 0.5 %, and the check runs no longer
   need `--offset-s`. If the regular-only check still can't be anchored, that's *Open*, item 2.
3. **The late starts against the bag.** The `late` line should match the bag's setpoint
   steps (+48, +97, +146, +195, +244 ms on 2026-11-17) within the drive's period-dependent lag
   (±20 ms).
4. **After change 2,** a session played with the new `play_dat` gives late starts within ±10 ms
   of 0, and the same drift.

## Open

1. **Which clock is off.** 253 ppm is high for two crystals. Check `chronyc tracking` on the Pi:
   is it synced, and what frequency correction does it apply? And does the catman PC (or the
   QuantumX) have NTP? If the Pi runs free (no RTC, no network time), ε may change from boot to
   boot. So measure it every time and never put it in the yaml.
2. **A file of regular runs only.** The anchor can still take a flipped sine for a shifted one
   (TODO item 43). Two ways: keep putting an irregular run first, or anchor on a short
   broadband "chirp" that `generate_waves` puts at the file's start (a few seconds at 2 % of
   the stroke). The second is a generator change; decide after change 1.
3. **0.8 s in the gain table.** The drift-free irregular run gives 1.021 against the regular
   run's 0.989 (that run's window is only 7.2 s, 3 averages). The table has 1.08. Set it to
   about 1.00 after check 1, and take the regular run's value with care until the 0.8 s run is
   longer.

## Later, not in the first pass

- **ε per run instead of per file**, if a long session ever shows the drift changing (a
  slewing NTP). The pieces' residuals (`rms_s`) would show it first.
- **The drive's lag by period** from the `a_k` steps, once `play_dat` (change 2) takes the
  goal latency out: a_k − a_0 − ε·Δt would then be the drive's lag at each period, a free check
  on the `drive` column's phase.
- **Regular windows long enough for 0.8 s.** 15 periods minus 3+3 ramp periods leave 7.2 s.
  Use `n_periods` of 30 at 0.8 s in item 43's matrix, or report the sine fit's ± instead of
  Welch's for regular runs.
