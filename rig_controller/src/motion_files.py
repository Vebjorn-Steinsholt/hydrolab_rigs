"""
Motion files for play_dat: what generate_waves and generate_oscillations have in common.

Writing the file (.dat), its run table (.csv), the record of how it was made (.txt) and the plot
(.png); assembling runs with rest between them; checking each run against the rig's limits; the
spectrum and synthesis of irregular runs; reading the file back with play_dat's own functions.
Nothing here knows about waves or a particular rig: the generators make each run's positions
and pass in the rig's limits (generate_waves.Wavemaker or rig_limits.RigLimits, which share the
fields used here).

Contract with play_dat.py
  .dat (read_motion)
    - Little-endian, as play_dat reads it ('<i4', '<f4'): int32 time step [ms] > 0, int32
      channel count > 0, then float32 frames row by row; the size must fit whole frames.
    - All values finite. Channel 0 'position', channel 1 'trigger' (CHANNELS); the position
      channel must not be all zeros.
    - Position: metres relative to upright, as the controller takes it (for a wavemaker at the
      attachment point, transfer gain applied). play_dat sends it unchanged and the controller
      applies no gain to pregenerated goals.
  Runs (rest_mask, split_runs, check_run)
    - Rest is |x| <= gap_threshold_m (default 0) and speed <= 1e-6 m/s: rest must be exactly 0.0.
    - Runs are separated by >= MIN_GAP_S (1 s) of rest; shorter rest stays inside the run.
    - Each run starts and ends within UPRIGHT_TOLERANCE_M (2 mm) of upright.
  .csv run table (read_run_table, apply_run_table), next to the .dat with the same base name
    - Required columns run, start_s, end_s; kind and period_s optional (display only).
    - Rows with status 'skipped' are dropped; the rest must match the detected runs in number
      and order, boundaries within RUN_TABLE_TOLERANCE_S (0.05 s).

Contract with the controller (pregenerated goal, controller_node.cpp ~800-950)
  - sample_interval finite and > 0 (from the header).
  - Every absolute position (upright_m + x) within the limits it derives from the yaml.
  - Peak speed, max|diff(x)|/dt, <= the actuator's max_velocity_mps().
  - Peak acceleration, max|second difference|/dt^2, <= max_goal_acceleration_mps2 if set (> 0)
    and <= the actuator's limit (IndraDrive: none; LinMot: has one).
  - A run starting at upright needs no blend from the actuator's position.
"""
