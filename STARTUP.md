# Startup checklist

Every lab session, before enabling a drive: the wavemaker first, then the oscillator part if it
runs too. It assumes commissioning is done
([COMMISSIONING.md](COMMISSIONING.md)); if anything below fails, stop and check the item named in
brackets there.

## Before launch

- [ ] Emergency stop is reachable and released. It is the only hardware stop; stops from the
      software (web page, bridge, Ctrl-C) do not remove drive power.
- [ ] Travel area is clear: nothing and nobody in reach of the flap, actuator or belt.
- [ ] Water depth measured. If the level has changed since the last session, reset the spring
      tension (item 34) and use the new depth at launch.
- [ ] Blue ropes: no new fraying or broken strands (item 34).
- [ ] Belt: tension and condition look normal (item 1).

## Launch

- [ ] If the water depth changed, set `WATER_DEPTH` with `sudoedit /etc/default/wavemaker` (see
      *Water depth* in the README).
- [ ] Start the service: `sudo systemctl start wavemaker@ladertanken` ([rpi/README.md](rpi/README.md)).
      Follow the log with `journalctl -u wavemaker@ladertanken -f`.
- [ ] Activation causes no motion.
- [ ] With the drive holding upright, motor torque or current in IndraWorks is close to zero
      (spring balance, item 34).
- [ ] Run `return_to_upright` and check the flap is visually upright (a jumped belt tooth shows as
      an offset, item 1).

## End of session

- [ ] Run `return_to_upright` and check the flap is visually upright.
- [ ] Stop the service (`sudo systemctl stop wavemaker@ladertanken`), so the controller disables
      the drive. If forgotten, it stops by itself after 2 h without a wave.
- [ ] Draining the tank? Release the spring tension first (item 34).

## Oscillator (ladertanken)

Both drives stay on the PROFIBUS line (MGate → wavemaker, termination OFF → oscillator,
termination ON): wavemaker slave 2 (blocks 0 / 0), oscillator slave 3 (7 / 6). Nothing is
replugged; each yaml reaches only its own drive. **The wavemaker and the vertical oscillator run
together** (since 2026-11-11, TODO 47); the sloshing oscillator only alone.

1. The oscillator's mains must be on at the cabinet (A0012 `Ab`, not A0013 `bb`).
2. `OSCILLATOR=vertical` in `/etc/default/oscillator` for the vertical rig; `sloshing` (or no
   line) for the sloshing one. One unit runs either oscillator, so those two never run together.
3. Start what is needed, in either order: `sudo systemctl start wavemaker@ladertanken`,
   `sudo systemctl start oscillator@ladertanken` (or `start_wavemaker.bat`,
   `start_oscillator.bat`). Pages: 8080 (wavemaker), 8081 (oscillator). The soft stop button
   stops every rig that runs.
4. The controller's `Drive status` line in the journal is the position check: `OM AF`, the
   position near where the slide or flap stands.
5. **After a stop the wavemaker drive stays in operating mode without torque** (A0012, `Ab`):
   the MGate's fault value for slave 2 is control word `0x0002` (since 2026-11-11, TODO 48). The
   flap then falls to its rest (about +194-211°); *Return to upright* after the start. With the
   old fault value (all `00`) every start after a stop failed with F2074.
6. **`mgate_example` without options is slave 2 (the wavemaker)**; the oscillator is
   `--input-base 7 --output-base 6 --profibus-address 3`. In parameter mode it shows
   `SW=0x0888 PM --` with everything 0 (the drive sends no actual values). Not while a rig runs.

A wavemaker drive powered off on the chain shows F4009; clear it before starting its service.
