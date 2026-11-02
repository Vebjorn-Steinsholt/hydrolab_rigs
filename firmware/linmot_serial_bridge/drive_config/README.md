# LinMot drive configuration, sloshing oscillator

LinMot-Talk backups of the drive (E1200-GP-UC/V1RE, firmware 6.10 Build 20211210, CANopen node 12).
The bridge firmware, `link_test.py` and the controller depend on the settings below; they live on
the drive, not in this repository, so a replaced or reset drive needs this file written back.

| File | What |
| --- | --- |
| `sloshing_2026-10-22.lmc` | The configuration in use. Write this to the drive. |
| `sloshing_as_found_2026-10-21.lmc` | The drive as found before commissioning, for comparison. |

Changes from the as-found configuration (the only seven):

| Parameter (LinMot-Talk parameter tree) | As found | In use | Why |
| --- | --- | --- | --- |
| Motion Control SW → Motor Configuration → Monitoring → Position Monitoring → Position Limits → Minimal Position | −57 mm | 3 mm | Quick stop before the 0 mm end stop |
| … → Position Limits → Maximal Position | 273 mm | 197 mm | Quick stop before the 200 mm end stop |
| Motion Control SW → Errors & Warnings → Error Detection Mask → Minimal Position | off | on | Without it the limit above does nothing |
| … → Error Detection Mask → Maximal Position | off | on | Same |
| Motion Control SW → State Machine Setup → Control Word → Ctrl Word Parameter Force Mask → /Abort Motion Command | forced | not forced | The bridge's watchdog aborts a motion through this bit |
| CANopen Interface → Legacy Sync Watchdog → LSW Enable | off | on | The drive stops itself when the SYNC frames stop (no e-stop on this rig) |
| CANopen Interface → Legacy Sync Watchdog → LSW monitored Cycle Time | 5000 µs | 50000 µs | The bridge sends a SYNC every 20 ms; error after 1.5 × 50 ms |

The CANopen node ID (12) is not in this file: the parameter value is 63, so it most likely comes
from the drive's address switches. The homing (Home Position 200 mm, Initial Position 100 mm) is
unchanged.

## After writing it to the drive

Load the file in LinMot-Talk, write it to the drive, restart the drive if LinMot-Talk asks, and
release LinMot-Talk's control. Then check everything the settings are for (the slide moves; keep
LinMot-Talk open to switch the drive off, the rig has no e-stop):

```bash
cd ~/ros_ws/src/ros2_wavemakers/firmware/linmot_serial_bridge
python3 link_test.py --move --home
```

Expect "All checks passed", and in the watchdog check "the drive stopped itself too (sync
watchdog on)". Then check the limits by jogging slowly towards each end in LinMot-Talk: the drive
must stop with "Min. Position Undershot" near 3 mm and "Max. Position Overshot" near 197 mm.

Compare a backup with the one in use (parameter values that differ):

```bash
python3 - sloshing_2026-10-22.lmc other.lmc <<'EOF'
import re, sys
pat = re.compile(r"\[A \d+ '((?:[^']|'')*)' '((?:[^']|'')*)'")
a, b = ({m[0]: m[1] for m in pat.findall(open(f, 'rb').read().decode('latin-1'))}
        for f in sys.argv[1:3])
for k in sorted(a):
    if k in b and a[k] != b[k]:
        print(f'{k}: {a[k]} -> {b[k]}')
EOF
```
