## MGATE Configuration Backup

Backups of the ladertanken MGate 5101 (`192.168.1.70`) configuration, exported from MGate
Manager. To restore: MGate Manager, *Import*, then check the settings below against the gateway.

| File | Date | State |
|---|---|---|
| `mgate_ladertanken_wavemaker_Both_Working_11_11_2026_holding_torque.dat` | 2026-11-11 | Both rigs on the daisy chain, working together |

Settings in the 2026-11-11 backup (COMMISSIONING.md, *Vertical oscillator*):

| Slave | Drive | User parameters | Modules | Input / output bytes (registers) | Output fault timeout | Output fault value |
|---|---|---|---|---|---|---|
| 2 | Wavemaker (MPB-04) | `80 00 08` | ParamCh not used ×2, Input 7 Words, Output 6 Words | 0-13 / 0-11 (0 / 0) | 500 ms | `00 02`, then `00`: control word `0x0002`, operating mode without torque (TODO 48, fixed 2026-11-11) |
| 3 | Vertical oscillator (MPB-05) | `C0 00 08` (Fail_Safe) | the same | 14-27 / 12-23 (7 / 6) | 500 ms | all `00` |

Keep in mind:
- The MGate assigns each slave's bytes when the slave is added and never moves them. Add slave 2
  first, then 3; a re-added slave also loses its user parameters and fault value.
- The scan fills user parameters from the old GSD (`80 00 08`); slave 3 needs `C0 00 08` typed in.
- Saving a configuration restarts the gateway; the wavemaker drive then shows F4009 (bus failure)
  once: clear it in IndraWorks before starting its service.
- "holding_torque" in the file name: after a stop the wavemaker drive stays in operating mode
  (A0012), but **without** torque; the flap is free and falls to its rest.
