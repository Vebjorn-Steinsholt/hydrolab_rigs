# Windows lab computer

Double-click files that start and stop the rigs on the lab's Raspberry Pi over ssh. They run the
same `systemctl` commands as [rpi/README.md](../rpi/README.md), *Use*.

| File | Does |
|---|---|
| `start_wavemaker.bat` | Starts `wavemaker@ladertanken` and opens its page (port 8080). A running vertical oscillator keeps running. |
| `start_oscillator.bat` | Starts `oscillator@ladertanken` and opens its page (port 8081). A running wavemaker keeps running. |
| `stop_rigs.bat` | Stops both; each disables its drive. The Pi stays on. |
| `_rig.bat` | Shared by the three; the Pi's user and hostname are set at its top. |

The wavemaker and the vertical oscillator can run together; the sloshing oscillator only alone.
The Pi refuses a start that is not allowed and the window shows why; run `stop_rigs.bat` first.

Stopping a rig waits for its ordered shutdown (up to 45 s); the window stays open until a key is
pressed so the result can be read.

## Setup (once per Windows user)

The Pi needs the sudoers rule for all four commands ([rpi/README.md](../rpi/README.md),
*Oscillator*). Windows 10 and 11 include the OpenSSH client. In a Command Prompt:

```bat
ssh-keygen -t ed25519
type %USERPROFILE%\.ssh\id_ed25519.pub | ssh ladertanken@ladertanken-rpi.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

Press Enter at the `ssh-keygen` prompts (no passphrase: the files cannot type one). The second
command asks to accept the Pi's host key (`yes`) and for the `ladertanken` password, once. Check:

```bat
ssh -o BatchMode=yes ladertanken@ladertanken-rpi.local "sudo -n -l"
```

It should log in without a password and list the four `systemctl` commands.

Anyone logged in as this Windows user can then log in to the Pi as `ladertanken`; lock the PC when
leaving it.

Copy the `windows` folder to the lab computer, or put shortcuts to the `.bat` files on the desktop.

## Troubleshooting

| Message | Cause |
|---|---|
| `Could not log in ... over ssh` | Pi off or not on the network, the key is not on the Pi, or the host key changed (reinstalled Pi: `ssh-keygen -R ladertanken-rpi.local`, then the setup again). If `ladertanken-rpi.local` does not resolve, set `PI_HOST` in `_rig.bat` to the Pi's IP address. |
| `sudo: a password is required` | The sudoers rule is missing or does not list that command. |
| `... only the vertical oscillator can run with ...` | The sloshing oscillator and the wavemaker cannot run together (`refuse_if_busy.sh`): `stop_rigs.bat`, then start again. |
| `port 8080 is in use` (or `8081`) | A launch by hand on the Pi holds the rig's web port. Stop it there. |
| Start failed | `journalctl -u wavemaker@ladertanken -n 50` (or `oscillator@`) on the Pi. |
