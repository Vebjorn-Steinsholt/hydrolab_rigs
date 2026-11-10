#!/usr/bin/env bash
# ExecStartPre: refuse combinations that cannot run together on this Pi. The wavemaker and the
# vertical oscillator may run together (TODO 47); the sloshing oscillator only alone.
# $1: this unit's rig (wavemaker | oscillator), $2: the Pi user (%i).
set -eo pipefail
rig="$1" pi="$2"
marker="/run/oscillator-$pi/oscillator"   # written by run.sh, removed when oscillator@ stops

# Starting and stopping count as running: the other unit may be in its own ExecStartPre, or
# still shutting its drive down. (is-active exits non-zero for these; ignored inside [[ ]].)
running() { [[ $(systemctl is-active "$1") =~ ^(active|activating|deactivating|reloading)$ ]]; }

if [[ $rig == wavemaker ]]; then
  if running "oscillator@$pi"; then
    running_osc="$(cat "$marker" 2>/dev/null || echo unknown)"
    if [[ $running_osc != vertical ]]; then
      echo "oscillator@$pi runs '$running_osc'; only the vertical oscillator can run with the wavemaker" >&2
      exit 1
    fi
  fi
else
  if [[ ${OSCILLATOR:-sloshing} != vertical ]] && running "wavemaker@$pi"; then
    echo "wavemaker@$pi is running; only the vertical oscillator can run with it" >&2
    exit 1
  fi
fi

# Catches a launch by hand on this unit's own port (the same defaults as run.sh).
if [[ $rig == wavemaker ]]; then default_port=8080; else default_port=8081; fi
port="${PORT:-$default_port}"
if ss -Hltn "sport = :$port" | grep -q .; then
  echo "port $port is in use, probably a launch by hand; stop it first" >&2
  exit 1
fi
