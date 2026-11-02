#!/usr/bin/env bash
# ExecStartPre: refuse to start while the other rig on this Pi runs. The wavemaker and the
# oscillator share the web port, the soft stop on GPIO17 and the ROS domain.
# $1: the other rig's unit, e.g. oscillator@ladertanken. On a Pi without it, is-active is false.
set -eo pipefail

if systemctl is-active --quiet "$1"; then
  echo "$1 is running; stop it first: sudo systemctl stop $1" >&2
  exit 1
fi
# Catches a launch by hand too: lab.launch.py and oscillator.launch.py both start the web page.
port="${PORT:-8080}"
if ss -Hltn "sport = :$port" | grep -q .; then
  echo "port $port is in use, probably a launch by hand; stop it first" >&2
  exit 1
fi
