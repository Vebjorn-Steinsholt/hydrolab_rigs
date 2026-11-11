"""
Tests for T18: both rig units set rto_min on the route to the gateway before the rig starts.

The ExecStartPre line is taken from rpi/wavemaker@.service and rpi/oscillator@.service, with
${MGATE_HOST} filled in as systemd does, and run with a stub `ip` on PATH. No systemd or root
needed.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_units_rto_min.py -v
"""

import os
import pathlib
import re
import shlex
import subprocess

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
UNITS = ['wavemaker@.service', 'oscillator@.service']

# `ip -o route get <host>` prints a route on eth0; any other call is printed, to be checked.
IP_STUB = """#!/bin/sh
if [ "$1 $2 $3" = "-o route get" ]; then
  echo "$4 dev eth0 src 192.168.1.98 uid 0 \\\\    cache "
else
  echo "ip $*"
fi
"""


def exec_start_pre(unit):
    """Return the unit's ExecStartPre= values, continuation lines joined, in order."""
    text = (REPO / 'rpi' / unit).read_text().replace('\\\n', ' ')
    return re.findall(r'^ExecStartPre=(.*)$', text, flags=re.MULTILINE)


def route_command(unit):
    (line,) = [v for v in exec_start_pre(unit) if 'rto_min' in v]
    return line


@pytest.fixture
def run(tmp_path):
    (tmp_path / 'ip').write_text(IP_STUB)
    (tmp_path / 'ip').chmod(0o755)

    def run(unit, mgate_host):
        line = route_command(unit)
        assert line.startswith('+'), 'needs root: the + prefix'
        # systemd replaces ${VAR} before the command runs; an unset variable becomes empty.
        line = line[1:].replace('${MGATE_HOST}', mgate_host)
        argv = shlex.split(line)
        env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}')
        result = subprocess.run(argv, env=env, capture_output=True, text=True)
        return result.returncode, result.stdout.strip(), result.stderr.strip()

    return run


@pytest.mark.parametrize('unit', UNITS)
def test_sets_rto_min_on_the_gateway_route(run, unit):
    assert run(unit, '192.168.1.70') == (
        0, 'ip route replace 192.168.1.70/32 dev eth0 rto_min 20ms', '')


@pytest.mark.parametrize('unit', UNITS)
def test_refuses_to_start_without_mgate_host(run, unit):
    rc, out, err = run(unit, '')
    assert rc != 0
    assert out == ''
    assert err == 'set MGATE_HOST'


@pytest.mark.parametrize('unit', UNITS)
def test_route_is_set_before_the_rig_starts(unit):
    # After EnvironmentFile= is read, and before refuse_if_busy.sh and run.sh.
    pre = exec_start_pre(unit)
    assert 'rto_min' in pre[0]
    assert 'refuse_if_busy.sh' in pre[1]


def test_both_units_use_the_same_command():
    assert route_command(UNITS[0]) == route_command(UNITS[1])
