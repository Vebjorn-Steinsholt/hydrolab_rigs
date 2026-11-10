"""
Tests for rpi/refuse_if_busy.sh (T13): which rigs may start next to each other on one Pi.

systemctl and ss are replaced by stubs on PATH; the marker path is redirected into tmp_path. No
systemd or Pi needed.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_refuse_if_busy.py -v
"""

import os
import pathlib
import subprocess

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'rpi' / 'refuse_if_busy.sh'
PI = 'ladertanken'

SYSTEMCTL = """#!/usr/bin/env bash
# is-active <unit>: prints the state from WM_STATE / OSC_STATE, exit 0 only for active
case "$2" in
  wavemaker@*) s="${WM_STATE:-inactive}" ;;
  oscillator@*) s="${OSC_STATE:-inactive}" ;;
  *) s=inactive ;;
esac
echo "$s"
[[ $s == active ]]
"""
# ss -Hltn "sport = :<port>": one line if the port is in BUSY_PORT
SS = """#!/usr/bin/env bash
port="${2#sport = :}"   # $1 is -Hltn
[[ -n $BUSY_PORT && " $BUSY_PORT " == *" $port "* ]] && echo "LISTEN 0 128 0.0.0.0:$port"
exit 0
"""


@pytest.fixture
def run(tmp_path):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for name, body in (('systemctl', SYSTEMCTL), ('ss', SS)):
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    marker = tmp_path / 'marker'
    script = tmp_path / 'refuse_if_busy.sh'
    text = SCRIPT.read_text()
    assert '/run/oscillator-$pi/oscillator' in text
    script.write_text(text.replace('/run/oscillator-$pi/oscillator', str(marker)))

    def run(rig, marker_value=None, **env):
        if marker_value is not None:
            marker.write_text(marker_value + '\n')
        elif marker.exists():
            marker.unlink()
        full_env = {k: v for k, v in os.environ.items()
                    if k not in ('OSCILLATOR', 'PORT', 'WM_STATE', 'OSC_STATE', 'BUSY_PORT')}
        full_env['PATH'] = f'{bin_dir}:{full_env["PATH"]}'
        full_env.update(env)
        result = subprocess.run(['bash', str(script), rig, PI], env=full_env,
                                capture_output=True, text=True)
        return result.returncode, result.stderr.strip()

    return run


# ---------------------------------------------------------------- the wavemaker starting

def test_wavemaker_alone(run):
    assert run('wavemaker') == (0, '')


def test_wavemaker_next_to_vertical(run):
    assert run('wavemaker', 'vertical', OSC_STATE='active') == (0, '')


def test_wavemaker_refused_next_to_sloshing(run):
    rc, err = run('wavemaker', 'sloshing', OSC_STATE='active')
    assert rc == 1
    assert "runs 'sloshing'" in err


@pytest.mark.parametrize('state', ['activating', 'deactivating', 'reloading'])
def test_wavemaker_refused_while_sloshing_starts_or_stops(run, state):
    assert run('wavemaker', 'sloshing', OSC_STATE=state)[0] == 1


def test_wavemaker_refused_when_the_oscillator_has_no_marker_yet(run):
    rc, err = run('wavemaker', None, OSC_STATE='activating')
    assert rc == 1
    assert "runs 'unknown'" in err


def test_wavemaker_ignores_a_stale_marker_of_a_stopped_oscillator(run):
    assert run('wavemaker', 'sloshing', OSC_STATE='inactive') == (0, '')


def test_wavemaker_ignores_the_oscillator_env_file(run):
    # The wavemaker's unit decides from the marker, not from OSCILLATOR (the env file can change).
    assert run('wavemaker', 'vertical', OSC_STATE='active', OSCILLATOR='sloshing') == (0, '')


# ---------------------------------------------------------------- the oscillator starting

def test_vertical_next_to_the_wavemaker(run):
    assert run('oscillator', OSCILLATOR='vertical', WM_STATE='active') == (0, '')


@pytest.mark.parametrize('oscillator', ['sloshing', ''])
def test_sloshing_refused_next_to_the_wavemaker(run, oscillator):
    # An empty or missing OSCILLATOR means sloshing.
    rc, err = run('oscillator', OSCILLATOR=oscillator, WM_STATE='active')
    assert rc == 1
    assert 'wavemaker@ladertanken is running' in err


def test_sloshing_refused_while_the_wavemaker_stops(run):
    assert run('oscillator', OSCILLATOR='sloshing', WM_STATE='deactivating')[0] == 1


def test_sloshing_alone(run):
    assert run('oscillator', OSCILLATOR='sloshing') == (0, '')


# ---------------------------------------------------------------- the port check

@pytest.mark.parametrize('rig, port', [('wavemaker', '8080'), ('oscillator', '8081')])
def test_own_default_port_in_use_is_refused(run, rig, port):
    rc, err = run(rig, OSCILLATOR='vertical', BUSY_PORT=port)
    assert rc == 1
    assert f'port {port} is in use' in err


@pytest.mark.parametrize('rig, other_port', [('wavemaker', '8081'), ('oscillator', '8080')])
def test_the_other_rigs_port_in_use_is_fine(run, rig, other_port):
    assert run(rig, OSCILLATOR='vertical', BUSY_PORT=other_port) == (0, '')


def test_port_from_the_env_file(run):
    assert run('oscillator', OSCILLATOR='vertical', PORT='9000', BUSY_PORT='9000')[0] == 1
    assert run('oscillator', OSCILLATOR='vertical', PORT='9000', BUSY_PORT='8081') == (0, '')
