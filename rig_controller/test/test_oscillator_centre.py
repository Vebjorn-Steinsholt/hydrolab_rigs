"""
Tests for TODO 54: the oscillator's centre from /etc/default/oscillator.

Two parts:
- oscillator.launch.py: upright_position_m:= overrides the yaml's value, and empty leaves it out.
  The launch description is built and the controller's OpaqueFunction run; nothing is started.
  Needs the workspace sourced (the launch file looks up package share directories).
- rpi/run.sh: passes VERTICAL_UPRIGHT_M or SLOSHING_UPRIGHT_M, whichever matches OSCILLATOR, as
  upright_position_m:=. Run with a fake $HOME whose install/setup.bash puts a stub `ros2` first
  on PATH, which prints its arguments. Needs /opt/ros/jazzy (run.sh sources it).

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_oscillator_centre.py -v
"""

import importlib.util
import os
import pathlib
import subprocess

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
LAUNCH_FILE = REPO / 'oscillator_bringup' / 'launch' / 'oscillator.launch.py'
RUN_SH = REPO / 'rpi' / 'run.sh'


# --- oscillator.launch.py ---------------------------------------------------------------------

def _launch():
    launch = pytest.importorskip('launch')
    from ament_index_python.packages import PackageNotFoundError
    spec = importlib.util.spec_from_file_location('oscillator_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        return launch, module.generate_launch_description()
    except PackageNotFoundError as error:
        pytest.skip(f'workspace not sourced: {error}')


def _arguments(ld):
    from launch.actions import DeclareLaunchArgument
    return {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


def controller_parameters(**overrides):
    """Return the controller's evaluated parameter list, every argument at its default."""
    launch, ld = _launch()
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    ctx = launch.LaunchContext()
    for name, arg in _arguments(ld).items():
        if arg.default_value is not None:
            ctx.launch_configurations[name] = ''.join(s.perform(ctx) for s in arg.default_value)
    ctx.launch_configurations.update(overrides)

    nodes = [n for e in ld.entities if isinstance(e, OpaqueFunction)
             for n in e.execute(ctx) or [] if isinstance(n, Node)]
    nodes += [n for n in ld.entities if isinstance(n, Node)]
    controllers = [n for n in nodes if n.node_executable == 'controller_node']
    assert len(controllers) == 1, 'exactly one controller'
    # Node keeps its parameters private; this is what launch evaluates at start.
    return evaluate_parameters(ctx, controllers[0]._Node__parameters)


def overridden(params):
    """Return the parameter dicts merged, the yaml file left out."""
    merged = {}
    for p in params:
        if isinstance(p, dict):
            merged.update(p)
    return merged


def test_upright_argument_defaults_to_empty():
    launch, ld = _launch()
    default = _arguments(ld)['upright_position_m'].default_value
    assert ''.join(s.perform(launch.LaunchContext()) for s in default) == ''


@pytest.mark.parametrize('oscillator', ['sloshing', 'vertical'])
def test_empty_upright_keeps_the_yaml(oscillator):
    params = controller_parameters(oscillator=oscillator)
    assert pathlib.Path(params[0]).name == 'oscillator.yaml'
    assert 'upright_position_m' not in overridden(params)


@pytest.mark.parametrize('oscillator', ['sloshing', 'vertical'])
def test_upright_overrides_the_yaml(oscillator):
    params = controller_parameters(oscillator=oscillator, upright_position_m='0.05')
    assert pathlib.Path(params[0]).name == 'oscillator.yaml'
    assert overridden(params) == {'bond_timeout': 4.0, 'upright_position_m': 0.05}


def test_override_comes_after_the_yaml():
    # Later entries win in ROS 2, so the override must follow the yaml file.
    params = controller_parameters(upright_position_m='0.05')
    assert 'upright_position_m' in params[-1]


def test_negative_upright_is_passed():
    assert overridden(controller_parameters(upright_position_m='-0.03'))[
        'upright_position_m'] == -0.03


# --- rpi/run.sh -------------------------------------------------------------------------------

ROS2_STUB = '#!/bin/sh\necho "ros2 $*"\n'


@pytest.fixture
def run(tmp_path):
    if not pathlib.Path('/opt/ros/jazzy/setup.bash').exists():
        pytest.skip('run.sh sources /opt/ros/jazzy')
    stub = tmp_path / 'bin'
    stub.mkdir()
    (stub / 'ros2').write_text(ROS2_STUB)
    (stub / 'ros2').chmod(0o755)
    install = tmp_path / 'ros_ws' / 'install'
    install.mkdir(parents=True)
    # Sourced after /opt/ros, which puts the real ros2 on PATH; the stub must come first.
    (install / 'setup.bash').write_text(f'PATH="{stub}:$PATH"\n')

    def run(rig='oscillator', **variables):
        """Run run.sh; return the launch arguments it passes, and the lines it logged before."""
        env = {k: v for k, v in os.environ.items()
               if not k.endswith('_UPRIGHT_M') and k not in ('RUNTIME_DIRECTORY', 'OSCILLATOR')}
        env.update(HOME=str(tmp_path), ROS_DOMAIN_ID='42', RIG=rig, **variables)
        result = subprocess.run(['bash', str(RUN_SH)], env=env, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        *log, command = result.stdout.strip().splitlines()
        launch = {'oscillator': 'oscillator_bringup oscillator.launch.py',
                  'wavemaker': 'wavemaker_bringup lab.launch.py'}[rig]
        assert command.startswith(f'ros2 launch {launch} ')
        return command.split()[4:], log

    return run


def test_vertical_upright_is_passed_and_logged(run):
    args, log = run(OSCILLATOR='vertical', VERTICAL_UPRIGHT_M='0.050')
    assert 'oscillator:=vertical' in args
    assert 'upright_position_m:=0.050' in args
    assert log == ['VERTICAL_UPRIGHT_M=0.050: centre from /etc/default/oscillator, '
                   'not oscillator.yaml']


def test_sloshing_upright_is_passed(run):
    args, _ = run(OSCILLATOR='sloshing', SLOSHING_UPRIGHT_M='0.120')
    assert 'upright_position_m:=0.120' in args


def test_sloshing_is_the_default_oscillator(run):
    args, _ = run(SLOSHING_UPRIGHT_M='0.120', VERTICAL_UPRIGHT_M='0.050')
    assert 'oscillator:=sloshing' in args
    assert 'upright_position_m:=0.120' in args


def test_other_oscillators_upright_is_ignored(run):
    # A vertical centre is a valid but wrong sloshing centre: it must not carry over.
    args, log = run(OSCILLATOR='sloshing', VERTICAL_UPRIGHT_M='0.050')
    assert not any(a.startswith('upright_position_m:=') for a in args)
    assert log == []


@pytest.mark.parametrize('value', [None, ''])
def test_unset_or_empty_passes_nothing(run, value):
    extra = {} if value is None else {'VERTICAL_UPRIGHT_M': value}
    args, log = run(OSCILLATOR='vertical', **extra)
    assert not any(a.startswith('upright_position_m:=') for a in args)
    assert log == []


def test_wavemaker_ignores_the_upright_variables(run):
    args, log = run(rig='wavemaker', WAVEMAKER='ladertanken', VERTICAL_UPRIGHT_M='0.050')
    assert not any(a.startswith('upright_position_m:=') for a in args)
    assert log == []
