"""
Tests for the soft_stop launch argument and the web port defaults of both launches.

lab.launch.py and oscillator.launch.py (T14, T11). The launch descriptions are built and their
conditions evaluated; nothing is started. Needs the workspace sourced (the launch files look up
package share directories).

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_launch_soft_stop.py -v
"""

import importlib.util
import pathlib

import pytest

launch = pytest.importorskip('launch')
from ament_index_python.packages import PackageNotFoundError  # noqa: E402, I100
from launch.actions import DeclareLaunchArgument  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[2]
LAUNCH_FILES = {
    'wavemaker': REPO / 'wavemaker_bringup' / 'launch' / 'lab.launch.py',
    'oscillator': REPO / 'oscillator_bringup' / 'launch' / 'oscillator.launch.py',
}


def description(rig):
    spec = importlib.util.spec_from_file_location(f'{rig}_launch', LAUNCH_FILES[rig])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        return module.generate_launch_description()
    except PackageNotFoundError as error:
        pytest.skip(f'workspace not sourced: {error}')


def arguments(ld):
    return {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


def context(ld, **overrides):
    """Return a launch context with every argument at its default, then the overrides."""
    ctx = launch.LaunchContext()
    for name, arg in arguments(ld).items():
        if arg.default_value is not None:
            ctx.launch_configurations[name] = ''.join(s.perform(ctx) for s in arg.default_value)
    ctx.launch_configurations.update(overrides)
    return ctx


def started(ld, ctx):
    """Return {executable name: whether the node starts}."""
    return {n.node_executable: (n.condition.evaluate(ctx) if n.condition else True)
            for n in ld.entities if isinstance(n, Node)}


@pytest.mark.parametrize('rig', LAUNCH_FILES)
def test_soft_stop_argument_defaults_to_true(rig):
    ld = description(rig)
    default = arguments(ld)['soft_stop'].default_value
    assert ''.join(s.perform(launch.LaunchContext()) for s in default) == 'true'


@pytest.mark.parametrize('rig', LAUNCH_FILES)
def test_soft_stop_node_starts_by_default(rig):
    ld = description(rig)
    assert started(ld, context(ld))['soft_stop'] is True


@pytest.mark.parametrize('rig', LAUNCH_FILES)
def test_soft_stop_false_leaves_only_the_soft_stop_node_out(rig):
    ld = description(rig)
    with_node = started(ld, context(ld))
    without = started(ld, context(ld, soft_stop='false'))
    assert without.pop('soft_stop') is False
    with_node.pop('soft_stop')
    assert without == with_node  # controller, bridge, lifecycle manager: unchanged
    assert all(without.values())


def test_oscillator_controller_starts_without_soft_stop():
    ld = description('oscillator')
    assert started(ld, context(ld, soft_stop='false'))['controller_node'] is True


@pytest.mark.parametrize('rig, port', [('wavemaker', '8080'), ('oscillator', '8081')])
def test_web_port_default(rig, port):
    ld = description(rig)
    default = arguments(ld)['port'].default_value
    assert ''.join(s.perform(launch.LaunchContext()) for s in default) == port
