"""
Tests for src/rig_limits.py: the controller's goal-limit rules, from a yaml and from parameters.

Written before rig_limits.py exists (TODO item 40, sketch.md); they fail until it does.

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_controller/test/test_rig_limits.py -v
"""

import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'rig_controller' / 'src'))
import rig_limits  # noqa: E402

WAVEMAKERS = str(REPO / 'wavemaker_bringup' / 'config' / 'wavemakers.yaml')
OSCILLATOR = str(REPO / 'oscillator_bringup' / 'config' / 'oscillator.yaml')
# A linear IndraDrive in the vertical oscillator's units: 600 x 10 mm/min = 0.1 m/s (T8).
LINEAR = {'drive_name': 'indradrive', 'drive_motion': 'linear',
          'travel_minimum_m': -0.2, 'travel_maximum_m': 0.2, 'max_velocity_rpm': 600.0,
          'drive_m_per_unit': 0.001, 'drive_velocity_per_unit_per_s': 6.0}


# ---------------------------------------------------------------- from the yaml

def test_ladertanken():
    limits = rig_limits.load_limits('wavemaker', 'ladertanken', WAVEMAKERS)
    assert (limits.min_m, limits.max_m) == pytest.approx((-0.14575, 0.14575))
    assert limits.max_velocity_mps == pytest.approx(0.7632)
    assert limits.max_acceleration_mps2 is None
    assert limits.upright_m == 0.0
    assert limits.namespace == '/wavemakers/ladertanken'
    assert limits.source == 'wavemakers.yaml'


@pytest.mark.parametrize('name', ['lilletanken', 'mclab'])
def test_linear_indradrive_placeholder_speed(name):
    # max_velocity_rpm 100 with the default factors (1 m per unit, 1/6): 100 x 6 x 1 = 600 m/s.
    # Absurd, but what their controllers enforce until their units are set (TODO item 19).
    limits = rig_limits.load_limits('wavemaker', name, WAVEMAKERS)
    assert (limits.min_m, limits.max_m) == (0.0, 0.5)
    assert limits.max_velocity_mps == pytest.approx(600.0)


def test_vertical():
    limits = rig_limits.load_limits('oscillator', 'vertical', OSCILLATOR)
    assert (limits.min_m, limits.max_m, limits.upright_m) == (-0.2, 0.2, 0.0)
    assert limits.max_velocity_mps == pytest.approx(0.3)  # 1800 / 6 x 0.001
    assert limits.max_acceleration_mps2 is None
    assert limits.namespace == '/oscillators/vertical'


def test_sloshing():
    limits = rig_limits.load_limits('oscillator', 'sloshing', OSCILLATOR)
    assert (limits.min_m, limits.max_m, limits.upright_m) == (0.015, 0.185, 0.1)
    assert limits.max_velocity_mps == 2.0          # LinMot default, not in the yaml
    assert limits.max_acceleration_mps2 == 9.81    # lower of 9.81 (goal) and 20 (drive)
    assert limits.namespace == '/oscillators/sloshing'
    assert limits.source == 'oscillator.yaml'


def test_unknown_name_lists_the_names():
    with pytest.raises(ValueError, match='ladertanken'):
        rig_limits.load_limits('wavemaker', 'nosuchtank', WAVEMAKERS)


def test_default_config_path_finds_the_yaml():
    assert pathlib.Path(rig_limits.default_config_path('wavemaker')).name == 'wavemakers.yaml'
    assert pathlib.Path(rig_limits.default_config_path('oscillator')).name == 'oscillator.yaml'


def package_not_installed(monkeypatch):
    """ROS sourced, but the bringup package not built (TODO item 42)."""
    packages = pytest.importorskip('ament_index_python.packages')

    def missing(package):
        raise packages.PackageNotFoundError(package)
    monkeypatch.setattr(packages, 'get_package_share_directory', missing)


def test_default_config_path_falls_back_to_the_repository(monkeypatch):
    package_not_installed(monkeypatch)
    path = pathlib.Path(rig_limits.default_config_path('wavemaker'))
    assert path.resolve() == pathlib.Path(WAVEMAKERS).resolve()


def test_default_config_path_installed_without_the_package(monkeypatch, tmp_path):
    """An installed copy has no repository next to it: say what to do, not a missing file."""
    package_not_installed(monkeypatch)
    installed = tmp_path / 'install' / 'rig_controller' / 'lib' / 'rig_controller'
    installed.mkdir(parents=True)
    monkeypatch.setattr(rig_limits, '__file__', str(installed / 'rig_limits.py'))
    with pytest.raises(FileNotFoundError, match='wavemaker_bringup is not installed.*--config'):
        rig_limits.default_config_path('wavemaker')


# ---------------------------------------------------------------- the rules

def test_linmot_without_goal_limit_uses_the_drives():
    params = dict(LINEAR, drive_name='linmot')
    limits = rig_limits.limits_from_parameters('oscillator', 't', params, 's')
    assert limits.max_acceleration_mps2 == 20.0
    assert limits.max_velocity_mps == 2.0


def test_linmot_speed_from_the_parameter():
    params = dict(LINEAR, drive_name='linmot', max_velocity_mps=1.5)
    limits = rig_limits.limits_from_parameters('oscillator', 't', params, 's')
    assert limits.max_velocity_mps == 1.5


def test_linear_indradrive_speed_from_the_drive_units():
    limits = rig_limits.limits_from_parameters('wavemaker', 't', LINEAR, 's')
    assert limits.max_velocity_mps == pytest.approx(0.1)


def test_linear_indradrive_speed_with_the_default_units():
    # As IndraDriveActuator::max_velocity_mps() with DriveScale's defaults: rpm x 6 x 1.
    params = {k: v for k, v in LINEAR.items()
              if k not in ('drive_m_per_unit', 'drive_velocity_per_unit_per_s')}
    limits = rig_limits.limits_from_parameters('wavemaker', 't', params, 's')
    assert limits.max_velocity_mps == pytest.approx(3600.0)


def test_goal_acceleration_limit_on_an_indradrive():
    params = dict(LINEAR, max_goal_acceleration_mps2=5.0)
    limits = rig_limits.limits_from_parameters('wavemaker', 't', params, 's')
    assert limits.max_acceleration_mps2 == 5.0


@pytest.mark.parametrize('key', ['drive_name', 'travel_minimum_m', 'travel_maximum_m',
                                 'max_velocity_rpm'])
def test_missing_key_is_named(key):
    params = {k: v for k, v in LINEAR.items() if k != key}
    with pytest.raises(ValueError, match=f'{key} not set in s'):
        rig_limits.limits_from_parameters('wavemaker', 't', params, 's')


@pytest.mark.parametrize('key', ['actuator_lead_m_per_degree', 'min_position_deg',
                                 'max_position_deg', 'max_velocity_rpm'])
def test_missing_angular_key_is_named(key):
    angular = {'drive_name': 'indradrive', 'drive_motion': 'angular',
               'actuator_lead_m_per_degree': 0.001, 'min_position_deg': -100.0,
               'max_position_deg': 100.0, 'max_velocity_rpm': 100.0}
    del angular[key]
    with pytest.raises(ValueError, match=key):
        rig_limits.limits_from_parameters('wavemaker', 't', angular, 's')


@pytest.mark.parametrize('change, message', [
    ({'drive_motion': 'angular', 'actuator_lead_m_per_degree': 0.0, 'min_position_deg': -1.0,
      'max_position_deg': 1.0, 'max_velocity_rpm': 1.0}, 'must be > 0'),
    ({'travel_minimum_m': 0.2}, 'not below'),
    ({'upright_position_m': 0.5}, 'outside'),
])
def test_configurations_the_controller_refuses(change, message):
    with pytest.raises(ValueError, match=message):
        rig_limits.limits_from_parameters('wavemaker', 't', dict(LINEAR, **change), 's')


# ---------------------------------------------------------------- compare

def test_parameters_and_yaml_give_the_same_limits():
    params = rig_limits.read_block('wavemaker', 'ladertanken', WAVEMAKERS)
    from_yaml = rig_limits.load_limits('wavemaker', 'ladertanken', WAVEMAKERS)
    from_params = rig_limits.limits_from_parameters('wavemaker', 'ladertanken', params,
                                                    'controller')
    assert rig_limits.compare(from_yaml, from_params) == []


def test_compare_reports_a_changed_limit():
    params = dict(rig_limits.read_block('wavemaker', 'ladertanken', WAVEMAKERS),
                  max_position_deg=250.0)
    live = rig_limits.limits_from_parameters('wavemaker', 'ladertanken', params, 'controller')
    lines = rig_limits.compare(rig_limits.load_limits('wavemaker', 'ladertanken', WAVEMAKERS),
                               live)
    assert lines == ['max_m: wavemakers.yaml +0.14575 m, controller +0.13250 m']


def test_compare_reports_a_changed_speed():
    params = dict(rig_limits.read_block('wavemaker', 'ladertanken', WAVEMAKERS),
                  max_velocity_rpm=200.0)
    live = rig_limits.limits_from_parameters('wavemaker', 'ladertanken', params, 'controller')
    lines = rig_limits.compare(rig_limits.load_limits('wavemaker', 'ladertanken', WAVEMAKERS),
                               live)
    assert len(lines) == 1 and lines[0].startswith('max_velocity_mps')


def test_compare_reports_an_acceleration_limit_on_one_side():
    a = rig_limits.limits_from_parameters('wavemaker', 't', LINEAR, 'yaml')
    b = rig_limits.limits_from_parameters(
        'wavemaker', 't', dict(LINEAR, max_goal_acceleration_mps2=9.81), 'controller')
    assert len(rig_limits.compare(a, b)) == 1


def test_compare_tolerates_rounding():
    a = rig_limits.limits_from_parameters('wavemaker', 't', LINEAR, 'yaml')
    b = rig_limits.limits_from_parameters(
        'wavemaker', 't', dict(LINEAR, travel_maximum_m=0.20005), 'controller')
    assert rig_limits.compare(a, b) == []  # 0.05 mm: below the 0.1 mm threshold
