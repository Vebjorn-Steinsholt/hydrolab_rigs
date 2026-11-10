#!/usr/bin/env python3
"""
Goal limits of a rig, from its yaml block or its live parameters, derived as the controller does.

Shared by play_dat and the generators (generate_waves, generate_oscillations) so they cannot
disagree. The rules are the controller's: position limits in controller_node.cpp on_configure
(~212-244, through DriveScale), speed and acceleration from the actuator (indradrive_actuator.cpp,
linmot_actuator.cpp) with the drive's units from drive_scale.hpp, and max_goal_acceleration_mps2.
"""

from dataclasses import dataclass
import os

import yaml

# rig -> (namespace prefix, bringup package, yaml file)

RIGS = {
    'wavemaker': ('wavemakers', 'wavemaker_bringup', 'wavemakers.yaml'),
    'oscillator': ('oscillators', 'oscillator_bringup', 'oscillator.yaml'),
}

LINMOT_MAX_VELOCITY_MPS = 2.0  # linmot_actuator.cpp default
LINMOT_MAX_ACCELERATION_MPS2 = 20.0


@dataclass
class RigLimits:
    rig: str
    name: str
    namespace: str
    upright_m: float
    min_m: float
    max_m: float
    max_velocity_mps: float
    max_acceleration_mps2: float | None
    source: str


def required(params: dict, key: str, name: str, source: str):
    """Return a key with no safe default: the controller's default would give wrong limits."""
    if key not in params:
        raise ValueError(f'{name}: {key} not set in {source}')
    return params[key]


def default_config_path(rig: str) -> str:
    """Return the installed yaml when the workspace is sourced, else the repository's copy."""
    _, package, filename = RIGS[rig]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
    except (ImportError, LookupError):  # ROS not sourced, or the package not installed
        share = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', package)
        if not os.path.isdir(share):  # an installed copy: no repository next to it
            raise FileNotFoundError(
                f'{package} is not installed, so there is no {filename}: build it '
                f'(colcon build --packages-up-to {package}) or give --config') from None
    return os.path.join(share, 'config', filename)


def read_block(rig: str, name: str, config_path: str) -> dict:
    """ros__parameters of /<prefix>/<name>/controller; lists the names in the file if missing."""
    prefix = RIGS[rig][0]
    with open(config_path) as f:
        config = yaml.safe_load(f) or {}
    key = f'/{prefix}/{name}/controller'
    if key not in config:
        names = [k.split('/')[2] for k in config if k.startswith(f'/{prefix}/')]
        raise ValueError(f"no {rig} '{name}' in {config_path}; have {', '.join(names) or 'none'}")
    return config[key]['ros__parameters']


def limits_from_parameters(rig: str, name: str, params: dict, source: str) -> RigLimits:
    """Apply the controller's rules to a yaml block or to parameters read from the controller."""
    drive = required(params, 'drive_name', name, source)
    upright_m = params.get('upright_position_m', 0.0)
    if params.get('drive_motion', 'linear') == 'angular':
        lead = required(params, 'actuator_lead_m_per_degree', name, source)
        if lead <= 0.0:
            raise ValueError(f'{name}: actuator_lead_m_per_degree must be > 0 in {source}')
        upright_deg = params.get('actuator_upright_angle_deg', 0.0)
        min_deg = required(params, 'min_position_deg', name, source)
        max_deg = required(params, 'max_position_deg', name, source)
        min_m = upright_m + (min_deg - upright_deg) * lead
        max_m = upright_m + (max_deg - upright_deg) * lead
        max_velocity = required(params, 'max_velocity_rpm', name, source) * 6.0 * lead
    else:
        min_m = required(params, 'travel_minimum_m', name, source)
        max_m = required(params, 'travel_maximum_m', name, source)
        if drive == 'linmot':
            max_velocity = params.get('max_velocity_mps', LINMOT_MAX_VELOCITY_MPS)
        else:  # indradrive: as IndraDriveActuator::max_velocity_mps(), defaults as DriveScale's
            max_velocity = (required(params, 'max_velocity_rpm', name, source)
                            / params.get('drive_velocity_per_unit_per_s', 1 / 6)
                            * params.get('drive_m_per_unit', 1.0))
    goal_acceleration = params.get('max_goal_acceleration_mps2', 0.0)
    acceleration = (goal_acceleration if goal_acceleration > 0.0 else None)
    if drive == 'linmot':
        drive_acceleration = params.get('max_acceleration_mps2', LINMOT_MAX_ACCELERATION_MPS2)
        acceleration = (drive_acceleration if acceleration is None
                        else min(acceleration, drive_acceleration))

    if not min_m < max_m:
        raise ValueError(f'{name}: lower limit {min_m} not below upper limit {max_m} in {source}')
    if not min_m <= upright_m <= max_m:
        raise ValueError(f'{name}: upright {upright_m} outside {min_m}..{max_m} in {source}')
    return RigLimits(rig, name, f'/{RIGS[rig][0]}/{name}', upright_m, min_m, max_m,
                     max_velocity, acceleration, source)


def load_limits(rig: str, name: str, config_path: str | None = None) -> RigLimits:
    """Load the limits of one rig from its yaml block."""
    config_path = config_path or default_config_path(rig)
    return limits_from_parameters(rig, name, read_block(rig, name, config_path),
                                  os.path.basename(config_path))


def compare(expected: RigLimits, actual: RigLimits) -> list[str]:
    """Differences worth a warning: positions beyond 0.1 mm, speed and acceleration beyond 1e-3."""
    lines = []
    for field in ('upright_m', 'min_m', 'max_m'):
        e, a = getattr(expected, field), getattr(actual, field)
        if abs(e - a) > 1e-4:
            lines.append(f'{field}: {expected.source} {e:+.5f} m, {actual.source} {a:+.5f} m')
    e, a = expected.max_velocity_mps, actual.max_velocity_mps
    if abs(e - a) > 1e-3:
        lines.append(f'max_velocity_mps: {expected.source} {e:.4f}, {actual.source} {a:.4f}')
    e, a = expected.max_acceleration_mps2, actual.max_acceleration_mps2
    if (e is None) != (a is None) or (e is not None and a is not None and abs(e - a) > 1e-3):
        lines.append(f'max_acceleration_mps2: {expected.source} {e}, {actual.source} {a}')
    return lines
