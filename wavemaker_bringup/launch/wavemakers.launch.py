"""
Launch one wavemaker controller, optionally with the bridge and the Nav2 lifecycle manager.

Terminal use (experienced users): the controller only, lifecycle stepped by hand.
    ros2 launch wavemaker_bringup wavemakers.launch.py wavemaker:=ladertanken
Each goal picks its type: height and period for a regular wave, positions for a pregenerated
one (e.g. play_dat), so both work without a relaunch.

Bridge use (normal users): use bridge.launch.py, which includes this file with
with_bridge:=true and the lifecycle manager on.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import LifecycleNode, Node

WAVEMAKERS = ['ladertanken', 'lilletanken', 'mclab']


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _launch_setup(context):
    config_file = get_package_share_directory('wavemaker_bringup') + '/config/wavemakers.yaml'
    wavemaker = context.launch_configurations['wavemaker']
    with_bridge = _is_true(context, 'with_bridge')
    enable_lifecycle_manager = _is_true(context, 'enable_lifecycle_manager')
    bond_timeout = float(context.launch_configurations['bond_timeout'])
    water_depth = context.launch_configurations['water_depth']
    transfer_gain = context.launch_configurations['wavemaker_transfer_gain']
    # Values given at launch override the ones in wavemakers.yaml; empty keeps the yaml value.
    overrides = {}
    if water_depth:
        overrides['water_depth'] = float(water_depth)
    if transfer_gain:
        overrides['wavemaker_transfer_gain'] = float(transfer_gain)
    namespace = f'wavemakers/{wavemaker}'

    actions = [
        LifecycleNode(
            package='rig_controller',
            executable='controller_node',
            namespace=namespace,
            name='controller',
            parameters=[config_file, {'bond_timeout': bond_timeout}, overrides],
            output='screen',
        )
    ]
    managed_nodes = ['controller']

    if with_bridge:
        actions.append(
            LifecycleNode(
                package='rig_controller',
                executable='bridge_node',
                namespace=namespace,
                name='bridge',
                parameters=[{'bond_timeout': bond_timeout}],
                output='screen',
            )
        )
        managed_nodes.append('bridge')

    if enable_lifecycle_manager:
        # The manager configures and activates the nodes in order and, if any of them stops
        # sending bond heartbeats for bond_timeout seconds, brings all of them down. With the
        # bridge this is what stops the paddle when the bridge crashes.
        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': managed_nodes,
                    'bond_timeout': bond_timeout,
                    # After a fault the nodes stay down. Nav2 would otherwise bring them back up
                    # within seconds (respawn reconnection), switching the drive on again without
                    # anyone asking. Restart the service instead.
                    'attempt_respawn_reconnection': False,
                }],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wavemaker',
            default_value='ladertanken',
            choices=WAVEMAKERS,
            description='Wavemaker to launch',
        ),
        DeclareLaunchArgument(
            'with_bridge',
            default_value='false',
            choices=['true', 'false'],
            description='Also start bridge_node (normally via bridge.launch.py)',
        ),
        DeclareLaunchArgument(
            'enable_lifecycle_manager',
            default_value='false',
            choices=['true', 'false'],
            description='Start the Nav2 lifecycle manager to configure and activate the nodes',
        ),
        DeclareLaunchArgument(
            'bond_timeout',
            default_value='4.0',
            description='Seconds without a bond heartbeat before the lifecycle manager brings '
                        'the nodes down',
        ),
        DeclareLaunchArgument(
            'water_depth',
            default_value='',
            description='Water depth in m for this run; empty uses the value in wavemakers.yaml',
        ),
        DeclareLaunchArgument(
            'wavemaker_transfer_gain',
            default_value='',
            description='Measured / target wave height for regular waves; empty uses the value '
                        'in wavemakers.yaml',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
