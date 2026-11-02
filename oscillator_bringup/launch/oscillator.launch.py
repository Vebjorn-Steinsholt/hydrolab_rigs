"""
Everything the sloshing oscillator runs.

Controller, bridge, lifecycle manager, soft stop and web page. Started on the Pi, normally
through the oscillator service (see rpi/README.md):

    ros2 launch oscillator_bringup oscillator.launch.py port:=8080

The lifecycle manager configures and activates the controller at start, which homes the drive
if it is not homed (about 20 s, the slide moves to its home stop). Ctrl-C (SIGINT) stops
everything and the controller switches the drive off.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

NAMESPACE = 'oscillators/sloshing'


def generate_launch_description():
    config_file = get_package_share_directory('oscillator_bringup') + '/config/oscillator.yaml'
    web_launch = get_package_share_directory('rig_web') + '/launch/web.launch.py'
    bond_timeout = ParameterValue(LaunchConfiguration('bond_timeout'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080', description='Web page port'),
        DeclareLaunchArgument(
            'soft_stop_pin', default_value='17', description='GPIO pin for soft stop button'),
        DeclareLaunchArgument(
            'idle_timeout_s', default_value='7200',
            description='Stop everything after this long without a wave; 0 turns it off'),
        DeclareLaunchArgument(
            'bond_timeout', default_value='4.0',
            description='Seconds without a bond heartbeat before the lifecycle manager brings '
                        'the nodes down'),

        LifecycleNode(
            package='rig_controller',
            executable='controller_node',
            namespace=NAMESPACE,
            name='controller',
            parameters=[config_file, {'bond_timeout': bond_timeout}],
            output='screen',
        ),
        LifecycleNode(
            package='rig_controller',
            executable='bridge_node',
            namespace=NAMESPACE,
            name='bridge',
            parameters=[config_file, {'bond_timeout': bond_timeout}],
            output='screen',
        ),
        # Configures and activates both in order; if either stops its bond heartbeats for
        # bond_timeout seconds, brings both down, which switches the drive off.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=NAMESPACE,
            name='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['controller', 'bridge'],
                'bond_timeout': bond_timeout,
                # After a fault the nodes stay down. Nav2 would otherwise bring them back up
                # within seconds (respawn reconnection), switching the drive on again without
                # anyone asking. Restart the service instead.
                'attempt_respawn_reconnection': False,
            }],
            output='screen',
        ),
        Node(
            package='rig_controller',
            executable='soft_stop',
            namespace=NAMESPACE,
            name='soft_stop',
            parameters=[{
                'pin': ParameterValue(LaunchConfiguration('soft_stop_pin'), value_type=int),
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch),
            launch_arguments={
                'rig': 'oscillator',
                'port': LaunchConfiguration('port'),
                'idle_timeout_s': LaunchConfiguration('idle_timeout_s'),
            }.items(),
        ),
    ])
