"""
Web page for one rig, served by rig_web's web_server.

The page talks to bridge_node, so start the bridge (bridge.launch.py) as well.
After idle_timeout_s without a wave the server stops the managed nodes and exits, and this launch
(or lab.launch.py, which includes it) shuts down with it. A crash of the server does the same.

    ros2 launch rig_web web.launch.py wavemaker:=ladertanken port:=8080
    ros2 launch rig_web web.launch.py rig:=oscillator port:=8080

The page runs in wavemakers/<wavemaker> for rig:=wavemaker and in oscillators/sloshing for
rig:=oscillator, unless namespace is given.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch_ros.actions import Node

WAVEMAKERS = ['ladertanken', 'lilletanken', 'mclab']
OSCILLATOR_NAMESPACE = 'oscillators/sloshing'


def _shutdown_unless_stopping(event, context):
    # The server also exits during an ordinary shutdown (Ctrl-C, systemctl stop). A second
    # Shutdown then makes launch fail with "Cannot shutdown a ROS adapter that is not running".
    if context.is_shutdown:
        return None
    return [Shutdown(reason='web server exited')]


def _namespace(context):
    namespace = context.launch_configurations['namespace']
    if namespace:
        return namespace
    if context.launch_configurations['rig'] == 'oscillator':
        return OSCILLATOR_NAMESPACE
    wavemaker = context.launch_configurations['wavemaker']
    if wavemaker not in WAVEMAKERS:
        raise RuntimeError(
            f"web.launch.py: wavemaker:=<{'|'.join(WAVEMAKERS)}> is required for "
            f"rig:=wavemaker unless namespace is given (got '{wavemaker}')")
    return f'wavemakers/{wavemaker}'


def _launch_setup(context):
    configurations = context.launch_configurations
    return [
        Node(
            package='rig_web',
            executable='web_server',
            namespace=_namespace(context),
            name='web',
            output='screen',
            parameters=[{
                'port': int(configurations['port']),
                'idle_timeout_s': float(configurations['idle_timeout_s']),
                'rig': configurations['rig'],
            }],
            on_exit=_shutdown_unless_stopping,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wavemaker',
            default_value='',
            description=f"Wavemaker the page controls ({', '.join(WAVEMAKERS)}); required for "
                        'rig:=wavemaker unless namespace is given',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of the page and the bridge it talks to; empty: '
                        f'wavemakers/<wavemaker>, or {OSCILLATOR_NAMESPACE} for rig:=oscillator',
        ),
        DeclareLaunchArgument('port', default_value='8080', description='HTTP port'),
        DeclareLaunchArgument(
            'rig', default_value='wavemaker', choices=['wavemaker', 'oscillator'],
            description='Texts on the page: wavemaker or oscillator'),
        DeclareLaunchArgument(
            'idle_timeout_s', default_value='7200',
            description='Stop the rig after this long without a wave; 0 turns it off'),
        OpaqueFunction(function=_launch_setup),
    ])
