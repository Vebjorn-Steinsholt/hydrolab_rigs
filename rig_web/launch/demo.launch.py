"""
Try the web page without hardware: fake controller, bridge, lifecycle manager and web page.

Everything runs on its own ROS domain (default 77) and only on this computer, so the demo can
never reach LabVIEW, another computer or the real controller. The web server prints the page's
address, http://<hostname>.local:8080/ (for example http://ladertanken-rpi.local:8080/).

    ros2 launch rig_web demo.launch.py
    ros2 launch rig_web demo.launch.py rig:=oscillator port:=8081

To try the idle auto-stop: idle_timeout_s:=120 (warning shown for the last 10 minutes, so at once).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _shutdown_unless_stopping(event, context):
    # The server also exits during an ordinary shutdown (Ctrl-C, systemctl stop). A second
    # Shutdown then makes launch fail with "Cannot shutdown a ROS adapter that is not running".
    if context.is_shutdown:
        return None
    return [Shutdown(reason='web server exited')]


def generate_launch_description():
    port = LaunchConfiguration('port')
    namespace = LaunchConfiguration('namespace')
    rig = LaunchConfiguration('rig')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080', description='HTTP port'),
        DeclareLaunchArgument(
            'rig', default_value='wavemaker', choices=['wavemaker', 'oscillator'],
            description="Rig: the page's texts, and the action and topic names of all nodes"),
        DeclareLaunchArgument(
            'namespace',
            default_value=PythonExpression([
                "'oscillators/sloshing' if '", LaunchConfiguration('rig'),
                "' == 'oscillator' else 'wavemakers/ladertanken'"]),
            description='Namespace of the demo nodes; default by rig'),
        DeclareLaunchArgument(
            'idle_timeout_s', default_value='7200',
            description='Stop after this long without a wave; 0 turns it off'),
        DeclareLaunchArgument(
            'domain_id', default_value='77',
            description='ROS domain for the demo; keep it different from the lab domain'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(
            package='rig_controller', executable='fake_controller',
            namespace=namespace, name='controller', output='screen',
            arguments=['--rig', rig]),
        Node(
            package='rig_controller', executable='bridge_node',
            namespace=namespace, name='bridge', output='screen',
            parameters=[{'bond_timeout': 1.0, 'rig': rig}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            namespace=namespace, name='lifecycle_manager', output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['bridge'],
                'bond_timeout': 1.0,
            }]),
        Node(
            package='rig_web', executable='web_server',
            namespace=namespace, name='web', output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=int),
                'idle_timeout_s': ParameterValue(
                    LaunchConfiguration('idle_timeout_s'), value_type=float),
                'rig': rig,
            }],
            on_exit=_shutdown_unless_stopping),
    ])
