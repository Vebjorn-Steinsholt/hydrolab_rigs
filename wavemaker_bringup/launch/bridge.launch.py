"""
Bridge mode: controller + bridge, started and supervised by the Nav2 lifecycle manager.

For normal users, who control the wavemaker through the bridge's height, period and stop
topics. The lifecycle manager configures and activates both nodes automatically. If the bridge
(or the controller) stops sending bond heartbeats for bond_timeout seconds, the manager brings
both down, and deactivating the controller disables the drive, so the paddle stops.

    ros2 launch wavemaker_bringup bridge.launch.py wavemaker:=ladertanken
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_launch = (
        get_package_share_directory('wavemaker_bringup') + '/launch/wavemakers.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'wavemaker',
            default_value='ladertanken',
            choices=['ladertanken', 'lilletanken', 'mclab'],
            description='Wavemaker to launch',
        ),
        DeclareLaunchArgument(
            'bond_timeout',
            default_value='1.0',
            description='Seconds without a bond heartbeat before everything is brought down; '
                        'this is how long the paddle can keep moving after the bridge dies',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'wavemaker': LaunchConfiguration('wavemaker'),
                'with_bridge': 'true',
                'enable_lifecycle_manager': 'true',
                'bond_timeout': LaunchConfiguration('bond_timeout'),
                'water_depth': LaunchConfiguration('water_depth'),
                'wavemaker_transfer_gain': LaunchConfiguration('wavemaker_transfer_gain'),
            }.items(),
        ),
    ])
