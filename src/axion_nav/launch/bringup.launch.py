"""
一期：与 mock.launch 相同（定位 + 直线去目标）。
后续将在此接入 map_server + AMCL + Nav2 bringup。
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = PathJoinSubstitution([
        FindPackageShare('axion_nav'),
        'launch',
        'mock.launch.py',
    ])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mock)),
    ])
