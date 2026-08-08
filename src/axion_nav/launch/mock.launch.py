from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fastdds_xml = PathJoinSubstitution([
        FindPackageShare('axion_nav'),
        'config',
        'fastdds_no_shm.xml',
    ])
    params_file = PathJoinSubstitution([
        FindPackageShare('axion_nav'),
        'config',
        'mock_nav.yaml',
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_xml),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='mock_nav parameters YAML',
        ),
        Node(
            package='axion_nav',
            executable='mock_nav_node',
            name='mock_nav',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
