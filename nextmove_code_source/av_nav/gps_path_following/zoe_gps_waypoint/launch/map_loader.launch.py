from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Chemin absolu vers le fichier YAML listant les paths à charger.',
    )

    map_loader_node = Node(
        package='zoe_gps_waypoint',
        executable='waypoint_map_loader',  
        name='waypoint_map_loader',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        map_loader_node,
    ])