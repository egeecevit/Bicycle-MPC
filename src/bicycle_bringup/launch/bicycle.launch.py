import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    default_rviz_config_path = os.path.join(get_package_share_directory('bicycle_bringup'), 'config', 'bicycle.rviz')


    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    bicycle_model_node = Node(
        package='bicycle_model',
        executable='bicycle_model',
        name='bicycle_model',
        output='screen',
    )

    controller_node = Node(
        package='bicycle_model',
        executable='controller',
        name='controller',
        output='screen',
    )

    path_publisher_node = Node(
        package='bicycle_model',
        executable='path_publisher',
        name='path_publisher',
        output='screen',
    )

    map_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
        name='map_publisher'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz2_node,
        map_publisher_node,
        path_publisher_node,
        bicycle_model_node,
        controller_node,
    ])