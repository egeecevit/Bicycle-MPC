from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ( 
    Command, 
    PathJoinSubstitution
)
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('bicycle_description'),'urdf','robot.urdf.xacro')
    
    robot_description = Command(['xacro ', urdf_path])

    world_config = LaunchConfiguration("world")

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="empty.world",
        description='world to launch bicycle in'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": LaunchConfiguration('use_sim_time')}
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bicycle', 
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
            "/gazebo.launch.py",
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                    FindPackageShare('bicycle_gazebo'),
                    'worlds',
                    world_config]),
            "verbose": "true",
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])