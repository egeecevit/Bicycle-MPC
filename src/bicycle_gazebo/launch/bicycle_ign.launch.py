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

    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--gui-config', '--render-engine', 'ogre2', '-r'],
        output='screen'
    )

    ign_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-file', urdf_path,
            '-name', 'bicycle',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        ign_spawn
    ])