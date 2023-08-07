import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    spawn_pkg_dir = get_package_share_directory('robot_description')
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_pkg_dir + '/launch/spawn_robot.launch.py'),
    )

    move_pkg_dir = get_package_share_directory('task')
    move_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_pkg_dir + '/launch/move_robot.launch.py'),
    )

    
    return LaunchDescription([
        spawn_launch,
        move_launch
    ])























