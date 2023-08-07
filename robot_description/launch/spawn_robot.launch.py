import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from rclpy.parameter import ParameterValue


def generate_launch_description():
    # Start Gazebo
    world_name = LaunchConfiguration('world_name', default='/home/akash/zal_ws/src/worlds/room/model.world')
    print(world_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world_name': world_name, 'use_sim_time': use_sim_time}.items(),
    )

    rviz_config = "/home/akash/zal_ws/src/robot_description/urdf/robot/robot_config.rviz"

    """ rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_config])
    ) """

    """ rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'RobotDescription': LaunchConfiguration('robot_description')}]
    ) """

    # Spawn the robot
    robot_sdf_path = os.path.join('/home/akash/zal_ws/src/robot_description', 'urdf', 'robot', 'model.sdf')

    world_sdf_path = '/home/akash/zal_ws/src/worlds/room/model.sdf'

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', robot_sdf_path])
    
    spawn_world_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_world', '-file', world_sdf_path])

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='/home/akash/zal_ws/src/worlds/room/model.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        spawn_robot_node,
        spawn_world_node
    ])

#/home/akash/building_editor_models/room/model.sdf

#/home/akash/zal_ws/src/worlds/room/model.world






















