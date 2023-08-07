from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_share_directory = get_package_share_directory('task')

    config = os.path.join(
        package_share_directory,
        'config',
        'goals.yaml'
        )

    return LaunchDescription([

        Node(
            package='task',    
            executable='controller_node.py',    
            name='controller_node',
            output='screen',
            parameters=[config]
            #arguments=['--goal_position', '10.0', '11.0', '--velocity_vector', '1.0', '0.0', '0.0', '0.0', '0.0', '1.0']  # Add your arguments here
        ),

        Node(
            package='task',    
            executable='robot_node.py', 
            name='robot_node',
            output='screen',
        )
    ])

