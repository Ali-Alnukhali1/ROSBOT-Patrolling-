from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('rosbot_patrolling')
    yaml_file_path = os.path.join(package_share_dir,'config','waypoints.yaml')


    return LaunchDescription([
        Node(
            package='rosbot_patrolling',
            executable='patrol_node',
            name='patrolling_action_client',
            output='screen',
            parameters=[yaml_file_path]
        )
    ])