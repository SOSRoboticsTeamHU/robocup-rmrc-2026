import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(launch_dir)
    urdf_path = os.path.join(pkg_dir, "urdf", "rescue_robot.urdf")
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        )
    ])
