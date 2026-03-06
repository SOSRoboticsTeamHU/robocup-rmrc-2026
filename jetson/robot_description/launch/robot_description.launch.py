"""
ROS2 Launch file for robot_state_publisher.
Publishes the robot URDF to /robot_description and TF transforms.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(urdf_dir, 'urdf', 'rescue_robot.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
    ])
