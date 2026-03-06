#!/usr/bin/env python3
"""
SLAM Visualizer Launch - 3rd person nézet a robottal
Használat:
  Robot (Jetson): ros2 launch /path/to/slam_visualizer.launch.py mode:=robot
  Laptop (GUI):   ros2 launch /path/to/slam_visualizer.launch.py mode:=laptop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full',
        description='Mode: robot (only state publisher), laptop (only rviz), full (both)'
    )
    
    # Paths
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(pkg_path, 'robot_description', 'urdf', 'rescue_robot.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'slam_3rd_person.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    mode = LaunchConfiguration('mode')
    
    # Robot state publisher - runs on robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'robot' or '", mode, "' == 'full'"])
        )
    )
    
    # RViz2 - runs on laptop or full mode
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'laptop' or '", mode, "' == 'full'"])
        )
    )
    
    return LaunchDescription([
        mode_arg,
        robot_state_publisher,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
