#!/usr/bin/env python3
"""
Complete SLAM launch with robot model and proper RViz config.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(script_dir, "robot_description", "urdf", "rescue_robot.urdf")
    rviz_config = os.path.join(script_dir, "rviz", "slam_with_robot.rviz")
    pkg_point_lio = get_package_share_directory('point_lio')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )
    
    # Point-LIO SLAM
    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_point_lio, 'launch', 'mapping_unilidar_l2.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )
    
    # RViz (delayed to let TF settle)
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_pub,
        point_lio,
        rviz,
    ])
