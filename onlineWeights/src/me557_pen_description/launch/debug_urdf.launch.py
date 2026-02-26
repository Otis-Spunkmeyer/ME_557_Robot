#!/usr/bin/env python3
"""Debug launch file to test URDF loading."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load URDF
    pkg_share_path = get_package_share_directory("me557_pen_description")
    urdf_path = os.path.join(pkg_share_path, "urdf", "me557_pen.urdf")
    
    print(f"Loading URDF from: {urdf_path}")
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
    print(f"URDF loaded, size: {len(robot_description_content)} bytes")
    
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
