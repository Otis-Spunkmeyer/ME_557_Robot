#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue
import os
import re
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    pkg_share = FindPackageShare("me557_pen_description")

    default_params = PathJoinSubstitution([pkg_share, "config", "me557_pen_params.yaml"])
    default_rviz = PathJoinSubstitution([pkg_share, "config", "me557_pen_rviz.rviz"])

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the ROS 2 parameters file to use",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to the RViz configuration file to use",
    )

    # Load URDF file directly (handles filename with spaces) from installed package.
    # Fail fast instead of silently publishing empty robot_description.
    pkg_share_path = get_package_share_directory("me557_pen_description")
    urdf_path = os.path.join(pkg_share_path, "urdf", "me557_pen.urdf")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    mesh_paths = re.findall(
        r'filename\\s*=\\s*"package://me557_pen_description/([^"]+)"',
        robot_description_content,
    )
    missing_meshes = [
        rel_path
        for rel_path in mesh_paths
        if not os.path.exists(os.path.join(pkg_share_path, rel_path))
    ]
    if missing_meshes:
        raise FileNotFoundError(
            "Missing meshes in installed package share: " + ", ".join(sorted(set(missing_meshes)))
        )

    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            params_file,
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, params_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_rviz_config,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
