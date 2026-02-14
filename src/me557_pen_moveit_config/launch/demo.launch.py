from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("me557_pen_moveit_config")
    moveit_config = MoveItConfigsBuilder(
        "me557_pen", package_name="me557_pen_moveit_config"
    ).planning_pipelines(pipelines=["ompl"], load_all=False).to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg}/launch/static_virtual_joint_tfs.launch.py")
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg}/launch/rsp.launch.py")
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                f"{pkg}/config/ros2_controllers.yaml",
            ],
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["write_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg}/launch/move_group.launch.py"),
            launch_arguments={"allow_trajectory_execution": "true"}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg}/launch/moveit_rviz.launch.py"),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return ld
