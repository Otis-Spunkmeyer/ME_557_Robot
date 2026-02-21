from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    start_moveit_demo = LaunchConfiguration("start_moveit_demo")
    params_file = LaunchConfiguration("params_file")
    writer_delay_sec = LaunchConfiguration("writer_delay_sec")
    plan_only_capture = LaunchConfiguration("plan_only_capture")
    trajectory_header_output = LaunchConfiguration("trajectory_header_output")

    moveit_pkg = get_package_share_directory("me557_pen_moveit_config")
    default_params = PathJoinSubstitution(
        [FindPackageShare("me557_pen_moveit_config"), "config", "write_on_board.yaml"]
    )

    write_on_board_node = Node(
        package="me557_pen_description",
        executable="write_on_board",
        name="board_writer",
        output="screen",
        parameters=[
            params_file,
            {
                "plan_only_capture": plan_only_capture,
                "trajectory_header_output": trajectory_header_output,
            },
        ],
        on_exit=[Shutdown(reason="write_on_board completed")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz in MoveIt demo stack.",
            ),
            DeclareLaunchArgument(
                "start_moveit_demo",
                default_value="true",
                description="Start me557_pen_moveit_config demo.launch.py before write_on_board.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="write_on_board ROS parameters file.",
            ),
            DeclareLaunchArgument(
                "writer_delay_sec",
                default_value="2.0",
                description="Delay before starting write_on_board to allow MoveIt startup.",
            ),
            DeclareLaunchArgument(
                "plan_only_capture",
                default_value="false",
                description="If true, capture planned trajectory without execution.",
            ),
            DeclareLaunchArgument(
                "trajectory_header_output",
                default_value="/home/rosubu/ME_557_Robot/online/robotMovements/trajectory_data.h",
                description="Output header path written by write_on_board.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{moveit_pkg}/launch/demo.launch.py"),
                condition=IfCondition(start_moveit_demo),
                launch_arguments={"use_rviz": use_rviz}.items(),
            ),
            TimerAction(period=writer_delay_sec, actions=[write_on_board_node]),
        ]
    )
