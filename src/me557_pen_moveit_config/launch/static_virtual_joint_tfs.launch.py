from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "me557_pen", package_name="me557_pen_moveit_config"
    ).planning_pipelines(pipelines=["ompl"], load_all=False).to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
