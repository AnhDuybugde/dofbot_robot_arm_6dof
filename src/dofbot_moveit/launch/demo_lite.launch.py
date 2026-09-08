"""MoveIt demo with the low-poly Dofbot visual model."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
        .robot_description(file_path="config/dofbot_lite.urdf.xacro")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
