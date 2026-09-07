import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    cfg=MoveItConfigsBuilder(
        "dofbot",
        package_name="dofbot_moveit"
    ).to_moveit_configs()

    tea_share=get_package_share_directory(
        "dofbot_tea_moveit"
    )

    moveit_share=get_package_share_directory(
        "dofbot_moveit"
    )

    srdf_path=os.path.join(
        tea_share,
        "config",
        "dofbot_tea.srdf"
    )

    rviz_config=os.path.join(
        moveit_share,
        "config",
        "moveit.rviz"
    )

    with open(
        srdf_path,
        "r",
        encoding="utf-8"
    ) as f:
        semantic={
            "robot_description_semantic":
                f.read()
        }

    rviz=Node(
        package="rviz2",
        executable="rviz2",
        name="tea_rviz",
        arguments=[
            "-d",
            rviz_config
        ],
        parameters=[
            cfg.to_dict(),
            semantic
        ],
        output="screen"
    )

    return LaunchDescription([
        rviz
    ])
