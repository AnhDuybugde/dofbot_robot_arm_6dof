import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    cfg=MoveItConfigsBuilder(
        "dofbot",
        package_name="dofbot_moveit"
    ).robot_description(
        file_path="config/dofbot.urdf.xacro"
    ).to_moveit_configs()

    tea_share=get_package_share_directory(
        "dofbot_tea_moveit"
    )

    srdf_path=os.path.join(
        tea_share,
        "config",
        "dofbot_tea.srdf"
    )

    task_yaml=os.path.join(
        tea_share,
        "config",
        "tea_task.yaml"
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

    task=Node(
        package="dofbot_tea_moveit",
        executable="tea_task",
        name="tea_moveit_app",
        parameters=[
            cfg.to_dict(),
            semantic,
            task_yaml
        ],
        output="screen",
        ros_arguments=[
            "--log-level","WARN",
            "--log-level",
            "tea_moveit_app:=INFO"
        ]
    )

    return LaunchDescription([
        task
    ])
