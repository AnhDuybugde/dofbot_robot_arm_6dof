from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    cfg=MoveItConfigsBuilder(
        "dofbot",
        package_name="dofbot_moveit"
    ).robot_description(
        file_path="config/dofbot.urdf.xacro"
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package="dofbot_tea_moveit",
            executable="smoke",
            output="screen",
            parameters=[cfg.to_dict()]
        )
    ])
