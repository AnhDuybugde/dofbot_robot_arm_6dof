"""Demo MoveIt dofbot (branch exp/arm5-fixed-pipeline).

KHONG dung generate_demo_launch: helper do include rsp/move_group bang file
rieng (tu build config default) nen override model khong bao gio toi move_group.
File nay build MoveItConfigs 1 lan (theo arg `model`) roi goi truc tiep cac
generate_*_launch(moveit_config).
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build(context):
    model = LaunchConfiguration("model").perform(context)
    builder = MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
    if model == "dofbot_fixed":
        # Planning model arm5-fixed tai q5=0 (arm_fixed = arm1-4).
        # File sinh boi tools/generate_fixed_model.py.
        builder = builder.robot_description(
            file_path="config/dofbot_fixed.urdf"
        ).robot_description_semantic(file_path="config/dofbot_fixed.srdf")
    mc = builder.to_moveit_configs()
    share = FindPackageShare("dofbot_moveit")
    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "static_virtual_joint_tfs.launch.py"]))))
    for gen in (generate_rsp_launch, generate_move_group_launch):
        sub = gen(mc)
        for entity in list(sub.entities):
            ld.add_action(entity)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "moveit_rviz.launch.py"])),
        condition=IfCondition(LaunchConfiguration("use_rviz"))))
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            mc.robot_description,
            str(mc.package_path / "config/ros2_controllers.yaml"),
        ],
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "spawn_controllers.launch.py"]))))
    return list(ld.entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model", default_value="dofbot",
            description="dofbot (5 joints) | dofbot_fixed (arm5 fixed tai q5=0)"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_build),
    ])
