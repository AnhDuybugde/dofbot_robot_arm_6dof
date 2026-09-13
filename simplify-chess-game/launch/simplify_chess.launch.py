"""Single-terminal bringup for simplify_chess_game: backend + RViz + board.

Controller-only backend (no MoveIt/move_group) together with the RViz view
and the static 64-square + 32-piece visualization. Launch this ONCE; starting
it twice duplicates robot_state_publisher/controller_manager under identical
node names and freezes the RViz robot while joints keep moving.

Arguments:
  use_lite_model: true -> 14-box proxy URDF instead of the ~104MB STL meshes.
    Same joints/links, much lighter rendering on weak GPUs.
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context):
    use_lite = LaunchConfiguration("use_lite_model").perform(context).lower() == "true"
    xacro_file = "dofbot_lite.urdf.xacro" if use_lite else "dofbot.urdf.xacro"
    moveit_share = get_package_share_directory("dofbot_moveit")
    simplify_share = get_package_share_directory("simplify_chess_game")
    urdf = os.path.join(moveit_share, "config", xacro_file)
    controllers = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    description = {"robot_description": xacro.process_file(urdf).toxml()}
    with open(os.path.join(moveit_share, "config", "dofbot.srdf"), encoding="utf-8") as stream:
        rviz_description = dict(description)
        rviz_description["robot_description_semantic"] = stream.read()
    return [
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="chess_world_to_base", arguments=["0", "0", "0", "0", "0", "0",
                                                    "world", "base_link"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="chess_robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[description, controllers]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_group_controller", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["grip_group_controller", "--controller-manager", "/controller_manager"]),
        Node(package="rviz2", executable="rviz2", name="simplify_chess_rviz",
             arguments=["-d", os.path.join(simplify_share, "config", "simplify_chess.rviz")],
             parameters=[rviz_description], output="screen"),
        Node(package="simplify_chess_game", executable="chess_board_viz",
             name="simplify_chess_board_visualizer", output="screen"),
        Node(package="simplify_chess_game", executable="arm_skeleton_viz",
             name="simplify_arm_skeleton", output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_lite_model", default_value="false",
            description="true: 14-box proxy URDF for weak GPUs (same joints/links)"),
        OpaqueFunction(function=_make_nodes),
    ])
