"""RViz-only visualization for simplify_chess_game."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("dofbot_moveit")
    simplify_share = get_package_share_directory("simplify_chess_game")
    urdf = os.path.join(moveit_share, "config", "dofbot.urdf.xacro")
    description = {"robot_description": xacro.process_file(urdf).toxml()}
    with open(os.path.join(moveit_share, "config", "dofbot.srdf"), encoding="utf-8") as stream:
        description["robot_description_semantic"] = stream.read()
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", name="simplify_chess_rviz",
             arguments=["-d", os.path.join(simplify_share, "config", "simplify_chess.rviz")],
             parameters=[description], output="screen")
        ,Node(package="simplify_chess_game", executable="chess_board_viz",
              name="simplify_chess_board_visualizer", output="screen")
        ,Node(package="simplify_chess_game", executable="arm_skeleton_viz",
              name="simplify_arm_skeleton", output="screen")
    ])
