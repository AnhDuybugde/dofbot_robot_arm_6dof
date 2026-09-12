"""RViz-only visualization for simplify_chess_game."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("dofbot_moveit")
    chess_share = get_package_share_directory("chess_moveit_demo")
    urdf = os.path.join(moveit_share, "config", "dofbot.urdf.xacro")
    description = {"robot_description": xacro.process_file(urdf).toxml()}
    with open(os.path.join(moveit_share, "config", "dofbot.srdf"), encoding="utf-8") as stream:
        description["robot_description_semantic"] = stream.read()
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", name="simplify_chess_rviz",
             arguments=["-d", os.path.join(chess_share, "config", "chess_lite.rviz")],
             parameters=[description], output="screen")
    ])
