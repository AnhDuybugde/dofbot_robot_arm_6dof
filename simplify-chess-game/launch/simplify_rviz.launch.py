"""RViz-only visualization for simplify_chess_game."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("dofbot_moveit")
    urdf = os.path.join(moveit_share, "config", "dofbot.urdf.xacro")
    description = {"robot_description": xacro.process_file(urdf).toxml()}
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", name="simplify_chess_rviz",
             parameters=[description], output="screen")
    ])
