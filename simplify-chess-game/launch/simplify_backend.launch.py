"""Controller-only backend for simplify_chess_game (no MoveIt/move_group)."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("dofbot_moveit")
    urdf = os.path.join(moveit_share, "config", "dofbot.urdf.xacro")
    controllers = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    description = {"robot_description": xacro.process_file(urdf).toxml()}
    return LaunchDescription([
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
    ])
