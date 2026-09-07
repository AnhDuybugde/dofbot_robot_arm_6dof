"""Chạy bàn cờ base: hoàn toàn không có MoveIt/pymoveit2/robot controller."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="chess_base_rviz",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("chess_moveit_demo"), "config", "chess_base.rviz"]
            )],
            output="screen",
        ),
        Node(
            package="chess_moveit_demo",
            executable="chess_base_node",
            name="chess_base_node",
            output="screen",
        ),
        Node(
            package="chess_moveit_demo",
            executable="chess_brain_node",
            name="chess_brain_node",
            output="screen",
        ),
    ])
