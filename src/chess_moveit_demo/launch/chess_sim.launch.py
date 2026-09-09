"""Khởi động Dofbot MoveIt2 fake-control + RViz rồi chạy demo cờ."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dofbot_moveit"), "launch", "demo.launch.py"]
            )
        ),
        # Demo MoveIt mặc định cũng mở RViz với config cho task rót trà. Demo cờ
        # tự mở RViz bên dưới để đăng ký MarkerArray /chess/visual.
        launch_arguments={"use_rviz": "false"}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
        .robot_description(file_path="config/dofbot.urdf.xacro")
        .to_moveit_configs()
    )
    chess_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="chess_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("chess_moveit_demo"), "config", "chess_lite.rviz"]
            ),
        ],
        parameters=[moveit_config.to_dict()],
        # RViz Qt/plugin spam log rất dài trên terminal launch nhưng hiếm khi
        # chứa lỗi liên quan game: đẩy ra log file, giữ terminal cho log
        # chess brain/pick-place ([OK]/[FAIL]).
        output="log",
        ros_arguments=["--log-level", "WARN"],
    )

    chess_brain = Node(
        package="chess_moveit_demo",
        executable="chess_brain_node",
        name="chess_brain_node",
        output="screen",
    )

    pick_place = Node(
        package="chess_moveit_demo",
        executable="pick_place_node",
        name="chess_pick_place_node",
        output="screen",
    )

    # TODO-1: bỏ timer cố định 12s. Brain/pick-place start ngay cùng MoveIt;
    # pick_place tự gate READY (scene 33/33 + planner + controller +
    # joint_states) rồi publish /chess/system_ready; brain chỉ đi khi READY.
    return LaunchDescription([moveit_launch, chess_rviz, chess_brain, pick_place])
