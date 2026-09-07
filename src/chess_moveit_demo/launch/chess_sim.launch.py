"""Khởi động Dofbot MoveIt2 fake-control + RViz rồi chạy demo cờ."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    moveit_config = MoveItConfigsBuilder(
        "dofbot", package_name="dofbot_moveit"
    ).to_moveit_configs()
    chess_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="chess_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("chess_moveit_demo"), "config", "chess.rviz"]
            ),
        ],
        parameters=[moveit_config.to_dict()],
        output="screen",
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

    # move_group, IK plugin và fake controllers của Dofbot cần vài giây để sẵn
    # sàng. Nếu brain publish ngay, nước đầu có thể bị mất vì các service planning
    # chưa được advertise. Chỉ bắt đầu game sau khi toàn bộ MoveIt stack ổn định.
    chess_after_moveit_ready = TimerAction(
        period=12.0,
        actions=[chess_brain, pick_place],
    )

    return LaunchDescription([moveit_launch, chess_rviz, chess_after_moveit_ready])
