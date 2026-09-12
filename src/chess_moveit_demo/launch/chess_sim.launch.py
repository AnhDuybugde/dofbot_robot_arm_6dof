"""Khởi động Dofbot MoveIt2 fake-control + RViz rồi chạy demo cờ."""

import fcntl
import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    session_lock = []

    def acquire_session(context):
        domain = int(LaunchConfiguration("ros_domain_id").perform(context))
        if not 0 <= domain <= 101:
            raise RuntimeError("ros_domain_id must be between 0 and 101")
        path = os.path.join(
            tempfile.gettempdir(), f"dofbot-chess-{os.getuid()}-{domain}.lock")
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            raise RuntimeError(f"Chess simulation already running in ROS domain {domain}")
        session_lock.append(fd)
        return [SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain))]

    def release_session(context):
        for fd in session_lock:
            os.close(fd)
        session_lock.clear()
        return []

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dofbot_moveit"), "launch", "demo.launch.py"]
            )
        ),
        # Demo MoveIt mặc định cũng mở RViz với config cho task rót trà. Demo cờ
        # tự mở RViz bên dưới để đăng ký MarkerArray /chess/visual.
        launch_arguments={
            "use_rviz": "false",
            "model": PythonExpression([
                "'dofbot_fixed' if '", LaunchConfiguration("use_fixed_model"),
                "' == 'true' else 'dofbot'"]),
        }.items(),
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
        # Sim-only: cho phep execute tren FakeSystem (khong tai). Launch
        # hardware TUYET DOI khong duoc set param nay (mac dinh False = khoa
        # nhu robot that, doi calibration TCP + low-speed test).
        parameters=[{"sim_allow_execute": True}],
    )

    # TODO-1: bỏ timer cố định 12s. Brain/pick-place start ngay cùng MoveIt;
    # pick_place tự gate READY (scene 33/33 + planner + controller +
    # joint_states) rồi publish /chess/system_ready; brain chỉ đi khi READY.
    return LaunchDescription([
        DeclareLaunchArgument("ros_domain_id", default_value="42"),
        DeclareLaunchArgument(
            "use_fixed_model", default_value="false",
            description="false: use five-joint arm_group with arm5 constrained to ±20 degrees"),
        OpaqueFunction(function=acquire_session),
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=release_session)])),
        RegisterEventHandler(OnProcessExit(
            target_action=pick_place,
            on_exit=[EmitEvent(event=Shutdown(reason="Pick-place executor exited"))])),
        RegisterEventHandler(OnProcessExit(
            target_action=chess_brain,
            on_exit=[EmitEvent(event=Shutdown(reason="Chess brain exited"))])),
        moveit_launch, chess_rviz, chess_brain, pick_place,
    ])
