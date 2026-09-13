"""RViz-only visualization for simplify_chess_game.

Robot description params follow the same PROVEN MoveItConfigsBuilder path
as chess_sim (checker: robot STL arm visible), so this view shows the same
real mesh as simplify_chess.launch.py. Pair use_lite_model with the backend.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import fcntl
import os
import tempfile

_session_locks = []


def _acquire_lock(name):
    """Same single-instance guard as simplify_chess.launch.py."""
    path = os.path.join(tempfile.gettempdir(), f"dofbot-{name}-{os.getuid()}.lock")
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError(
            f"simplify {name} already running (lock {path}). "
            "Ctrl-C the old launch first; only ONE simplify rviz at a time.")
    _session_locks.append(fd)


def _refuse_if_chess_sim_running():
    """Same cross-guard as simplify_chess.launch.py (shared /chess/visual)."""
    path = os.path.join(
        tempfile.gettempdir(), f"dofbot-chess-{os.getuid()}-42.lock")
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError(
            "chess_sim is already running (lock "
            f"{path}). Ctrl-C it first; chess_sim and simplify must NEVER "
            "run together (shared /chess/visual topic).")
    os.close(fd)


FULL_URDF_PATH = os.path.join(tempfile.gettempdir(), "dofbot_simplify_full.urdf")
LITE_URDF_PATH = os.path.join(tempfile.gettempdir(), "dofbot_simplify_lite.urdf")


def _write_urdf_snapshot():
    """Write processed full+lite URDFs for the RViz File-source Robot
    display. Rewritten on every launch so it never goes stale."""
    full = MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit").to_moveit_configs()
    lite = MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit").robot_description(
        file_path="config/dofbot_lite.urdf.xacro").to_moveit_configs()
    with open(FULL_URDF_PATH, "w", encoding="utf-8") as stream:
        stream.write(full.robot_description["robot_description"])
    with open(LITE_URDF_PATH, "w", encoding="utf-8") as stream:
        stream.write(lite.robot_description["robot_description"])
    return full, lite


def _build(context):
    _acquire_lock("simplify-rviz")
    _refuse_if_chess_sim_running()
    use_lite = LaunchConfiguration("use_lite_model").perform(context).lower() == "true"
    mc_full, mc_lite = _write_urdf_snapshot()
    mc = mc_lite if use_lite else mc_full
    simplify_share = get_package_share_directory("simplify_chess_game")
    rviz_config = os.path.join(
        simplify_share, "config",
        "simplify_chess_lite.rviz" if use_lite else "simplify_chess.rviz")

    ld = LaunchDescription()
    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="simplify_chess_rviz",
        arguments=["-d", rviz_config],
        parameters=[mc.to_dict()],
        output="screen",
    ))
    ld.add_action(Node(
        package="simplify_chess_game",
        executable="chess_board_viz",
        name="simplify_chess_board_visualizer",
        output="screen",
    ))
    ld.add_action(Node(
        package="simplify_chess_game",
        executable="arm_skeleton_viz",
        name="simplify_arm_skeleton",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_skeleton")),
    ))
    return list(ld.entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_lite_model", default_value="false",
            description="true: 14-box proxy URDF for weak GPUs (same joints/links)"),
        DeclareLaunchArgument(
            "show_skeleton", default_value="false",
            description="true: also show red-bone/green-joint stick figure, else only real mesh RobotModel"),
        OpaqueFunction(function=_build),
    ])
