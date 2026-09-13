"""Controller-only backend for simplify_chess_game (no MoveIt/move_group).

Same PROVEN bringup as dofbot_moveit demo.launch.py (used by chess_sim):
MoveItConfigsBuilder + generate_rsp_launch + ros2_control_node +
spawn_controllers.launch.py + static_virtual_joint_tfs.launch.py.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

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
            "Ctrl-C the old launch first; only ONE simplify backend at a time.")
    _session_locks.append(fd)


def _refuse_if_chess_sim_running():
    """Same cross-guard as simplify_chess.launch.py."""
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
            "run together (duplicate controller_manager/robot_state_publisher).")
    os.close(fd)


FULL_URDF_PATH = os.path.join(tempfile.gettempdir(), "dofbot_simplify_full.urdf")
LITE_URDF_PATH = os.path.join(tempfile.gettempdir(), "dofbot_simplify_lite.urdf")


def _write_urdf_snapshot():
    """Write processed full+lite URDFs for the RViz File-source Robot
    display (paired simplify_rviz.launch.py reads them). Rewritten on
    every launch so it never goes stale."""
    full = MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit").to_moveit_configs()
    lite = MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit").robot_description(
        file_path="config/dofbot_lite.urdf.xacro").to_moveit_configs()
    with open(FULL_URDF_PATH, "w", encoding="utf-8") as stream:
        stream.write(full.robot_description["robot_description"])
    with open(LITE_URDF_PATH, "w", encoding="utf-8") as stream:
        stream.write(lite.robot_description["robot_description"])
    return full, lite


def _build(context):
    _acquire_lock("simplify-backend")
    _refuse_if_chess_sim_running()
    use_lite = LaunchConfiguration("use_lite_model").perform(context).lower() == "true"
    mc_full, mc_lite = _write_urdf_snapshot()
    mc = mc_lite if use_lite else mc_full
    share = FindPackageShare("dofbot_moveit")

    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [share, "launch", "static_virtual_joint_tfs.launch.py"]))))
    for entity in list(generate_rsp_launch(mc).entities):
        ld.add_action(entity)
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            mc.robot_description,
            str(mc.package_path / "config/ros2_controllers.yaml"),
        ],
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [share, "launch", "spawn_controllers.launch.py"]))))
    return list(ld.entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_lite_model", default_value="false",
            description="true: 14-box proxy URDF for weak GPUs (same joints/links)"),
        OpaqueFunction(function=_build),
    ])
