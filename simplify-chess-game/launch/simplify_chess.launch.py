"""Single-terminal bringup for simplify_chess_game: backend + RViz + board.

Backend bringup follows the PROVEN path of
src/chess_moveit_demo/launch/chess_sim.launch.py -> dofbot_moveit
demo.launch.py (MoveItConfigsBuilder + generate_rsp_launch +
ros2_control_node + spawn_controllers.launch.py +
static_virtual_joint_tfs.launch.py), minus move_group/moveit_rviz.
chess_sim shows the real STL arm through this exact path, so simplify
reuses it instead of hand-rolled xacro.process_file / spawner nodes.

Controller-only: no MoveIt/move_group is started.

Launch this ONCE; starting it twice duplicates robot_state_publisher /
controller_manager under identical node names and freezes the RViz robot
while joints keep moving.

Arguments:
  use_lite_model: true -> 14-box proxy URDF instead of the ~104MB STL meshes.
    Same joints/links, much lighter rendering on weak GPUs.
  show_skeleton: true -> also show the red-bone/green-joint stick figure
    overlay (fallback for virtualized GL where STL stays invisible).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

import fcntl
import os
import tempfile

_session_locks = []


def _acquire_lock(name):
    """Refuse to start twice: duplicate rsp/controller_manager/RViz with
    identical node names freeze the RViz robot while joints keep moving
    (same guard idea as chess_sim's session lock, fixed path, no env change).
    flock releases automatically when this launch process dies."""
    path = os.path.join(tempfile.gettempdir(), f"dofbot-{name}-{os.getuid()}.lock")
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError(
            f"simplify {name} already running (lock {path}). "
            "Ctrl-C the old launch first; only ONE simplify launch at a time.")
    _session_locks.append(fd)


def _refuse_if_chess_sim_running():
    """chess_sim uses the same node names (controller_manager,
    robot_state_publisher, spawners) and the same /chess/visual topic:
    co-running corrupts BOTH launches. chess_sim holds
    /tmp/dofbot-chess-<uid>-42.lock while alive (default domain)."""
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
    """Write processed full+lite URDFs for the RViz Robot display.

    Proven by experiment: RobotModel with Description Source=Topic stays
    empty (Status Ok, nothing drawn) in this environment, while File source
    renders the full arm. Rewritten on every launch so it never goes stale.
    """
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
    _acquire_lock("simplify-rviz")
    _refuse_if_chess_sim_running()
    use_lite = LaunchConfiguration("use_lite_model").perform(context).lower() == "true"
    # Same builder as dofbot_moveit demo.launch.py (model=dofbot default);
    # snapshots feed the RViz File-source Robot display (see helper above).
    mc_full, mc_lite = _write_urdf_snapshot()
    mc = mc_lite if use_lite else mc_full
    share = FindPackageShare("dofbot_moveit")
    simplify_share = get_package_share_directory("simplify_chess_game")
    rviz_config = os.path.join(
        simplify_share, "config",
        "simplify_chess_lite.rviz" if use_lite else "simplify_chess.rviz")

    ld = LaunchDescription()
    # world -> base_link, same as chess_sim.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [share, "launch", "static_virtual_joint_tfs.launch.py"]))))
    # robot_state_publisher, same as chess_sim.
    for entity in list(generate_rsp_launch(mc).entities):
        ld.add_action(entity)
    # ros2_control_node, exact copy from dofbot_moveit demo.launch.py.
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            mc.robot_description,
            str(mc.package_path / "config/ros2_controllers.yaml"),
        ],
    ))
    # joint_state_broadcaster + arm/grip controllers, same as chess_sim.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [share, "launch", "spawn_controllers.launch.py"]))))
    # RViz with full moveit params, same style as chess_sim. The Robot
    # display inside reads its model from the /tmp snapshot file (see
    # helper), chosen here to match use_lite_model.
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
