import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    tea_share=get_package_share_directory(
        "dofbot_tea_moveit"
    )

    # Giới hạn conservative của task trà (1.0 rad/s). File generic của
    # dofbot_moveit để 1000 rad/s và không có accel limit — trước đây file này
    # là orphan, không launch nào load, nên move_group chạy với limit ảo.
    joint_limits=os.path.join(
        tea_share,
        "config",
        "tea_joint_limits.yaml"
    )

    cfg=MoveItConfigsBuilder(
        "dofbot",
        package_name="dofbot_moveit"
    ).robot_description(
        file_path="config/dofbot.urdf.xacro"
    ).joint_limits(
        joint_limits
    ).to_moveit_configs()

    moveit_share=get_package_share_directory(
        "dofbot_moveit"
    )

    srdf_path=os.path.join(
        tea_share,
        "config",
        "dofbot_tea.srdf"
    )

    with open(
        srdf_path,
        "r",
        encoding="utf-8"
    ) as f:
        semantic={
            "robot_description_semantic":
                f.read()
        }

    controllers=os.path.join(
        moveit_share,
        "config",
        "ros2_controllers.yaml"
    )

    static_tf=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tea_world_to_base",
        arguments=[
            "--x","0",
            "--y","0",
            "--z","0",
            "--roll","0",
            "--pitch","0",
            "--yaw","0",
            "--frame-id","world",
            "--child-frame-id","base_link"
        ],
        output="screen",
        ros_arguments=[
            "--log-level","WARN"
        ]
    )

    rsp=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            cfg.robot_description
        ],
        output="screen",
        ros_arguments=[
            "--log-level","WARN"
        ]
    )

    control=Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            cfg.robot_description,
            controllers
        ],
        output="screen",
        ros_arguments=[
            "--log-level","WARN"
        ]
    )

    jsb=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    arm=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_group_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    grip=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "grip_group_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    move_group=Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        parameters=[
            cfg.to_dict(),
            semantic,
            {
                "allow_trajectory_execution":True,
                "publish_robot_description_semantic":True
            }
        ],
        output="screen",
        ros_arguments=[
            "--log-level","WARN"
        ]
    )

    return LaunchDescription([
        static_tf,
        rsp,
        control,
        jsb,
        arm,
        grip,
        move_group
    ])
