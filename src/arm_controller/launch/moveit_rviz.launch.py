from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="arm_controller")
        .to_moveit_configs()
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]

    rviz_config = str(moveit_config.package_path / "config/moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
    )

    return LaunchDescription([rviz_node])