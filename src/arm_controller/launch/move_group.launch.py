import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="arm_controller")
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": True,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return LaunchDescription([move_group_node])