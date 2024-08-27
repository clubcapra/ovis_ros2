from ast import arg
import re
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ovis", package_name="ovis_moveit").to_moveit_configs()


    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        #namespace="ovis",
        output="both",
        respawn=False,
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=rviz_parameters,
        remappings=[('/robot_description', '/ovis/robot_description'),
        ],
    )

    return LaunchDescription([moveit_rviz])