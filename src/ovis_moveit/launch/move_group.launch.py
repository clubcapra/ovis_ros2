from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ovis", package_name="ovis_moveit").to_moveit_configs()

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(""),
        "disable_capabilities": ParameterValue(""),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace="ovis",
        parameters=move_group_params,
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )
    return LaunchDescription([move_group])