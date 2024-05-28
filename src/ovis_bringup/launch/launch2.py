# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils.launches import generate_moveit_rviz_launch, generate_spawn_controllers_launch, generate_move_group_launch, generate_rsp_launch


def generate_launch_description():
    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')
    bringup_pkg_path = get_package_share_directory('ovis_bringup')
    moveit_pkg_path = get_package_share_directory('ovis_moveit')

    # Get the URDF file
    # urdf_path = os.path.join(pkg_ovis_description, 'urdf', 'ovis_standalone.urdf.xacro')
    # robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    urdf_path = os.path.join(pkg_ovis_description, 'urdf', 'ovis2', 'urdf', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ovis_bringup"),
    #         "config",
    #         "ovis_controllers.yaml",
    #     ]
    # )
    
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ovis_moveit"),
    #         "config",
    #         "ovis_controllers.yaml",
    #     ]
    # )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ovis_bringup"),
            "config",
            "ovis_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )
    
    # Takes the description and joint angles as inputs and publishes
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        respawn=True,
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            # {"use_sim_time": True, }
        ]
    )
    # robot_state_pub_node = generate_rsp_launch()
    # Visualize in RViz
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ovis_description, 'config',
                                     'basic.rviz')],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ovis_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)