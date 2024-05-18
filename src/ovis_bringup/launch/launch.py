import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Configure ROS nodes for launch

    # Get the launch directory
    pkg_ovis_bringup = get_package_share_directory('ovis_bringup')
    pkg_ovis_description = get_package_share_directory('ovis_description')

    # # Get the URDF file
    # world_file_name = 'worlds/base_world.world'
    # world = os.path.join(pkg_ovis_description, world_file_name)

    # # Setup to launch the simulator and Gazebo world
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
    # )

    # # Spawn robot
    # create = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-name', 'ovis',
    #                '-topic', 'robot_description',
    #                '-x', '0',
    #                '-y', '0',
    #                '-z', '0.1',
    #                ],
    #     output='screen',
    # )

    # # Bridge ROS topics and Gazebo messages for establishing communication
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(pkg_ovis_description, 'config',
    #                                     'default_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #         "use_sim_time": True,
    #     }],
    #     output='screen'
    # )

    # Include common launch configuration
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_bringup, "launch", "common.launch.py"),
        ),
    )

    return LaunchDescription([
            # gz_sim,
            # bridge,
            # create,
            common,
            ])
