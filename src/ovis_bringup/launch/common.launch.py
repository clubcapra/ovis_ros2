import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')
    bringup_pkg_path = get_package_share_directory('ovis_bringup')

    # Get the URDF file
    urdf_path = os.path.join(pkg_ovis_description, 'urdf', 'ovis_standalone.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Takes the description and joint angles as inputs and publishes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {"use_sim_time": True, }
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ovis_description, 'config',
                                     'basic.rviz')],
    )

     # Include MoveIt configuration
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ovis_moveit'), 'launch', 'demo.launch.py')
        ),
    )

    return LaunchDescription([
            robot_state_publisher,
            rviz,
            moveit,
            ])
