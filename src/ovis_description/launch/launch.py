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
    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')

    # Get the URDF file
    urdf_path = os.path.join(pkg_ovis_description, 'urdf', 'ovis_standalone.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

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

    # Takes the description and joint angles as inputs and publishes
    # the 3D poses of the robot links
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

    # A GUI to manipulate the joint state values
    joint_state_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui')

    return LaunchDescription([
            joint_state_gui,
            robot_state_publisher,
            rviz,
            # create,
            ])
