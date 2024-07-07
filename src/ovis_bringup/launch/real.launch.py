import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    moveit_pkg_path = get_package_share_directory('ovis_moveit')
    pkg_ovis_servo = get_package_share_directory('ovis_servo')

    # Get the URDF file (robot)
    urdf_path = os.path.join(moveit_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path, ' hardware_type:=', "ovis"]), value_type=str)

    # Get the launch directory
    pkg_ovis_moveit = get_package_share_directory('ovis_moveit')


    # Include launch files based on the configuration
    virtual_joints_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'static_virtual_joint_tfs.launch.py')
        )
    )

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

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'move_group.launch.py'),
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
    )


    # kinova driver
    kinova_joint_driver = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            os.path.join(pkg_ovis_moveit, 'config', 'ros2_controllers.yaml'),

        ],
    )

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'spawn_controllers.launch.py')
        )
    )

    servo =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_servo, 'launch', 'servo.launch.py')
        )
    )

    return LaunchDescription([
            virtual_joints_launch,
            robot_state_publisher,
            move_group_launch,
            rviz_launch,
            kinova_joint_driver,
            spawn_controllers_launch,
            servo,
            ])
