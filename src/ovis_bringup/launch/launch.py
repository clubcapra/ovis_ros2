import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    moveit_pkg_path = get_package_share_directory('ovis_moveit')
    pkg_ovis_servo = get_package_share_directory('ovis_servo')

    # Get the URDF file (robot)
    urdf_path = os.path.join(moveit_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path, ' hardware_type:=', "mock"]), value_type=str)

    # Get the launch directory
    pkg_ovis_moveit = get_package_share_directory('ovis_moveit')

    namespace = '/ovis'


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
        namespace=namespace,
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


    # Fake joint driver
    fake_joint_driver = Node(
        package='controller_manager',
        namespace='/ovis',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            os.path.join(pkg_ovis_moveit, 'config', 'ros2_controllers.yaml'),

        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster', '-c', '/ovis/controller_manager'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller', '-c', '/ovis/controller_manager'],
        output='screen'
    )

    servo =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_servo, 'launch', 'servo.launch.py')
        )
    )

    return LaunchDescription([
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_state_publisher,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_arm_controller],
                )
            ),
            virtual_joints_launch,
            robot_state_publisher,
            move_group_launch,
            rviz_launch,
            fake_joint_driver,
            servo,
            ])
