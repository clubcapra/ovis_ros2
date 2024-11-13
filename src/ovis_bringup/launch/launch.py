import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, SetParameter
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

    # Takes the description and joint angles as inputs and publishes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
        ]
    )

    # Fake joint driver
    fake_joint_driver = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            {'robot_description': robot_desc},
            os.path.join(pkg_ovis_moveit, 'config', 'ros2_controllers.yaml'),
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            f"{namespace}/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "arm_controller",
            "--controller-manager",
            f"{namespace}/controller_manager",
        ],
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    servo =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_servo, 'launch', 'servo.launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'rviz_config': os.path.join(pkg_ovis_moveit, 'config', 'moveit.rviz'),
        }.items(),
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        robot_state_publisher,
        fake_joint_driver,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_launch,
        rviz_launch,
        # servo # Uncomment this line to enable the servo (gamepad control)
    ])
