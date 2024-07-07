import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')
    bringup_pkg_path = get_package_share_directory('ovis_bringup')
    moveit_pkg_path = get_package_share_directory('ovis_moveit')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ovis_servo = get_package_share_directory('ovis_servo')

    # Get the URDF file (robot)
    urdf_path = os.path.join(moveit_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Get the URDF file (world)
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_ovis_description, world_file_name)

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
    )

    # Set the use_sim_time parameter to true for the entire system
    # Move group doesn't accept use sim time as an argument, we need to manually call it or use SetParameter
    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value='true',
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
    )

    # Spawn robot
    ovis_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ovis',
                   '-topic', 'robot_description',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.1',
                   ],
        output='screen',
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ovis_description, 'config',
                                        'default_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            "use_sim_time": True,
        }],
        output='screen'
    )

    servo =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_servo, 'launch', 'servo.launch.py')
        )
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )


    return LaunchDescription([
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ovis_spawner,
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
            set_use_sim_time,
            move_group_launch,
            rviz_launch,
            gz_sim,
            bridge,
            ovis_spawner,
            servo,
            ])
