import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command 
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')
    moveit_pkg_path = get_package_share_directory('ovis_moveit')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ovis_servo = get_package_share_directory('ovis_servo')
    pkg_ovis_moveit = get_package_share_directory('ovis_moveit')

    # Get the URDF file (robot)
    urdf_path = os.path.join(moveit_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path, ' hardware_type:=', "gazebo"]), value_type=str)

    # Get the URDF file (world)
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_ovis_description, world_file_name)


    namespace = '/ovis'

    # Takes the description and joint angles as inputs and publishes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {"use_sim_time": True }
        ]
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis_moveit, 'launch', 'move_group.launch.py'),
        ),
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

    # Spawn ovis (robot arm)
    ovis_spawner = Node(
        package='ros_gz_sim',
        # namespace=namespace,
        executable='create',
        arguments=['-name', 'ovis',
                   '-topic', '/ovis/robot_description',
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
             'joint_state_broadcaster', '-c', '/ovis/controller_manager'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller', '-c', '/ovis/controller_manager'],
        output='screen'
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )


    return LaunchDescription([
            SetParameter(name='use_sim_time', value=True),
            static_tf,
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
            robot_state_publisher,
            move_group_launch,
            rviz_launch,
            gz_sim,
            bridge,
            ovis_spawner,
            servo,
            ])
