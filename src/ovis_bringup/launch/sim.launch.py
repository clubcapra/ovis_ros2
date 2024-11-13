import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command 
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the 'with_rove' launch argument
    declare_with_rove_arg = DeclareLaunchArgument(
        'with_rove',
        default_value='false',
        description='Set to "true" to disable gz_sim, bridge, and rviz_launch nodes.'
    )

    declare_with_joy_arg = DeclareLaunchArgument(
        'with_joy',
        default_value='true',
        description='Set to "false" to disable joy node.'
    )

    declare_ovis_origin = DeclareLaunchArgument(
        'ovis_base_origin',
        default_value='0.2 0 0.3 0 0 3.14',
        description='Base origin as "x y z roll pitch yaw"'
    )



    # Retrieve package directories
    pkg_ovis_description = get_package_share_directory('ovis_description')
    moveit_pkg_path = get_package_share_directory('ovis_moveit')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ovis_servo = get_package_share_directory('ovis_servo')
    pkg_ovis_moveit = get_package_share_directory('ovis_moveit')

    # Define paths to URDF and world files
    urdf_path = os.path.join(moveit_pkg_path, 'config', 'ovis.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path,
        ' hardware_type:=', 'gazebo',
        ' ovis_base_origin:="', LaunchConfiguration('ovis_base_origin'), '"'
    ]), value_type=str)
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_ovis_description, world_file_name)

    namespace = '/ovis'

    # Define nodes and launch descriptions
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[
            {'robot_description': robot_desc},
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
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('with_rove'), "' == 'false'"]))
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('with_rove'), "' == 'false'"]))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ovis_description, 'config', 'default_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            "use_sim_time": True
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('with_rove'), "' == 'false'"]))
    )

    
    ovis_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ovis',
                   '-topic', '/ovis/robot_description'],
        output='screen'
    )

    servo = IncludeLaunchDescription(
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

    return LaunchDescription([
            declare_with_rove_arg,
            declare_with_joy_arg,
            declare_ovis_origin,
            SetParameter(name='use_sim_time', value=True),
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
            servo, # Comment this to allow rviz trajectory planner instead of gamepad
            ])

