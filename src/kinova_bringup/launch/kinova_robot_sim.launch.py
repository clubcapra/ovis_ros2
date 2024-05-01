import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
import yaml

configurable_parameters = [
    {'name': 'use_urdf',              'default': "true"},
    {'name': 'kinova_robotType',      'default': "c2n6s200"},
    {'name': 'kinova_robotName',      'default': "ovis"},
    {'name': 'kinova_robotSerial',    'default': "not_set"},
    {'name': 'use_jaco_v1_fingers',   'default': "true"},
    {'name': 'feedback_publish_rate', 'default': "0.1"},
    {'name': 'tolerance',             'default': "2.0"},
]

def generate_launch_description():
    # Configure ROS nodes for launch

    # Get the launch directory
    pkg_ovis_description = get_package_share_directory('ovis_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Get the URDF file
    urdf_path = os.path.join(pkg_ovis_description, 'urdf', 'ovis_standalone.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_ovis_description, world_file_name)

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
    )

    # Spawn robot
    create = Node(
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

    # Takes the description and joint angles as inputs and publishes
    # the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {"use_sim_time": True}
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ovis_description, 'config', 'moveit.rviz')],
       condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ovis_description, 'config', 'default_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            "use_sim_time": True,
        }],
        output='screen'
    )

    # Call launch_setup function to set up Kinova nodes
    kinova_nodes = LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function = launch_setup)])

    return LaunchDescription([
            gz_sim,
            DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
            bridge,
            #robot_state_publisher,
            rviz,
            create,
            kinova_nodes,
            ])

def launch_setup(context):
    robot_type = LaunchConfiguration("kinova_robotType").perform(context)

    # Add your Kinova nodes setup here
    # For example:
    kinova_tf_updater = Node(
        package='kinova_driver',
        name=robot_type+'_tf_updater',
        executable='kinova_tf_updater',
        parameters=[{'base_frame': 'root'}],
        remappings=[(robot_type+'_tf_updater/in/joint_angles', robot_type+'_driver/out/joint_angles')],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_urdf")),
    )
    kinova_interactive_control = Node(
        package='kinova_driver',
        name=robot_type+'_interactive_control',
        executable='kinova_interactive_control',
        parameters=[{'base_frame': 'root'}],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_urdf")),
    )
    xacro_file = os.path.join(get_package_share_directory('ovis_description'), 'urdf', 'ovis' + '_standalone.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[('joint_states', robot_type+'_driver/out/joint_state')],
        output='screen',
        parameters=[{'robot_description': robot_desc},],
        condition=IfCondition(LaunchConfiguration("use_urdf")),
    )
    return [kinova_tf_updater,robot_state_publisher,kinova_interactive_control]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
