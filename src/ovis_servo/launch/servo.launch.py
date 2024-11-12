import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory("ovis_moveit"))
    xacro_file = os.path.join(pkg_path, 'config', "ovis.urdf.xacro")
    srdf_file = os.path.join(pkg_path, 'config', "ovis.srdf")
    
    # Build moveit config
    moveit_config = (
        MoveItConfigsBuilder("ovis", package_name="ovis_moveit")
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=os.path.join(pkg_path, "config", "kinematics.yaml"))
        .to_moveit_configs()
    )

    # Load servo config
    servo_yaml = load_yaml("ovis_servo", "config/ovis_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/ovis",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="ovis_servo",
                plugin="ovis_servo::JoyToServoPub",
                name="controller_to_servo_node",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                remappings=[
                    ("/joy", "/ovis/joy"),
                    ("~/delta_twist_cmds", "/ovis/servo_node/delta_twist_cmds"),
                    ("~/delta_joint_cmds", "/ovis/servo_node/delta_joint_cmds"),
                    ("/planning_scene", "/ovis/planning_scene")
                ],
            ),
            ComposableNode(
                package="joy",
                namespace="/ovis",
                plugin="joy::Joy",
                name="joy_node",
                condition=IfCondition(LaunchConfiguration('with_joy'))
            ),
        ],
        output="screen",
    )

    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        namespace="ovis",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # Add service call to start servo
    start_servo_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/ovis/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # Register event handler to call start_servo after servo_node has started
    start_servo_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=servo_node,
            on_start=[start_servo_cmd]
        )
    )

    return LaunchDescription([
        servo_node,
        container,
        start_servo_event
    ])
