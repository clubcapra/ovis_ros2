import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


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
    kine_file = os.path.join(pkg_path, 'config', "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("ovis", package_name="ovis_moveit")
        .robot_description(file_path=xacro_file)
        .robot_description_kinematics(file_path=kine_file)
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("ovis_servo", "config/ovis_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    kine_yaml = load_yaml("ovis_moveit", "config/kinematics.yaml")
    kine_params = {"robot_description_kinematics": kine_yaml}
    joint_limits_yaml = load_yaml("ovis_moveit", "config/joint_limits.yaml")
    joint_limits_params = {"moveit_servo": joint_limits_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            kine_params,
            joint_limits_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        output="screen",
    )

    return LaunchDescription([
        servo_node,
        container,
    ])
