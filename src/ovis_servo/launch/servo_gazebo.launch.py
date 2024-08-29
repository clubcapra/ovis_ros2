import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import xacro
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
    xacro_file = os.path.join(pkg_path, 'config', "ovis_gazebo.urdf.xacro")
    kine_file = os.path.join(pkg_path, 'config', "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("ovis")
        .robot_description(file_path=xacro_file)
        .robot_description_kinematics(file_path=kine_file)
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("ovis_servo", "config/ovis_gazebo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    kine_yaml = load_yaml("ovis_moveit", "config/kinematics.yaml")
    kine_params = {"moveit_servo": kine_yaml}
    joint_limits_yaml = load_yaml("ovis_moveit", "config/joint_limits.yaml")
    joint_limits_params = {"moveit_servo": joint_limits_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'use_sim_time': True},
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ovis_moveit"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ovis_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #         kine_params,
            #         joint_limits_params,
            #     ],
            # ),
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[moveit_config.robot_description, {'use_sim_time': True},],
            # ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/ovis_link_base", "frame_id": "/ovis_base_link", "use_sim_time": True}],
            ),
            ComposableNode(
                package="ovis_servo",
                plugin="ovis_servo::JoyToServoPub",
                name="controller_to_servo_node",
                parameters=[{'use_sim_time': True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                parameters=[{'use_sim_time': True}],
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
            {'use_sim_time': True}
        ],
        output="screen",
    )

    return LaunchDescription([
        rviz_node,
        #ros2_control_node,
        joint_state_broadcaster_spawner,
        ovis_arm_controller_spawner,
        servo_node,
        container,
    ])
