import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to other launch files
    demo_launch = os.path.join(get_package_share_directory('ovis_moveit'), 'launch', 'demo.launch.py')
    servo_demo_launch = os.path.join(get_package_share_directory('ovis_servo'), 'launch', 'servo_demo.launch.py')
    
    # Node for publishing the initial trajectory
    publish_trajectory_node = Node(
        package='ovis_bringup',
        executable='publish_trajectory.py',
        name='publish_trajectory_node',
        output='screen'
    )
    
    # Event handler to launch servo_keyboard_input after publish_trajectory_node exits
    on_publish_trajectory_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=publish_trajectory_node,
            on_exit=[
                LogInfo(msg="Starting servo_keyboard_input node..."),
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'ros2', 'run', 'ovis_realtime_servo', 'servo_keyboard_input'],
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([
        # Include the existing demo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch)
        ),
        # Include the existing servo_demo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_demo_launch)
        ),
        # Start the servo service
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
                    output='screen'
                )
            ]
        ),
        # Publish the initial trajectory
        TimerAction(
            period=10.0,
            actions=[
                publish_trajectory_node
            ]
        ),
        # Launch servo_keyboard_input after publish_trajectory_node exits
        on_publish_trajectory_exit
    ])
