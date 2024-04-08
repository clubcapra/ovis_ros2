# ovis-ros2

Launch gazebo command:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ovis_description sim.launch.py
```

Launch rviz command:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ovis_description launch.py
```

Launch kinova robot
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kinova_bringup kinova_robot_launch.py
```
## Interactive mode
```bash
ros2 run kinova_driver kinova_interactive_control c2n6s200
```

## Moveit
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch kinova_bringup moveit_robot_launch.py
```
```bash
ros2 run kinova_driver joint_trajectory_action_server c2n6s200
```
```bash
ros2 run kinova_driver gripper_command_action_server c2n6s200
```

To install needed dependencies of kinova drivers:
```bash
sudo rosdep init
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro humble

sudo apt update
sudo apt install ros-humble-moveit
```