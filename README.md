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



sudo apt update

sudo apt install ros-humble-moveit
