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