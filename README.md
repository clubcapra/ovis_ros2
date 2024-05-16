# ovis-ros2

Launch gz sim :
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ovis_bringup sim.launch.py
```

Launch gui state publisher and arm visualization :
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ovis_description launch.py
```