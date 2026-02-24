# ovis-ros2

Ovis is a 6 Degrees of Freedom (DoF) robotic arm developed by the CAPRA student club. It is designed to be highly versatile and is compatible with MoveIt for motion planning and Kinova motors for precise control. This repository provides the necessary tools and launch files to simulate and visualize the Ovis robotic arm using ROS2.


## Installation

Clone and build the repository :
```bash
git clone https://github.com/clubcapra/ovis_ros2.git
cd ovis_ros2
vcs import src < ovis.repos
colcon build --symlink-install
source install/setup.bash
```

## Usage

Start the gazebo simulation :
```bash
ros2 launch ovis_bringup sim.launch.py
```

Launch gui state publisher and arm visualization :
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ovis_description launch.py
```

Launch MoveIt! :
```bash
ros2 launch ovis_moveit demo.launch.py
```

Launch Joystick control with Moveit! and Servo :
```bash
ros2 launch ovis_bringup combined.launch.py
```

## Contributing

Before pushing your code, you should try to deploy it by using:
```bash
docker compose build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) --build-arg USERNAME=$(id -un)
```