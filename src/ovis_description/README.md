# Ovis Description

This package contains the URDF description of the ovisr and everything related to the simulation of the robot.

# Usage

Once everything has been installed and compiled, you can use the following command to launch the simulation:

```bash
ros2 launch ovis_description sim.launch.py
```

This command will start the simulation and the GUI. You will then be able to make the robot move using ros2 publishers.

# Note

In an attempt to add IK to ovis, some configuration is needed.
According to https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html#switch-to-cyclone-dds it may be needed to add this.