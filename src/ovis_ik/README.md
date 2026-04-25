# ovis_ik

ROS 2 Humble `ament_python` package integrating inverse differential kinematics for Ovis.

## Nodes

- `velocity_node`: converts cartesian `Twist` into joint velocity commands with a DH model.
- `constant_vel_publisher`: test publisher that emits a constant `Twist` on `/mouse_twist`.
- `fk_simulation_node`: bridge node forwarding `/joint_states` into `/robot_joint_states`.
- `ovis_viz_node`: plots the DH arm state from `/robot_joint_states`.

## Run

```bash
ros2 run ovis_ik velocity_node
ros2 run ovis_ik constant_vel_publisher
ros2 run ovis_ik fk_simulation_node
ros2 run ovis_ik ovis_viz_node
```

## Dependencies

Install baseline ROS dependencies with `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

`roboticstoolbox-python` is a pip dependency and typically not available as a standard Ubuntu/rosdep system package key.
Install it manually:

The numpy version is important for roboticstoolbox
```bash
python3 -m pip install roboticstoolbox-python numpy==1.24.4
```
