# Robotic Arm Demo (MoveIt 2 + Franka Panda)

A small ROS 2 Humble C++ node that drives the Franka Panda arm through MoveIt 2's
`MoveGroupInterface`:

1. Plans and executes a move to the named configuration **`ready`** (defined in the SRDF).
2. Plans and executes a move to a **pose goal** (x=0.3, y=0.2, z=0.5), so MoveIt solves IK first.

Runs in RViz with MoveIt's simulated ("fake") controllers. No Gazebo yet.

## Requirements
- ROS 2 Humble
- `ros-humble-moveit`, `ros-humble-moveit-resources-panda-moveit-config`

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run
Terminal 1: MoveIt + RViz with the Panda:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
Terminal 2: this demo (once Terminal 1 prints "You can start planning now!"):
```bash
ros2 launch panda_moveit_demo panda_arm_demo.launch.py
```

## Layout
- `src/panda_moveit_demo/src/panda_arm_demo.cpp`: the node
- `src/panda_moveit_demo/launch/panda_arm_demo.launch.py`: loads the Panda MoveIt config and starts the node