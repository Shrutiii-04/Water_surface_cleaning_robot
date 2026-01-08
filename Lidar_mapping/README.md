# Water Surface Cleaning Robot Simulation

This repository contains the simulation of an autonomous water surface cleaning robot developed using Gazebo Ignition and ROS 2 Humble.  
The simulation focuses on LiDAR-based mapping and visualization using RViz, enabling testing of navigation and mapping algorithms in a virtual water environment.

---

## Features

- Simulated water surface cleaning robot (boat-based platform)
- LiDAR sensor integration for environment perception
- Real-time mapping using SLAM Toolbox
- Visualization using RViz2
- Manual teleoperation support
- Gazebo Ignition-based physics simulation

---

## Requirements

### Operating System
- Ubuntu 22.04 (recommended)

### Software Dependencies
- ROS 2 Humble Hawksbill
- Gazebo Ignition

---

## Running the Simulation

### Step 1: Kill Existing Gazebo and ROS Processes (Recommended)
```
pkill -9 -f "ign gazebo|gz sim|ros2|rviz2|slam_toolbox|parameter_bridge|ruby.*ign|gzserver|gzclient"
```

---

### Step 2: Launch Gazebo Ignition Environment
```
ign gazebo world.sdf
```
This command starts the Gazebo Ignition simulation with the water surface environment and robot model.

---

Step 3: Start Mapping and RViz Visualization

Open a new terminal and run:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch boat_control absolute_mapping.launch.py
```
This step performs the following:

- Starts SLAM Toolbox for LiDAR-based mapping
- Launches RViz2 for visualization
- Displays robot pose, laser scan, and the generated map

---

Step 4: Enable Teleoperation

Open another terminal and run:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run boat_control teleop
```

Use the configured keyboard controls to manually navigate the robot within the simulated environment.
