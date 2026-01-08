# Autonomous Water Surface Cleaning Robot

## Project Overview:

The Water Surface Cleaning Robot is an autonomous/semi-autonomous robotic system designed to remove floating waste such as plastic, leaves, and debris from water bodies including lakes, ponds, rivers, and reservoirs. The project aims to reduce water pollution while minimizing human effort and risk.
This robot operates on the water surface using a propulsion system and a waste collection mechanism, making it suitable for environmental monitoring and cleaning applications.

## About this repository
This repository contains the software stack for an autonomous water surface cleaning robot built using ROS 2. It includes coverage path planning, path following, odometry-based localization, GPS conversion, and MQTT-based cloud communication. The system allows users to select a cleaning area via a our application and autonomously execute the mission while streaming live location data.

## Technologies Used

- ROS 2 (Jazzy / Humble compatible)
- Python
- MQTT (EMQX)

## Setup Instructions
### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 installed and sourced
- Python 3.10+
- MQTT Broker (local or cloud)

### Workspace Setup
```bash # Create workspace and clone repo
mkdir -p ~/wscr_ws/src
cd ~/wscr_ws/src
git clone <your-repository-url>

# Install dependencies
cd ~/wscr_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source environment
source ~/wscr_ws/install/setup.bash

# Run the server and start ROS2 Nodes
ros2 launch robot_mission_node path_planning.launch.py
ros2 launch boat_1 rviz.launch.py
