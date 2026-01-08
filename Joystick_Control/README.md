# Simulation 

## Requirements

- ROS 2 : https://docs.ros.org/en/humble/Installation.html
- ignition gazebo 6 : https://gazebosim.org/docs/latest/ros_installation/ 

## How to Run

1. Run gazebo envarment 

   cd: ign gazebo world.sdf -v 4

2. run ros to nodes 

    cd: ros2 launch boat_control start_boat.launch.py


## Project Structure

```
SIMULATION_2/
├── boat_1/
│   ├── meshes/
│   │   ├── boat.stl
│   │   └── prop.stl
│   ├── model.config
│   └── model.sdf
│
├── environment/
│   ├── meshes/
│   │   └── la.stl
│   ├── model.config
│   └── model.sdf
│
├── build/
├── install/
├── log/
│
├── src/
│   └── boat_control/
│       ├── boat_control/
│       │   └── __init__.py
│       │
│       ├── launch/
│       │   ├── boat_bridge.launch.py
│       │   └── start_boat.launch.py
│       │
│       ├── resource/
│       │   └── boat_control
│       │
│       ├── test/
│       │   ├── test_copyright.py
│       │   ├── test_flake8.py
│       │   └── test_pep257.py
│       │
│       ├── package.xml
│       ├── setup.cfg
│       ├── setup.py
│       └── README.md
│
└── world.sdf

```
