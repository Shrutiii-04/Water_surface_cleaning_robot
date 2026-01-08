# Autonomous Water Surface Cleaning Robot

## 🚀 Autonomous Water Surface Cleaning Robot
An autonomous robot designed to remove floating waste from water bodies using computer vision, ROS 2 navigation, RTK positioning, and solar energy.

India generates over 25,000 tons of waste daily, with a significant portion polluting rivers and lakes. Manual cleaning methods are inefficient, unsafe, and unsustainable. This project proposes a fully autonomous water surface cleaning robot that detects floating trash in real time using YOLOv8, navigates accurately with ROS 2 and RTK GPS, and collects waste through an efficient rotating flap mechanism.
The robot is solar-powered, low-maintenance, and monitored via a web-based dashboard, making it a scalable and eco-friendly solution for municipalities, environmental agencies, and research institutions.
<<<<<<< HEAD
________________________________________

📌 Overview

=======

## 📌 Overview
>>>>>>> 5e87aa7 (Updated README)
This repository contains the complete codebase for Autonomous Water Cleaning Robot, including:

•	📱 App Code – Mobile / Web application source code

•	🧪 Simulation Code – Simulations for testing and validation

•	💻 Software Code – Core logic, algorithms, and system software

This project addresses the problem by developing an autonomous water surface cleaning robot using ROS 2–based navigation, RTK positioning, and a mechanical collection system to enable safe, efficient, and sustainable water surface cleaning.
<<<<<<< HEAD
________________________________________

🧩 Repository Structure

=======

## Repository Structure
>>>>>>> 5e87aa7 (Updated README)
📦 water-surface-cleaning-robot
├── App/
│   ├── android/                 
│   ├── ios/                     
│   ├── web/                   
│   ├── windows/                 
│   ├── macos/                 
│   ├── linux/                   
│   ├── lib/                     
│   ├── assets/                  
│   ├── test/                    
│   ├── analysis_options.yaml    
│   ├── pubspec.yaml             
│   ├── pubspec.lock             
│   └── README.md                
│
├── Joystick_Control/
│   └── simulation_2/
│       ├── boat_1/             
│       ├── environment/         
│       ├── src/
│       │   └── boat_control/    
│       ├── world.sdf            
│       └── README.md            
│
├── Lidar_mapping/
│   ├── boat_1/                  
│   ├── environment/             
│   ├── maps/                    
│   ├── src/
│   │   └── boat_control/        
│   ├── fastdds_no_shm.xml       
│   ├── world.sdf                
│   └── README.md                
│
├── Path_Planning/
│   └── src/
│       ├── boat_1/              
│       ├── robot_mission_node/  
│       └── README.md            
│
├── Server/
│   └── backend/
│       ├── routes/              
│       ├── server.js            
│       ├── websocket_server.js  
│       └── README.md            
│
<<<<<<< HEAD
└── README.md                   

________________________________________

⚙️ Technologies Used

•	Programming Languages:

o	Python

o	C / C++

o	 Dart 

o	JavaScript 

o	Swift

•	Frameworks & Tools:

o	ROS 2

o	Flutter / Android SDK

o	Gazebo 

o	Git & GitHub
________________________________________
🧪 Simulation Details

=======
└── README.md       

## ⚙️ Technologies Used
•	Programming Languages:
•	Python
•	C / C++
•	 Dart 
•	JavaScript 
•	Swift
•	Frameworks & Tools:
•	ROS 2 (Humble)
•	Flutter / Android SDK
•	Gazebo 
•	Git & GitHub

## 🧪 Simulation Details
>>>>>>> 5e87aa7 (Updated README)
The simulation module is used to:

•	Test system behavior before real-world deployment

•	Validate algorithms and control logic

•	Reduce hardware dependency during development

Simulation Tools Used:

•	Gazebo 

<<<<<<< HEAD
📱 App Details

=======
## 📱 App Details
>>>>>>> 5e87aa7 (Updated README)
The application provides:

•	User interface for interaction

•	Real-time data visualization

•	Control and monitoring features

Platform:
•	Android / iOS / Web 
<<<<<<< HEAD
________________________________________

🛠️ Software Module

=======


## 🛠️ Software Module
>>>>>>> 5e87aa7 (Updated README)
The software layer handles:

•	Core logic and algorithms

•	Sensor data processing

•	Communication between modules
<<<<<<< HEAD
________________________________________

🚀 How to Run

1️⃣ Clone the Repository
git clone https://github.com/Shrutiii-04/Water_surface_cleaning_robot

2️⃣ Run App Code
cd:into repository folder, flutter pub get , flutter run

3️⃣ Run Simulation
cd: ign gazebo world.sdf -v 4

4️⃣ Run Software Code
ros2 launch robot_mission_node path_planning.launch.py
ros2 launch boat_1 rviz.launch.py


________________________________________
📊 Features

=======

## 🚀 How to Run
1️⃣ Clone the Repository
`git clone https://github.com/Shrutiii-04/Water_surface_cleaning_robot
2️⃣ Run App Code
`cd:into repository folder, flutter pub get , flutter run
3️⃣ Run Simulation
`cd: ign gazebo world.sdf -v 4
4️⃣ Run Software Code
`ros2 launch robot_mission_node path_planning.launch.py
`ros2 launch boat_1 rviz.launch.py

## 📊 Features
>>>>>>> 5e87aa7 (Updated README)
•	Modular architecture

•	Easy to extend and modify

•	Supports simulation + real implementation

•	Well-documented code

<<<<<<< HEAD
________________________________________

📈 Future Improvements

=======
## 📈 Future Improvements
>>>>>>> 5e87aa7 (Updated README)
•	Add more simulations

•	Improve UI/UX

•	Optimize performance

•	Hardware integration

<<<<<<< HEAD
________________________________________

📄 License

This project is licensed under the MIT License

________________________________________

🙋‍♀️ Author

=======
## 📄 License
This project is licensed under the MIT License

## 🙋‍♀️ Author
Team Innovex
>>>>>>> 5e87aa7 (Updated README)
📧 Email: shrutipatil0880@gmail.com

🔗 GitHub: https://github.com/Shrutiii-04
