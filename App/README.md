# water_surface_robot_app

**Water Surface Cleaning Robot App**

A Flutter-based mobile application designed to control, monitor, and interact with an autonomous water surface cleaning robot.
The app provides real-time control, mapping, telemetry, cloud integration, and remote communication capabilities.


**Features**

•	Joystick-based robot control

•	Real-time communication using MQTT & WebSockets

•	Cloud storage & database using Firebase

•	User authentication

•	Embedded web views for dashboards or analytics


**Tech Stack**

•	Frontend: Flutter (Dart)

•	Backend / Cloud: Firebase

•	Communication: MQTT, WebSockets

•	Maps: Google Maps API


 **Dependencies**
 
•	Core Flutter

•	flutter – Flutter SDK


**Control & Interaction**

•	flutter_joystick – Joystick UI for robot control


**Networking & Communication**

•	mqtt_client – MQTT communication with robot

•	web_socket_channel – Real-time WebSocket communication

•	webview_flutter – Display web dashboards inside the app


**Firebase Services**

•	firebase_core – Firebase initialization

•	firebase_auth – User authentication

•	cloud_firestore – Cloud database

•	firebase_storage – Store images and files


**Maps & Location**

•	google_maps_flutter – Live robot location tracking


**Utilities**

•	path_provider – Access device storage

•	share_plus – Share files and images

•	intl – Date & time formatting

•	image – Image processing and manipulation


**Development & Testing**

•	flutter_test – Flutter testing framework

•	flutter_lints – Recommended lint rules


**Assets**

assets/
 └── robot_icon.png


Ensure assets are properly declared in pubspec.yaml.


**Environment**

•	Dart SDK: ^3.10.1

•	Flutter: Latest stable version recommended


**A. Getting Started**

1) Clone the Repository
   
    git clone https://github.com/your-username/water_surface_robot_app.git

    cd water_surface_robot_app

2) Install Dependencies
   
    flutter pub get

3) Configure Firebase

    a)	Create a Firebase project

    b)	Add Android/iOS app

    c)	Download and place:

          google-services.json (Android)

          GoogleService-Info.plist (iOS)

4) Run the App
   
    flutter run


**Use Cases**

•	Autonomous water surface cleaning

•	Remote robot monitoring

•	Environmental data collection

•	Research & academic robotics projects


**Developed By**

Team Innovex

Flutter + Robotics
