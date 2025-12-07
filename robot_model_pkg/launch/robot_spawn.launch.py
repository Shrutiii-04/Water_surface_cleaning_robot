from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
import os, tempfile

def generate_launch_description():
    pkg_path = os.path.expanduser("~/water_cleaning_robot/src/robot_model_pkg")
    xacro_path = os.path.join(pkg_path, "urdf", "my_robot.urdf.xacro")

    # Convert Xacro → URDF
    urdf_xml = os.popen(f"xacro {xacro_path}").read()

    # Save URDF temporarily
    urdf_path = os.path.join(tempfile.gettempdir(), "water_bot.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf_xml)

    # Set Gazebo resource path
    set_env = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", pkg_path)

    # Launch Gazebo Harmonic world
    launch_gz = ExecuteProcess(
        cmd=["gz", "sim", os.path.join(pkg_path, "worlds", "cleaning_world.sdf")],
        output="screen"
    )

    # Spawn Robot in Harmonic
    spawn_robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-name", "water_cleaner_bot",
             "-file", urdf_path,
             "-x", "0", "-y", "0", "-z", "0.1"],
        output="screen"
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_xml}],
        output="screen"
    )

    # LiDAR Bridge (GZ → ROS2)
    bridge = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        output="screen"
    )

    return LaunchDescription([ set_env, launch_gz, rsp, spawn_robot, bridge ])
