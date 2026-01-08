import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_boat_control = get_package_share_directory('boat_control')


    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_boat_control, 'launch', 'boat_bridge.launch.py')
        )
    )

 
    ws_client_node = Node(
        package='boat_control',
        executable='robot_ws_client',
        output='screen'
    )

   
    joy_node = Node(
        package='boat_control',
        executable='robot_ws_joy_to_cmdvel',
        output='screen'
    )


    thruster_node = Node(
        package='boat_control',
        executable='cmd_vel_to_thrusters',
        output='screen'
    )

    return LaunchDescription([
        bridge_launch,
        ws_client_node,
        joy_node,
        thruster_node
    ])