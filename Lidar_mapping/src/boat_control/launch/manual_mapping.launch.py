"""
Manual Mapping Launch File
Launches all nodes needed for LIDAR-based mapping WITH MANUAL CONTROL.
Use teleop node separately to control the boat while mapping.
Automatically opens RViz with the correct configuration.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('boat_control')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'mapping.rviz')
    bridge_launch = os.path.join(pkg_share, 'launch', 'boat_bridge.launch.py')

    
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch)
        ),

        
        Node(
            package='boat_control',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[use_sim_time],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_link_tf',
            arguments=['--x', '0.8', '--y', '0', '--z', '0.8',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'boat_1/hull_1', 
                       '--child-frame-id', 'boat_1/lidar_link'],
            parameters=[use_sim_time],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'boat_1/lidar_link', 
                       '--child-frame-id', 'boat_1/lidar_link/front_lidar'],
            parameters=[use_sim_time],
            output='screen'
        ),

        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, use_sim_time],
            remappings=[('/scan', '/scan')],
            output='screen'
        ),

        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[use_sim_time],
            output='screen'
        ),

        
    ])
