"""
Mapping Launch File
Launches all nodes needed for LIDAR-based autonomous mapping.
This launch file brings up:
  - Gazebo-ROS bridge
  - Pose to TF converter
  - SLAM Toolbox for mapping
  - Static TF publishers for sensor frames
  - LIDAR Explorer for obstacle-aware autonomous exploration
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('boat_control')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    bridge_launch = os.path.join(pkg_share, 'launch', 'boat_bridge.launch.py')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch)
        ),

        
        Node(
            package='boat_control',
            executable='pose_to_tf',
            name='pose_to_tf',
            output='screen'
        ),

        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            remappings=[('/scan', '/scan')],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'boat_1/lidar_link', 'boat_1/lidar_link/front_lidar'],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_link_tf',
            arguments=['0.8', '0', '0.8', '0', '0', '0',
                       'boat_1/hull_1', 'boat_1/lidar_link'],
            output='screen'
        ),

        
        Node(
            package='boat_control',
            executable='lidar_explorer',
            name='lidar_explorer',
            output='screen',
            parameters=[{
                'forward_thrust': 50.0,
                'turn_thrust': 40.0,
                'obstacle_distance': 3.0,  
                'critical_distance': 1.5,  
            }]
        ),
    ])
