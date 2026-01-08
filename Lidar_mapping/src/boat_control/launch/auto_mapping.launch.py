"""
Automatic Exploration Launch File

Runs the boat in automatic mode to explore and map the entire terrain.
The boat will navigate autonomously, avoiding obstacles and mapping as it goes.

Usage:
  ros2 launch boat_control auto_mapping.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('boat_control')
    rviz_config = os.path.join(pkg_share, 'config', 'mapping.rviz')
    bridge_launch = os.path.join(pkg_share, 'launch', 'boat_bridge.launch.py')

    
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        
        ExecuteProcess(
            cmd=['bash', '-c', 'rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true'],
            output='screen'
        ),
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch)
        ),

        
        Node(
            package='boat_control',
            executable='absolute_mapper',
            name='absolute_mapper',
            parameters=[
                use_sim_time,
                {
                    'resolution': 0.1,          
                    'map_size': 150.0,          
                    'origin_x': -75.0,          
                    'origin_y': -75.0,
                    'save_dir': '/home/vedanti/simuletion/maps',
                    'save_interval': 30.0,      
                    'obstacle_threshold': 3,    
                    'free_threshold': 2,        
                    'lidar_offset_x': 0.8,      
                }
            ],
            output='screen'
        ),

        
        
        Node(
            package='boat_control',
            executable='dwa_explorer',
            name='dwa_explorer',
            parameters=[
                use_sim_time,
                {
                    
                    'max_thrust': 30.0,
                    
                    
                    'trajectory_time': 5.0,       
                    
                    
                    'goal_weight': 1.0,           
                    'obstacle_weight': 8.0,       
                    'speed_weight': 0.5,          
                    'drift_weight': 1.0,          
                    
                    
                    'obstacle_distance': 8.0,     
                    'critical_distance': 3.0,     
                    
                    
                    'exploration_radius': 40.0,
                    'goal_tolerance': 4.0,        
                }
            ],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', 
                       '--child-frame-id', 'world'],
            parameters=[use_sim_time],
            output='screen'
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
            arguments=['--x', '0.8', '--y', '0', '--z', '1.0',
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[use_sim_time],
            output='screen'
        ),
    ])
