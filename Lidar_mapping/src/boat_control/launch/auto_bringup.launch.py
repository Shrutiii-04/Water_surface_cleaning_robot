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
            executable='auto_wander',
            name='auto_wander',
            output='screen'
        ),
    ])
