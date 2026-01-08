from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('boat_1')
    urdf_file = os.path.join(pkg_path, 'urdf', 'boat_1.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'boat_1',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/lidar_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        ),
    ])