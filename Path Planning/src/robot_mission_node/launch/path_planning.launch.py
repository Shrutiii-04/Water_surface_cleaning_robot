from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    boat_pkg = get_package_share_directory('boat_1')
    urdf_path = os.path.join(boat_pkg, 'urdf', 'boat_1.urdf')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),
        Node(
            package='robot_mission_node',
            executable='bcd_planner',
            name='bcd_planner',
            output='screen'
        ),
        Node(
            package='robot_mission_node',
            executable='mqtt_node',
            name='mqtt_node',
            output='screen'
        ),
        Node(
            package='robot_mission_node',
            executable='fake_odom',
            name='fake_odom',
            output='screen'
        ),

        Node(
            package='robot_mission_node',
            executable='path_follower',
            name='path_follower',
            output='screen'
        ),
        Node(
            package='robot_mission_node',
            executable='local_to_geo',
            name='local_to_geo',
            output='screen'
        ),
        Node(
            package='robot_mission_node',
            executable='mqtt_robot_location',
            name='mqtt_robot_laocation',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
