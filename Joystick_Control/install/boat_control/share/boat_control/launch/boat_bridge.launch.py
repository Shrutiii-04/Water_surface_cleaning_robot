from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Left Thruster (Notice the comma at the end!)
                '/boat_1/left_thrust_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                
                # Right Thruster (Notice the comma at the end!)
                '/boat_1/right_thrust_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                
                # LIDAR
                '/lidar_points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
            ],
            output='screen'
        )
    ])