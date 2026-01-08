from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                
                '/boat_1/left_thrust_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                
                
                '/boat_1/right_thrust_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                
                
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',

                
                '/boat_1/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                
                
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen'
        ),
    ])