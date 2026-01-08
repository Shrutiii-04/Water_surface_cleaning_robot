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
                
                '/boat_1/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
            
            ],
            output='screen'
        )
    ])