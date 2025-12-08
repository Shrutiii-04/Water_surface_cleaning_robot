# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():

#     pkg_path = get_package_share_directory('boat_1')
#     urdf = os.path.join(pkg_path, 'urdf', 'boat_1.urdf')

#     return LaunchDescription([

#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[{'robot_description': open(urdf).read()}]
#         ),

#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui'
#         ),

#         Node(
#             package='rviz2',
#             executable='rviz2'
#         )
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boat_1')
    urdf = os.path.join(pkg, 'urdf', 'boat_1.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_ign_gazebo', 'create', '-file', urdf, '-name', 'boat_1'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf).read()}],
            output='screen'
        ),
    ])
