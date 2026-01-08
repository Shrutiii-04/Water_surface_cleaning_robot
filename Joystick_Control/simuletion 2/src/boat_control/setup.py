from setuptools import setup
import os
from glob import glob

package_name = 'boat_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pk',
    maintainer_email='pk@todo.todo',
    description='Boat control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = boat_control.boat_teleop:main',
            'robot_ws_client = boat_control.robot_ws_client:main',
            'robot_ws_joy_to_cmdvel = boat_control.robot_ws_joy_to_cmdvel:main',
            'cmd_vel_to_thrusters = boat_control.cmd_vel_to_thrusters:main',
            'auto_wander = boat_control.auto_wander:main',
            'pose_to_tf = boat_control.pose_to_tf:main',
            'lidar_explorer = boat_control.lidar_explorer:main',
            'absolute_mapper = boat_control.absolute_mapper:main',
            'auto_explorer = boat_control.auto_explorer:main',
            'smart_explorer = boat_control.smart_explorer:main',
            'dwa_explorer = boat_control.dwa_explorer:main',
    ],
    },
)