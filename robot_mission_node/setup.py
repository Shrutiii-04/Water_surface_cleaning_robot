from setuptools import find_packages, setup

package_name = 'robot_mission_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shruti',
    maintainer_email='shruti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'mission_to_path = robot_mission_node.mission_to_path:main',
        'path_follower = robot_mission_node.path_follower:main',
        'robot_marker = robot_mission_node.robot_marker:main', 
        'fake_odom = robot_mission_node.fake_odom:main',
        'fake_lidar = robot_mission_node.fake_lidar:main', 
        'bcd_planner = robot_mission_node.bcd_planner:main',
        'obstacle_marker = robot_mission_node.obstacle_marker:main',
        'mqtt_node = robot_mission_node.mqtt_node:main',

        ],
    },
)

