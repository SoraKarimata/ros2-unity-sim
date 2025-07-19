from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_rover_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = nav_rover_control.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav_rover_control.interactive_waypoint_follower:main',
            'odometry = nav_rover_control.odometry:main',
            'rover_control = nav_rover_control.rover_control:main',
            'sensor = nav_rover_control.sensor:main',
            'rover_control_pub = nav_rover_control.rover_control_pub:main',
            'gps_waypoint_logger = nav_rover_control.gps_waypoint_logger:main'
        ],
    },
)
