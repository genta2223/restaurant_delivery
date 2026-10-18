import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'restaurant_delivery'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models', 'running_child'), glob(os.path.join('models', 'running_child', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Student',
    maintainer_email='user@example.com',
    description='Restaurant delivery robot simulation package with dynamic obstacle avoidance and emergency alert system for ROS 2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_controller = restaurant_delivery.delivery_controller:main',
            'obstacle_monitor = restaurant_delivery.obstacle_monitor:main',
            'child_spawner = restaurant_delivery.child_spawner:main',
            'debug_recorder = restaurant_delivery.debug_recorder:main',
        ],
    },
)
