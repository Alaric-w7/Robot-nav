import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'elderbot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('lib', package_name), ['elderbot_bringup/libcontrolcan.so']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elderbot',
    maintainer_email='elderbot@todo.todo',
    description='ElderBot Bringup - hardware drivers and launch files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf = elderbot_bringup.static_tf:main',
            'can_driver = elderbot_bringup.can_driver:main',
            'laser_filter = elderbot_bringup.laser_filter:main',
            'depth_scan_filter = elderbot_bringup.depth_scan_filter:main',
            'initial_pose_pub = elderbot_bringup.initial_pose_pub:main',
            'navigation_test = elderbot_bringup.navigation_test:main',
            'battery_monitor = elderbot_bringup.battery_monitor:main',
            'ultrasonic_memory = elderbot_bringup.ultrasonic_memory:main',
        ],
    },
)
