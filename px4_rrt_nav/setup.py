import os
from glob import glob
from setuptools import setup

package_name = 'px4_rrt_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # Include rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosusr',
    maintainer_email='user@todo.todo',
    description='PX4 Offboard Control with RRT Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_planner = px4_rrt_nav.rrt_planner:main',
            'offboard_control = px4_rrt_nav.offboard_control:main',
        ],
    },
)