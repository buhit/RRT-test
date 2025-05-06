from setuptools import setup
import os
from glob import glob

package_name = 'rrt_star_turtlesim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='buhitmon123',
    maintainer_email='buhitmon123@todo.todo',
    description='RRT* path planning for turtlesim',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_star_turtlesim_node = rrt_star_turtlesim.rrt_star_turtlesim_node:main',
        ],
    },
)
