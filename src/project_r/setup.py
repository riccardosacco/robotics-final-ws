from setuptools import setup

import os
from glob import glob

package_name = 'project_r'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='riccardosacco,rkalvitis',
    maintainer_email='riccardo.sacco@usi.ch,r.kalvitis@usi.ch',
    description='ROS2 robot controller implementing autonomous navigation with obstacle avoidance and fire detection using computer vision',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ('main_node = ' + package_name + '.main_controller:main')
        ],
    },
)
