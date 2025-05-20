from setuptools import setup
import os
from glob import glob

package_name = 'explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RoboMaster exploration package for finding visual markers in unknown environments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = explorer.explorer_node:main',
        ],
    },
)
