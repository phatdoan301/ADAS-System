import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'ADAS_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phat',
    maintainer_email='phat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detect = ADAS_system.obstacle_detect:main',
            'lane_detection  = ADAS_system.lane_detection:main',
            'CAN_send        = ADAS_system.CAN_send:main'
        ],
    },
)
