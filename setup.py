from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_challenge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='',
    description='Challenge package for ROSBot navigation and return',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_nav = my_robot_challenge_pkg.occupancy_nav:main',
        ],
    },
)