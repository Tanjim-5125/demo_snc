from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_robot_challenge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='rmitaiil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_rsequire=['pytest'],
    entry_points={
        'console_scripts': [
            f"nav_wall_follow = aiil_rosbot_demo.nav_wall_follow:main",
        ],
    },
)
