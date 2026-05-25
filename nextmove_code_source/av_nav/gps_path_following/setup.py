from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zoe_gps_waypoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('zoe_gps_waypoint/launch/*.py')),
(os.path.join('share', package_name, 'config'), glob('zoe_gps_waypoint/config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cohoma',
    maintainer_email='maxime.duquesne@polytech-lille.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'gps_to_XY_point = zoe_gps_waypoint.gps_to_XY_point:main',
               'XY_follow = zoe_gps_waypoint.XY_follow:main',
               'waypoint_map_loader = zoe_gps_waypoint.map_loader:main',
        ],
    },
)
