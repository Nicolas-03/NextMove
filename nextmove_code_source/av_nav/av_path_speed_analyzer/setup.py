from setuptools import find_packages, setup

package_name = 'av_path_speed_analyzer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pierre Malet',
    maintainer_email='pierre.malet@ig2i.centralelille.fr',
    description='Contains differents methods for speed calculation relative to trajectory',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "speed_analyzer = src.speed_analyzer:main",
            "obs_detect = src.obstacle_detect:main",
            "genProfile = src.profile_generator:main"
        ],
    },
)
