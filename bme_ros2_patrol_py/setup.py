from setuptools import find_packages, setup

package_name = 'bme_ros2_patrol_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deadend',
    maintainer_email='deadend@example.com',
    description='Autonomous waypoint patrol using Nav2 FollowWaypoints action',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_patroller = bme_ros2_patrol_py.waypoint_patroller:main',
        ],
    },
)
