from setuptools import find_packages, setup

package_name = 'robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='terzi',
    maintainer_email='terziogluyunus@gmail.com',
    description='Contains two controllers for turtlesim and ros2_ur5_interface for generating control commands.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = robot_controllers.turtle_controller:main',
            'ur_controller = robot_controllers.ur_controller:main'
        ],
    },
)
